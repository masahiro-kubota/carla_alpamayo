from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from ad_stack.agents import (
    ExpertBasicAgent,
    ExpertBasicAgentConfig,
    LearnedLateralAgent,
    PilotNetScenePolicy,
)
from ad_stack.overtake import build_stopped_obstacle_targets
from ad_stack.overtake.policies import accept_stopped_overtake_target
from ad_stack.agents.base import ControlDecision, VehicleCommand
from ad_stack.inference import load_pilotnet_runtime, select_device
from ad_stack.runtime import ObservationBuilder
from ad_stack.world_model import (
    DynamicVehicleStateView,
    EgoState,
    RouteState,
    TrafficLightStateView,
)
from libs.carla_utils import (
    compute_local_target_point,
    ensure_carla_agents_on_path,
)


def _completion_ratio(route_index: int | None, route_point_count: int) -> float:
    if route_index is None or route_point_count <= 1:
        return 0.0
    bounded_index = max(0, min(route_index, route_point_count - 1))
    return max(0.0, min(1.0, bounded_index / (route_point_count - 1)))


def _smooth_steer(
    raw_steer: float,
    previous_steer: float | None,
    *,
    smoothing: float,
    max_delta: float | None,
) -> float:
    if previous_steer is None:
        applied = raw_steer
    else:
        applied = ((1.0 - smoothing) * previous_steer) + (smoothing * raw_steer)
    if previous_steer is not None and max_delta is not None:
        delta = applied - previous_steer
        if delta > max_delta:
            applied = previous_steer + max_delta
        elif delta < -max_delta:
            applied = previous_steer - max_delta
    return max(-1.0, min(1.0, applied))


def _actor_speed_mps(actor: Any) -> float:
    velocity = actor.get_velocity()
    return float((velocity.x**2 + velocity.y**2 + velocity.z**2) ** 0.5)


def _lane_id(waypoint: Any | None) -> str | None:
    if waypoint is None:
        return None
    return f"{waypoint.road_id}:{waypoint.lane_id}"


def _build_ego_state(
    vehicle_transform: Any,
    *,
    speed_mps: float,
    speed_limit_mps: float | None,
    lane_id: str | None,
    adjacent_lanes_open: dict[str, bool],
) -> EgoState:
    vehicle_location = vehicle_transform.location
    vehicle_rotation = vehicle_transform.rotation
    return EgoState(
        x_m=vehicle_location.x,
        y_m=vehicle_location.y,
        yaw_deg=vehicle_rotation.yaw,
        speed_mps=speed_mps,
        lane_id=lane_id,
        speed_limit_mps=speed_limit_mps,
        adjacent_lanes_open=adjacent_lanes_open,
    )


def to_carla_control(carla_module: Any, command: VehicleCommand) -> Any:
    return carla_module.VehicleControl(
        throttle=float(command.throttle),
        steer=float(command.steer),
        brake=float(command.brake),
        hand_brake=bool(command.hand_brake),
        reverse=bool(command.reverse),
        manual_gear_shift=False,
        gear=0,
    )


@dataclass(slots=True)
class StackStepResult:
    decision: ControlDecision
    expert_decision: ControlDecision | None = None
    applied_steer: float | None = None
    behavior: str | None = None
    progress_ratio: float = 0.0
    done: bool = False
    route_point: tuple[float, float] | None = None
    scene_state: Any | None = None


@dataclass(slots=True)
class StackDescription:
    model_name: str | None = None


@dataclass(slots=True)
class _SpeedController:
    target_speed_kmh: float
    throttle_gain: float = 0.06
    brake_gain: float = 0.08
    derivative_gain: float = 0.01
    max_throttle: float = 0.65
    max_brake: float = 0.5
    previous_error: float = 0.0

    def step(self, current_speed_mps: float) -> tuple[float, float]:
        current_speed_kmh = current_speed_mps * 3.6
        error = self.target_speed_kmh - current_speed_kmh
        derivative = error - self.previous_error
        self.previous_error = error
        if error >= 0.0:
            throttle = min(
                self.max_throttle,
                max(0.0, (self.throttle_gain * error) + (self.derivative_gain * derivative)),
            )
            return throttle, 0.0
        brake = min(self.max_brake, max(0.0, self.brake_gain * (-error)))
        return 0.0, brake


class _RouteSceneMixin:
    _MAX_ACTIVE_TRAFFIC_LIGHT_DISTANCE_M = 45.0

    def __init__(
        self,
        *,
        vehicle: Any,
        world_map: Any,
        planned_trace: list[tuple[Any, Any]],
        route_id: str,
        town_id: str,
        target_speed_kmh: float,
        route_geometry: Any | None = None,
        route_lookahead_m: float = 8.0,
        route_target_normalization_m: float = 20.0,
    ) -> None:
        ensure_carla_agents_on_path()
        from agents.tools.misc import get_trafficlight_trigger_location, is_within_distance

        self._vehicle = vehicle
        self._world = vehicle.get_world()
        self._map = world_map
        self.route_id = route_id
        self.town_id = town_id
        self.target_speed_mps = target_speed_kmh / 3.6
        self._planned_trace = list(planned_trace)
        self._route_geometry = route_geometry
        self._route_lookahead_m = route_lookahead_m
        self._route_target_normalization_m = route_target_normalization_m
        self._route_index_cursor: int | None = None
        self._max_route_index: int = 0
        self._observation_builder = ObservationBuilder()
        self._get_trafficlight_trigger_location = get_trafficlight_trigger_location
        self._is_within_distance = is_within_distance
        self._light_trigger_cache: dict[int, Any] = {}
        self._point_to_trace_index = self._build_route_index_mapping()

    def _build_route_index_mapping(self) -> list[int]:
        mapping: list[int] = []
        last_point: tuple[float, float] | None = None
        for trace_index, (waypoint, _option) in enumerate(self._planned_trace):
            location = waypoint.transform.location
            point = (location.x, location.y)
            if (
                last_point is None
                or ((point[0] - last_point[0]) ** 2 + (point[1] - last_point[1]) ** 2) ** 0.5 > 0.05
            ):
                mapping.append(trace_index)
                last_point = point
        return mapping

    def _route_trace_index(self, route_index: int | None) -> int | None:
        if route_index is None or not self._point_to_trace_index:
            return None
        bounded_index = max(0, min(route_index, len(self._point_to_trace_index) - 1))
        return self._point_to_trace_index[bounded_index]

    def _route_lane_id(self, route_index: int | None) -> str | None:
        trace_index = self._route_trace_index(route_index)
        if trace_index is None:
            return None
        waypoint, _option = self._planned_trace[trace_index]
        return _lane_id(waypoint)

    def _adjacent_lanes_open(self, ego_waypoint: Any | None) -> dict[str, bool]:
        if ego_waypoint is None:
            return {"left": False, "right": False}
        left_waypoint = ego_waypoint.get_left_lane()
        right_waypoint = ego_waypoint.get_right_lane()
        return {
            "left": bool(
                left_waypoint is not None
                and left_waypoint.lane_type == self._carla.LaneType.Driving
            ),
            "right": bool(
                right_waypoint is not None
                and right_waypoint.lane_type == self._carla.LaneType.Driving
            ),
        }

    def _lane_relation(self, ego_waypoint: Any | None, target_waypoint: Any | None) -> str:
        if ego_waypoint is None or target_waypoint is None:
            return "unknown"
        if (
            target_waypoint.road_id == ego_waypoint.road_id
            and target_waypoint.lane_id == ego_waypoint.lane_id
        ):
            return "same_lane"
        left_waypoint = ego_waypoint.get_left_lane()
        if (
            left_waypoint is not None
            and left_waypoint.lane_type == self._carla.LaneType.Driving
            and target_waypoint.road_id == left_waypoint.road_id
            and target_waypoint.lane_id == left_waypoint.lane_id
        ):
            return "left_lane"
        right_waypoint = ego_waypoint.get_right_lane()
        if (
            right_waypoint is not None
            and right_waypoint.lane_type == self._carla.LaneType.Driving
            and target_waypoint.road_id == right_waypoint.road_id
            and target_waypoint.lane_id == right_waypoint.lane_id
        ):
            return "right_lane"
        return "unknown"

    def _build_tracked_objects(
        self, vehicle_transform: Any, ego_waypoint: Any | None
    ) -> tuple[DynamicVehicleStateView, ...]:
        ego_location = vehicle_transform.location
        forward_vector = vehicle_transform.get_forward_vector()
        right_vector = vehicle_transform.get_right_vector()
        tracked_objects: list[DynamicVehicleStateView] = []
        for actor in self._world.get_actors().filter("*vehicle*"):
            try:
                if actor.id == self._vehicle.id:
                    continue
                if hasattr(actor, "is_alive") and not actor.is_alive:
                    continue
                actor_transform = actor.get_transform()
                actor_location = actor_transform.location
                if actor_location.distance(ego_location) > 60.0:
                    continue
                target_waypoint = self._map.get_waypoint(
                    actor_location, lane_type=self._carla.LaneType.Driving
                )
                dx = actor_location.x - ego_location.x
                dy = actor_location.y - ego_location.y
                longitudinal_distance_m = (dx * forward_vector.x) + (dy * forward_vector.y)
                lateral_distance_m = (dx * right_vector.x) + (dy * right_vector.y)
                tracked_objects.append(
                    DynamicVehicleStateView(
                        actor_id=int(actor.id),
                        x_m=float(actor_location.x),
                        y_m=float(actor_location.y),
                        yaw_deg=float(actor_transform.rotation.yaw),
                        speed_mps=_actor_speed_mps(actor),
                        lane_id=_lane_id(target_waypoint),
                        relation=self._lane_relation(ego_waypoint, target_waypoint),
                        longitudinal_distance_m=float(longitudinal_distance_m),
                        lateral_distance_m=float(lateral_distance_m),
                        is_ahead=bool(longitudinal_distance_m > 0.0),
                    )
                )
            except RuntimeError as exc:
                if "destroyed actor" in str(exc):
                    continue
                raise
        return tuple(
            sorted(tracked_objects, key=lambda actor: abs(actor.longitudinal_distance_m or 0.0))
        )

    def _build_traffic_lights(
        self, vehicle_transform: Any, ego_waypoint: Any | None
    ) -> tuple[TrafficLightStateView, ...]:
        if ego_waypoint is None:
            return ()
        ego_location = vehicle_transform.location
        ego_forward = vehicle_transform.get_forward_vector()
        ego_right = vehicle_transform.get_right_vector()
        traffic_lights: list[TrafficLightStateView] = []
        for traffic_light in self._world.get_actors().filter("*traffic_light*"):
            trigger_waypoint = self._light_trigger_cache.get(int(traffic_light.id))
            if trigger_waypoint is None:
                trigger_location = self._get_trafficlight_trigger_location(traffic_light)
                trigger_waypoint = self._map.get_waypoint(trigger_location)
                self._light_trigger_cache[int(traffic_light.id)] = trigger_waypoint
            trigger_location = trigger_waypoint.transform.location
            distance_m = float(trigger_location.distance(ego_location))
            if distance_m > 80.0:
                continue

            affects_ego = False
            stop_line_distance_m: float | None = None
            if (
                trigger_waypoint.road_id == ego_waypoint.road_id
                and trigger_waypoint.lane_id == ego_waypoint.lane_id
            ):
                trigger_forward = trigger_waypoint.transform.get_forward_vector()
                dot = (
                    (ego_forward.x * trigger_forward.x)
                    + (ego_forward.y * trigger_forward.y)
                    + (ego_forward.z * trigger_forward.z)
                )
                dx = trigger_location.x - ego_location.x
                dy = trigger_location.y - ego_location.y
                longitudinal_distance_m = (dx * ego_forward.x) + (dy * ego_forward.y)
                lateral_distance_m = (dx * ego_right.x) + (dy * ego_right.y)
                if dot >= 0.0 and longitudinal_distance_m > 0.0 and abs(lateral_distance_m) <= 4.0:
                    stop_line_distance_m = float(longitudinal_distance_m)
                    affects_ego = stop_line_distance_m <= self._MAX_ACTIVE_TRAFFIC_LIGHT_DISTANCE_M

            state_name = str(traffic_light.state).split(".")[-1].lower()
            if state_name not in {"red", "yellow", "green"}:
                state_name = "unknown"
            traffic_lights.append(
                TrafficLightStateView(
                    actor_id=int(traffic_light.id),
                    state=state_name,
                    distance_m=distance_m,
                    affects_ego=affects_ego,
                    stop_line_distance_m=stop_line_distance_m,
                )
            )
        return tuple(sorted(traffic_lights, key=lambda light: light.distance_m))

    def _build_scene(
        self,
        *,
        timestamp_s: float,
        vehicle_transform: Any,
        speed_mps: float,
        current_route_command: str,
        metadata: dict[str, Any] | None = None,
    ) -> tuple[Any, tuple[float, float] | None, float]:
        route_point: tuple[float, float] | None = None
        route_index: int | None = None
        if self._route_geometry is not None:
            vehicle_location = vehicle_transform.location
            route_point, route_index = compute_local_target_point(
                self._route_geometry,
                vehicle_x=vehicle_location.x,
                vehicle_y=vehicle_location.y,
                vehicle_yaw_deg=vehicle_transform.rotation.yaw,
                lookahead_m=self._route_lookahead_m,
                target_normalization_m=self._route_target_normalization_m,
                previous_route_index=self._route_index_cursor,
            )
            self._route_index_cursor = route_index
            if route_index is not None:
                self._max_route_index = max(self._max_route_index, route_index)

        ego_waypoint = self._map.get_waypoint(
            vehicle_transform.location, lane_type=self._carla.LaneType.Driving
        )
        scene_state = self._observation_builder.build(
            timestamp_s=timestamp_s,
            town_id=self.town_id,
            ego=_build_ego_state(
                vehicle_transform,
                speed_mps=speed_mps,
                speed_limit_mps=self.target_speed_mps,
                lane_id=_lane_id(ego_waypoint),
                adjacent_lanes_open=self._adjacent_lanes_open(ego_waypoint),
            ),
            route=RouteState(
                route_id=self.route_id,
                maneuver=current_route_command,
                progress_ratio=_completion_ratio(
                    self._max_route_index if self._route_geometry is not None else None,
                    len(self._route_geometry.points) if self._route_geometry is not None else 0,
                ),
                target_speed_mps=self.target_speed_mps,
                target_lane_id=self._route_lane_id(route_index),
                route_index=route_index,
            ),
            tracked_objects=self._build_tracked_objects(vehicle_transform, ego_waypoint),
            traffic_lights=self._build_traffic_lights(vehicle_transform, ego_waypoint),
            metadata={"route_point": route_point, **dict(metadata or {})},
        )
        return scene_state, route_point, scene_state.route.progress_ratio


class ExpertCollectorStack(_RouteSceneMixin):
    def __init__(
        self,
        *,
        vehicle: Any,
        world_map: Any,
        planned_trace: list[tuple[Any, Any]],
        route_id: str,
        town_id: str,
        target_speed_kmh: float,
        ignore_traffic_lights: bool,
        ignore_stop_signs: bool,
        ignore_vehicles: bool,
        sampling_resolution_m: float,
        route_geometry: Any | None = None,
        route_lookahead_m: float = 8.0,
        route_target_normalization_m: float = 20.0,
        expert_config: ExpertBasicAgentConfig | None = None,
    ) -> None:
        import carla

        self._carla = carla
        super().__init__(
            vehicle=vehicle,
            world_map=world_map,
            planned_trace=planned_trace,
            route_id=route_id,
            town_id=town_id,
            target_speed_kmh=target_speed_kmh,
            route_geometry=route_geometry,
            route_lookahead_m=route_lookahead_m,
            route_target_normalization_m=route_target_normalization_m,
        )
        self._agent = ExpertBasicAgent(
            vehicle,
            world_map,
            config=expert_config
            or ExpertBasicAgentConfig(
                target_speed_kmh=target_speed_kmh,
                ignore_traffic_lights=ignore_traffic_lights,
                ignore_stop_signs=ignore_stop_signs,
                ignore_vehicles=ignore_vehicles,
                sampling_resolution_m=sampling_resolution_m,
            ),
            target_policy=build_stopped_obstacle_targets,
            target_acceptance_policy=accept_stopped_overtake_target,
        )
        self._agent.set_global_plan(planned_trace)

    def describe(self) -> StackDescription:
        return StackDescription(model_name=None)

    def run_step(
        self,
        *,
        timestamp_s: float,
        vehicle_transform: Any,
        speed_mps: float,
        metadata: dict[str, Any] | None = None,
    ) -> StackStepResult:
        current_route_command = self._agent.current_route_command()
        scene_state, route_point, progress_ratio = self._build_scene(
            timestamp_s=timestamp_s,
            vehicle_transform=vehicle_transform,
            speed_mps=speed_mps,
            current_route_command=current_route_command,
            metadata=metadata,
        )
        decision = self._agent.step(scene_state)
        return StackStepResult(
            decision=decision,
            behavior=decision.behavior,
            progress_ratio=progress_ratio,
            done=self._agent.done() or progress_ratio >= 0.995,
            route_point=route_point,
            scene_state=scene_state,
        )


class PilotNetEvalStack(_RouteSceneMixin):
    def __init__(
        self,
        *,
        vehicle: Any,
        world_map: Any,
        planned_trace: list[tuple[Any, Any]],
        route_id: str,
        town_id: str,
        target_speed_kmh: float,
        checkpoint_path: str | Path,
        device: str | None,
        ignore_traffic_lights: bool,
        ignore_stop_signs: bool,
        ignore_vehicles: bool,
        sampling_resolution_m: float,
        route_geometry: Any | None = None,
        expert_config: ExpertBasicAgentConfig | None = None,
    ) -> None:
        import carla

        self._carla = carla
        resolved_checkpoint_path = Path(checkpoint_path).resolve()
        runtime_device = select_device(device)
        self._runtime = load_pilotnet_runtime(resolved_checkpoint_path, runtime_device)
        super().__init__(
            vehicle=vehicle,
            world_map=world_map,
            planned_trace=planned_trace,
            route_id=route_id,
            town_id=town_id,
            target_speed_kmh=target_speed_kmh,
            route_geometry=route_geometry,
            route_lookahead_m=float(self._runtime.model_config.get("route_lookahead_m", 8.0)),
            route_target_normalization_m=float(
                self._runtime.model_config.get("route_target_normalization_m", 20.0)
            ),
        )
        self._rgb_history: deque[Any] = deque(maxlen=self._runtime.frame_stack)
        self._previous_applied_steer: float | None = None
        self._expert_agent = ExpertBasicAgent(
            vehicle,
            world_map,
            config=expert_config
            or ExpertBasicAgentConfig(
                target_speed_kmh=target_speed_kmh,
                ignore_traffic_lights=ignore_traffic_lights,
                ignore_stop_signs=ignore_stop_signs,
                ignore_vehicles=ignore_vehicles,
                sampling_resolution_m=sampling_resolution_m,
            ),
            target_policy=build_stopped_obstacle_targets,
            target_acceptance_policy=accept_stopped_overtake_target,
        )
        self._expert_agent.set_global_plan(planned_trace)
        self._latest_expert_decision: ControlDecision | None = None

        def expert_longitudinal_policy(scene_state) -> tuple[float, float]:
            expert_decision = self._expert_agent.step(scene_state)
            self._latest_expert_decision = expert_decision
            return expert_decision.command.throttle, expert_decision.command.brake

        self._agent = LearnedLateralAgent(
            lateral_policy=PilotNetScenePolicy(self._runtime),
            longitudinal_policy=expert_longitudinal_policy,
        )

    def describe(self) -> StackDescription:
        model_name = self._runtime.checkpoint.get("model_name")
        return StackDescription(model_name=str(model_name) if model_name is not None else None)

    def run_step(
        self,
        *,
        timestamp_s: float,
        vehicle_transform: Any,
        speed_mps: float,
        current_rgb: Any,
        steering_smoothing: float,
        max_steer_delta: float | None,
    ) -> StackStepResult:
        self._rgb_history.append(current_rgb)
        current_command = self._expert_agent.current_route_command()
        scene_state, route_point, progress_ratio = self._build_scene(
            timestamp_s=timestamp_s,
            vehicle_transform=vehicle_transform,
            speed_mps=speed_mps,
            current_route_command=current_command,
            metadata={
                "front_rgb_history": list(self._rgb_history),
                "command": current_command,
            },
        )
        decision = self._agent.step(scene_state)
        applied_steer = _smooth_steer(
            decision.command.steer,
            self._previous_applied_steer,
            smoothing=steering_smoothing,
            max_delta=max_steer_delta,
        )
        self._previous_applied_steer = applied_steer
        decision.command.steer = applied_steer
        return StackStepResult(
            decision=decision,
            expert_decision=self._latest_expert_decision,
            applied_steer=applied_steer,
            behavior=decision.behavior,
            progress_ratio=progress_ratio,
            done=self._expert_agent.done() or progress_ratio >= 0.995,
            route_point=route_point,
            scene_state=scene_state,
        )


class InteractivePilotNetController:
    def __init__(
        self,
        *,
        checkpoint_path: str | Path,
        device: str | None,
        target_speed_kmh: float,
    ) -> None:
        resolved_checkpoint_path = Path(checkpoint_path).resolve()
        runtime_device = select_device(device)
        self._runtime = load_pilotnet_runtime(resolved_checkpoint_path, runtime_device)
        if int(self._runtime.model_config.get("route_point_dim", 0)) > 0:
            raise ValueError("This app only supports checkpoints without route-point conditioning.")
        self._speed_controller = _SpeedController(target_speed_kmh=target_speed_kmh)
        self._rgb_history: deque[Any] = deque(maxlen=self._runtime.frame_stack)
        self._previous_applied_steer: float | None = None

    def describe(self) -> StackDescription:
        model_name = self._runtime.checkpoint.get("model_name")
        return StackDescription(model_name=str(model_name) if model_name is not None else None)

    def run_step(
        self,
        *,
        current_rgb: Any,
        speed_mps: float,
        command: str,
        steering_smoothing: float,
        max_steer_delta: float | None,
    ) -> StackStepResult:
        self._rgb_history.append(current_rgb)
        raw_steer = self._runtime.predict_steer(
            rgb_history=list(self._rgb_history),
            speed_mps=speed_mps,
            command=command,
            route_point=None,
        )
        applied_steer = _smooth_steer(
            raw_steer,
            self._previous_applied_steer,
            smoothing=steering_smoothing,
            max_delta=max_steer_delta,
        )
        self._previous_applied_steer = applied_steer
        throttle, brake = self._speed_controller.step(speed_mps)
        return StackStepResult(
            decision=ControlDecision(
                command=VehicleCommand(
                    steer=applied_steer,
                    throttle=throttle,
                    brake=brake,
                ).bounded(),
                behavior=command,
                planner_state="interactive_pilotnet_runtime",
            ),
            applied_steer=applied_steer,
            behavior=command,
        )


def create_expert_collector_stack(**kwargs) -> ExpertCollectorStack:
    return ExpertCollectorStack(**kwargs)


def create_pilotnet_eval_stack(**kwargs) -> PilotNetEvalStack:
    return PilotNetEvalStack(**kwargs)


def create_interactive_pilotnet_controller(**kwargs) -> InteractivePilotNetController:
    return InteractivePilotNetController(**kwargs)


__all__ = [
    "ControlDecision",
    "ExpertCollectorStack",
    "InteractivePilotNetController",
    "PilotNetEvalStack",
    "StackDescription",
    "StackStepResult",
    "VehicleCommand",
    "create_expert_collector_stack",
    "create_interactive_pilotnet_controller",
    "create_pilotnet_eval_stack",
    "to_carla_control",
]
