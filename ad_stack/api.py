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
from ad_stack.agents.base import ControlDecision, VehicleCommand
from ad_stack.inference import load_pilotnet_runtime, select_device
from ad_stack.runtime import ObservationBuilder
from ad_stack.world_model import EgoState, RouteState
from libs.carla_utils import compute_local_target_point


def _completion_ratio(initial_waypoint_count: int, remaining_count: int) -> float:
    if initial_waypoint_count <= 0:
        return 0.0
    progress = 1.0 - (remaining_count / initial_waypoint_count)
    return max(0.0, min(1.0, progress))


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


def _build_ego_state(
    vehicle_transform: Any,
    *,
    speed_mps: float,
    speed_limit_mps: float | None,
) -> EgoState:
    vehicle_location = vehicle_transform.location
    vehicle_rotation = vehicle_transform.rotation
    return EgoState(
        x_m=vehicle_location.x,
        y_m=vehicle_location.y,
        yaw_deg=vehicle_rotation.yaw,
        speed_mps=speed_mps,
        speed_limit_mps=speed_limit_mps,
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


class ExpertCollectorStack:
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
    ) -> None:
        self.route_id = route_id
        self.town_id = town_id
        self.target_speed_mps = target_speed_kmh / 3.6
        self._route_geometry = route_geometry
        self._route_lookahead_m = route_lookahead_m
        self._route_target_normalization_m = route_target_normalization_m
        self._route_index_cursor: int | None = None
        self._observation_builder = ObservationBuilder()
        self._agent = ExpertBasicAgent(
            vehicle,
            world_map,
            config=ExpertBasicAgentConfig(
                target_speed_kmh=target_speed_kmh,
                ignore_traffic_lights=ignore_traffic_lights,
                ignore_stop_signs=ignore_stop_signs,
                ignore_vehicles=ignore_vehicles,
                sampling_resolution_m=sampling_resolution_m,
            ),
        )
        self._agent.set_global_plan(planned_trace)
        self._initial_waypoint_count = self._agent.remaining_waypoints()
        if self._initial_waypoint_count <= 0:
            raise RuntimeError("Planner produced an empty local plan.")

    def _remaining_waypoints(self) -> int:
        return self._agent.remaining_waypoints()

    def _current_behavior(self) -> str:
        return self._agent.current_behavior()

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
        route_point: tuple[float, float] | None = None
        if self._route_geometry is not None:
            vehicle_location = vehicle_transform.location
            route_point, self._route_index_cursor = compute_local_target_point(
                self._route_geometry,
                vehicle_x=vehicle_location.x,
                vehicle_y=vehicle_location.y,
                vehicle_yaw_deg=vehicle_transform.rotation.yaw,
                lookahead_m=self._route_lookahead_m,
                target_normalization_m=self._route_target_normalization_m,
                previous_route_index=self._route_index_cursor,
            )
        current_behavior = self._current_behavior()
        current_progress = _completion_ratio(self._initial_waypoint_count, self._remaining_waypoints())
        scene_state = self._observation_builder.build(
            timestamp_s=timestamp_s,
            town_id=self.town_id,
            ego=_build_ego_state(
                vehicle_transform,
                speed_mps=speed_mps,
                speed_limit_mps=self.target_speed_mps,
            ),
            route=RouteState(
                route_id=self.route_id,
                maneuver=current_behavior,
                progress_ratio=current_progress,
                target_speed_mps=self.target_speed_mps,
            ),
            metadata={"route_point": route_point, **dict(metadata or {})},
        )
        decision = self._agent.step(scene_state)
        progress_after_step = _completion_ratio(self._initial_waypoint_count, self._remaining_waypoints())
        return StackStepResult(
            decision=decision,
            behavior=decision.behavior,
            progress_ratio=progress_after_step,
            done=self._agent.done(),
            route_point=route_point,
        )


class PilotNetEvalStack:
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
    ) -> None:
        resolved_checkpoint_path = Path(checkpoint_path).resolve()
        runtime_device = select_device(device)
        self._runtime = load_pilotnet_runtime(resolved_checkpoint_path, runtime_device)
        self.route_id = route_id
        self.town_id = town_id
        self.target_speed_mps = target_speed_kmh / 3.6
        self._route_geometry = route_geometry
        self._route_index_cursor: int | None = None
        self._rgb_history: deque[Any] = deque(maxlen=self._runtime.frame_stack)
        self._previous_applied_steer: float | None = None
        self._observation_builder = ObservationBuilder()
        self._expert_agent = ExpertBasicAgent(
            vehicle,
            world_map,
            config=ExpertBasicAgentConfig(
                target_speed_kmh=target_speed_kmh,
                ignore_traffic_lights=ignore_traffic_lights,
                ignore_stop_signs=ignore_stop_signs,
                ignore_vehicles=ignore_vehicles,
                sampling_resolution_m=sampling_resolution_m,
            ),
        )
        self._expert_agent.set_global_plan(planned_trace)
        self._initial_waypoint_count = self._expert_agent.remaining_waypoints()
        if self._initial_waypoint_count <= 0:
            raise RuntimeError("Planner produced an empty local plan.")
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

    def _remaining_waypoints(self) -> int:
        return self._expert_agent.remaining_waypoints()

    def _current_behavior(self) -> str:
        return self._expert_agent.current_behavior()

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
        current_command = self._current_behavior()
        progress_before_step = _completion_ratio(self._initial_waypoint_count, self._remaining_waypoints())
        route_point: tuple[float, float] | None = None
        if self._runtime.requires_route_point() and self._route_geometry is not None:
            vehicle_location = vehicle_transform.location
            route_point, self._route_index_cursor = compute_local_target_point(
                self._route_geometry,
                vehicle_x=vehicle_location.x,
                vehicle_y=vehicle_location.y,
                vehicle_yaw_deg=vehicle_transform.rotation.yaw,
                lookahead_m=float(self._runtime.model_config.get("route_lookahead_m", 8.0)),
                target_normalization_m=float(self._runtime.model_config.get("route_target_normalization_m", 20.0)),
                previous_route_index=self._route_index_cursor,
            )
        scene_state = self._observation_builder.build(
            timestamp_s=timestamp_s,
            town_id=self.town_id,
            ego=_build_ego_state(
                vehicle_transform,
                speed_mps=speed_mps,
                speed_limit_mps=self.target_speed_mps,
            ),
            route=RouteState(
                route_id=self.route_id,
                maneuver=current_command,
                progress_ratio=progress_before_step,
                target_speed_mps=self.target_speed_mps,
            ),
            metadata={
                "front_rgb_history": list(self._rgb_history),
                "command": current_command,
                "route_point": route_point,
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
        progress_after_step = _completion_ratio(self._initial_waypoint_count, self._remaining_waypoints())
        return StackStepResult(
            decision=decision,
            expert_decision=self._latest_expert_decision,
            applied_steer=applied_steer,
            behavior=decision.behavior,
            progress_ratio=progress_after_step,
            done=self._expert_agent.done(),
            route_point=route_point,
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
