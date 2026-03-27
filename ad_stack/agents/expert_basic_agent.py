from __future__ import annotations

from dataclasses import dataclass
import math
from collections import deque
from typing import Any, Literal

from ad_stack.agents.base import ControlDecision, VehicleCommand
from ad_stack.world_model import DynamicVehicleStateView, SceneState, TrafficLightStateView
from libs.carla_utils import ensure_carla_agents_on_path, road_option_name


def _lane_id(waypoint: Any | None) -> str | None:
    if waypoint is None:
        return None
    return f"{waypoint.road_id}:{waypoint.lane_id}"


def _speed_kmh(speed_mps: float) -> float:
    return speed_mps * 3.6


@dataclass(slots=True)
class ExpertBasicAgentConfig:
    target_speed_kmh: float = 30.0
    ignore_traffic_lights: bool = False
    ignore_stop_signs: bool = True
    ignore_vehicles: bool = False
    sampling_resolution_m: float = 2.0
    follow_headway_seconds: float = 1.8
    yellow_stop_margin_seconds: float = 1.0
    traffic_light_stop_buffer_m: float = 3.0
    traffic_light_red_latch_seconds: float = 0.5
    overtake_speed_delta_kmh: float = 8.0
    overtake_min_front_gap_m: float = 20.0
    overtake_min_rear_gap_m: float = 15.0
    overtake_signal_suppression_distance_m: float = 35.0
    overtake_trigger_distance_m: float = 18.0
    overtake_resume_front_gap_m: float = 12.0
    preferred_deceleration_mps2: float = 4.0
    reaction_margin_m: float = 2.0
    ttc_emergency_threshold_s: float = 0.75
    lead_brake_distance_m: float = 4.5
    allow_overtake: bool = True
    preferred_overtake_direction: Literal["left_first", "right_first"] = "left_first"
    lane_change_same_lane_distance_m: float = 6.0
    lane_change_distance_m: float = 14.0
    lane_change_other_lane_distance_m: float = 22.0
    overtake_hold_distance_m: float = 30.0


class ExpertBasicAgent:
    """Rule-based expert policy that reuses CARLA's local planner for low-level control."""

    name = "expert_route_policy"

    def __init__(
        self,
        vehicle: Any,
        world_map: Any,
        *,
        config: ExpertBasicAgentConfig | None = None,
    ) -> None:
        ensure_carla_agents_on_path()
        from agents.navigation.basic_agent import BasicAgent
        from agents.navigation.controller import VehiclePIDController

        self.config = config or ExpertBasicAgentConfig()
        self._vehicle = vehicle
        self._world = vehicle.get_world()
        self._map = world_map
        self._agent = BasicAgent(
            vehicle,
            target_speed=self.config.target_speed_kmh,
            opt_dict={
                # Hazard handling is implemented in this policy. The local planner is only reused
                # for waypoint tracking and longitudinal/lateral PID generation.
                "ignore_traffic_lights": True,
                "ignore_stop_signs": True,
                "ignore_vehicles": True,
                "sampling_resolution": self.config.sampling_resolution_m,
            },
            map_inst=world_map,
        )
        self._overtake_controller = VehiclePIDController(
            vehicle,
            args_lateral={"K_P": 1.95, "K_I": 0.05, "K_D": 0.2, "dt": 1.0 / 20.0},
            args_longitudinal={"K_P": 1.0, "K_I": 0.05, "K_D": 0.0, "dt": 1.0 / 20.0},
            max_throttle=0.75,
            max_brake=0.5,
            max_steering=0.8,
        )
        self._base_trace: list[tuple[Any, Any]] = []
        self._route_point_to_trace_index: list[int] = []
        self._max_route_index: int = 0
        self._overtake_state = "idle"
        self._overtake_direction: Literal["left", "right"] | None = None
        self._overtake_origin_lane_id: str | None = None
        self._overtake_target_lane_id: str | None = None
        self._overtake_aborted = False
        self._overtake_waypoints: deque[Any] = deque()
        self._car_follow_active = False
        self._waiting_on_light = False
        self._traffic_light_speed_error_kmh = 0.0
        self._latched_red_light: TrafficLightStateView | None = None
        self._latched_red_until_s: float = -1.0

    def set_global_plan(self, trace: list[tuple[Any, Any]]) -> None:
        self._base_trace = list(trace)
        self._route_point_to_trace_index = []
        last_point: tuple[float, float] | None = None
        for trace_index, (waypoint, _option) in enumerate(self._base_trace):
            location = waypoint.transform.location
            point = (location.x, location.y)
            if last_point is None or math.hypot(point[0] - last_point[0], point[1] - last_point[1]) > 0.05:
                self._route_point_to_trace_index.append(trace_index)
                last_point = point
        self._agent.set_global_plan(trace, stop_waypoint_creation=True, clean_queue=True)

    def remaining_waypoints(self) -> int:
        return len(self._agent.get_local_planner().get_plan())

    def current_behavior(self) -> str:
        return road_option_name(self._agent.get_local_planner().target_road_option)

    def done(self) -> bool:
        return bool(self._agent.done())

    def reset(self) -> None:
        self._max_route_index = 0
        self._overtake_state = "idle"
        self._overtake_direction = None
        self._overtake_origin_lane_id = None
        self._overtake_target_lane_id = None
        self._overtake_aborted = False
        self._overtake_waypoints.clear()
        self._car_follow_active = False
        self._waiting_on_light = False
        self._traffic_light_speed_error_kmh = 0.0
        self._latched_red_light = None
        self._latched_red_until_s = -1.0

    def _traffic_light_longitudinal_control(
        self,
        *,
        current_speed_mps: float,
        target_speed_kmh: float,
    ) -> tuple[float, float]:
        current_speed_kmh = _speed_kmh(current_speed_mps)
        error_kmh = target_speed_kmh - current_speed_kmh
        derivative_kmh = error_kmh - self._traffic_light_speed_error_kmh
        self._traffic_light_speed_error_kmh = error_kmh

        if error_kmh >= 0.0:
            throttle = min(
                0.5,
                max(0.0, (0.06 * error_kmh) + (0.01 * derivative_kmh)),
            )
            return throttle, 0.0

        brake = min(0.5, max(0.0, 0.08 * (-error_kmh)))
        return 0.0, brake

    def _stopping_distance_m(self, current_speed_mps: float) -> float:
        return self.config.reaction_margin_m + (
            (current_speed_mps * current_speed_mps) / max(2.0 * self.config.preferred_deceleration_mps2, 1e-3)
        )

    def _route_trace_index(self, route_index: int | None) -> int:
        if not self._route_point_to_trace_index:
            return 0
        if route_index is None:
            return self._route_point_to_trace_index[0]
        bounded_index = max(0, min(route_index, len(self._route_point_to_trace_index) - 1))
        return self._route_point_to_trace_index[bounded_index]

    def _set_rejoin_plan(self, route_index: int | None) -> None:
        if not self._base_trace:
            return
        trace_index = min(self._route_trace_index(route_index) + 8, len(self._base_trace) - 1)
        rejoin_plan = self._base_trace[trace_index:]
        self._agent.set_global_plan(
            rejoin_plan,
            stop_waypoint_creation=True,
            clean_queue=True,
        )
        self._overtake_waypoints = deque(waypoint for waypoint, _option in rejoin_plan)

    def _consume_overtake_waypoint(self) -> Any | None:
        if not self._overtake_waypoints:
            return None
        vehicle_location = self._vehicle.get_location()
        while len(self._overtake_waypoints) > 1:
            next_waypoint = self._overtake_waypoints[0]
            if vehicle_location.distance(next_waypoint.transform.location) > max(self.config.sampling_resolution_m, 3.0):
                break
            self._overtake_waypoints.popleft()
        return self._overtake_waypoints[0]

    def _start_overtake(self, direction: Literal["left", "right"], route_index: int | None) -> bool:
        if not self._base_trace:
            return False
        current_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        lane_change_path = self._agent._generate_lane_change_path(  # noqa: SLF001 - deliberate internal reuse
            current_waypoint,
            direction=direction,
            distance_same_lane=self.config.lane_change_same_lane_distance_m,
            distance_other_lane=self.config.lane_change_other_lane_distance_m,
            lane_change_distance=self.config.lane_change_distance_m,
            # Gap and signal checks are handled in this policy. Keep CARLA's path generator
            # as a geometry helper instead of letting it reject opposite-lane overtakes.
            check=False,
            lane_changes=1,
            step_distance=self.config.sampling_resolution_m,
        )
        if not lane_change_path:
            return False

        combined_plan = list(lane_change_path)
        hold_waypoint = lane_change_path[-1][0]
        hold_option = lane_change_path[-1][1]
        hold_lane_id = _lane_id(hold_waypoint)
        remaining_hold_distance_m = self.config.overtake_hold_distance_m
        while remaining_hold_distance_m > 1e-3:
            next_waypoints = hold_waypoint.next(min(self.config.sampling_resolution_m, remaining_hold_distance_m))
            if not next_waypoints:
                break
            preferred_waypoint = next(
                (waypoint for waypoint in next_waypoints if _lane_id(waypoint) == hold_lane_id),
                next_waypoints[0],
            )
            if preferred_waypoint.transform.location.distance(hold_waypoint.transform.location) <= 0.05:
                break
            combined_plan.append((preferred_waypoint, hold_option))
            remaining_hold_distance_m -= preferred_waypoint.transform.location.distance(hold_waypoint.transform.location)
            hold_waypoint = preferred_waypoint

        trace_index = min(
            self._route_trace_index(route_index)
            + 8
            + int(math.ceil(self.config.overtake_hold_distance_m / max(self.config.sampling_resolution_m, 0.1))),
            len(self._base_trace) - 1,
        )
        for waypoint, option in self._base_trace[trace_index:]:
            if combined_plan:
                tail_location = combined_plan[-1][0].transform.location
                if tail_location.distance(waypoint.transform.location) <= 0.5:
                    continue
            combined_plan.append((waypoint, option))
        self._overtake_waypoints = deque(waypoint for waypoint, _option in combined_plan)
        return True

    @staticmethod
    def _nearest_lead(
        tracked_objects: tuple[DynamicVehicleStateView, ...],
        *,
        relation: str = "same_lane",
    ) -> DynamicVehicleStateView | None:
        candidates = [
            actor
            for actor in tracked_objects
            if actor.relation == relation
            and actor.is_ahead
            and actor.longitudinal_distance_m is not None
            and actor.longitudinal_distance_m > 0.0
        ]
        if not candidates:
            return None
        return min(candidates, key=lambda actor: float(actor.longitudinal_distance_m))

    @staticmethod
    def _select_active_light(
        traffic_lights: tuple[TrafficLightStateView, ...],
    ) -> TrafficLightStateView | None:
        candidates = [light for light in traffic_lights if light.affects_ego]
        if not candidates:
            return None
        return min(
            candidates,
            key=lambda light: (
                float(light.stop_line_distance_m) if light.stop_line_distance_m is not None else float(light.distance_m)
            ),
        )

    def _resolve_active_light(
        self,
        traffic_lights: tuple[TrafficLightStateView, ...],
        *,
        timestamp_s: float,
    ) -> TrafficLightStateView | None:
        active_light = self._select_active_light(traffic_lights)
        if active_light is not None:
            if active_light.state == "red":
                self._latched_red_light = active_light
                self._latched_red_until_s = timestamp_s + self.config.traffic_light_red_latch_seconds
            else:
                self._latched_red_light = None
                self._latched_red_until_s = -1.0
            return active_light

        if (
            self._latched_red_light is not None
            and timestamp_s <= self._latched_red_until_s
        ):
            return self._latched_red_light

        self._latched_red_light = None
        self._latched_red_until_s = -1.0
        return None

    @staticmethod
    def _lane_gaps(
        tracked_objects: tuple[DynamicVehicleStateView, ...],
        relation: str,
    ) -> tuple[float, float]:
        front_gap_m = float("inf")
        rear_gap_m = float("inf")
        for actor in tracked_objects:
            if actor.relation != relation or actor.longitudinal_distance_m is None:
                continue
            longitudinal_distance = float(actor.longitudinal_distance_m)
            if actor.is_ahead and longitudinal_distance >= 0.0:
                front_gap_m = min(front_gap_m, longitudinal_distance)
            elif longitudinal_distance < 0.0:
                rear_gap_m = min(rear_gap_m, abs(longitudinal_distance))
        return front_gap_m, rear_gap_m

    def _choose_overtake_direction(
        self,
        scene_state: SceneState,
        active_light: TrafficLightStateView | None,
    ) -> tuple[Literal["left", "right"] | None, bool]:
        if not self.config.allow_overtake:
            return None, False
        if self.config.ignore_vehicles:
            return None, False
        if active_light is not None and active_light.state in {"red", "yellow"}:
            stop_distance = active_light.stop_line_distance_m or active_light.distance_m
            if stop_distance <= self.config.overtake_signal_suppression_distance_m:
                return None, False

        ordered_directions: tuple[Literal["left", "right"], Literal["left", "right"]]
        if self.config.preferred_overtake_direction == "left_first":
            ordered_directions = ("left", "right")
        else:
            ordered_directions = ("right", "left")

        any_lane_open = False
        for direction in ordered_directions:
            if not scene_state.ego.adjacent_lanes_open.get(direction, False):
                continue
            any_lane_open = True
            relation = "left_lane" if direction == "left" else "right_lane"
            front_gap_m, rear_gap_m = self._lane_gaps(scene_state.tracked_objects, relation)
            if (
                front_gap_m >= self.config.overtake_min_front_gap_m
                and rear_gap_m >= self.config.overtake_min_rear_gap_m
            ):
                return direction, True
        return None, any_lane_open

    def _should_stop_for_light(
        self,
        light: TrafficLightStateView | None,
        current_speed_mps: float,
    ) -> bool:
        if self.config.ignore_traffic_lights or light is None:
            return False

        raw_stop_distance = light.stop_line_distance_m if light.stop_line_distance_m is not None else light.distance_m
        if light.state == "red":
            return True
        if raw_stop_distance < 0.5:
            return False
        target_stop_distance = max(0.0, raw_stop_distance - self.config.traffic_light_stop_buffer_m)
        if light.state != "yellow":
            return False

        stopping_distance = self._stopping_distance_m(current_speed_mps)
        margin_distance = current_speed_mps * self.config.yellow_stop_margin_seconds
        return target_stop_distance >= (stopping_distance + margin_distance)

    def _traffic_light_stop_target_distance_m(self, light: TrafficLightStateView | None) -> float | None:
        if light is None:
            return None
        stop_distance = light.stop_line_distance_m if light.stop_line_distance_m is not None else light.distance_m
        return max(0.0, stop_distance - self.config.traffic_light_stop_buffer_m)

    def _traffic_light_stop_target_speed_kmh(
        self,
        light: TrafficLightStateView | None,
    ) -> float:
        target_distance = self._traffic_light_stop_target_distance_m(light)
        if target_distance is None or target_distance <= 0.25:
            return 0.0
        desired_speed_mps = math.sqrt(
            max(0.0, 2.0 * max(self.config.preferred_deceleration_mps2, 1e-3) * target_distance)
        )
        return min(self.config.target_speed_kmh, desired_speed_mps * 3.6)

    def step(self, scene_state: SceneState) -> ControlDecision:
        current_speed_mps = scene_state.ego.speed_mps
        route_index = scene_state.route.route_index
        if route_index is not None:
            self._max_route_index = max(self._max_route_index, route_index)

        active_light = self._resolve_active_light(
            scene_state.traffic_lights,
            timestamp_s=scene_state.timestamp_s,
        )
        lead_vehicle = self._nearest_lead(scene_state.tracked_objects, relation="same_lane")
        lead_distance_m = float(lead_vehicle.longitudinal_distance_m) if lead_vehicle and lead_vehicle.longitudinal_distance_m is not None else None
        lead_speed_mps = float(lead_vehicle.speed_mps) if lead_vehicle is not None else 0.0
        lead_speed_kmh = _speed_kmh(lead_speed_mps) if lead_vehicle is not None else 0.0
        closing_speed_mps = current_speed_mps - lead_speed_mps if lead_vehicle is not None else 0.0
        min_ttc = float("inf")
        if lead_distance_m is not None and closing_speed_mps > 1e-3:
            min_ttc = lead_distance_m / closing_speed_mps
        follow_target_speed_kmh = self.config.target_speed_kmh
        if lead_distance_m is not None:
            distance_limited_speed_kmh = max(
                0.0,
                (lead_distance_m / max(self.config.follow_headway_seconds, 0.25)) * 3.6,
            )
            follow_target_speed_kmh = min(
                self.config.target_speed_kmh,
                lead_speed_kmh + 2.0,
                distance_limited_speed_kmh,
            )

        planner_state = "nominal_cruise"
        target_speed_kmh = self.config.target_speed_kmh
        target_lane_id = scene_state.route.target_lane_id or scene_state.ego.lane_id
        current_lane_id = scene_state.ego.lane_id
        event_flags = {
            "event_traffic_light_stop": False,
            "event_traffic_light_resume": False,
            "event_car_follow_start": False,
            "event_overtake_attempt": False,
            "event_overtake_success": False,
            "event_overtake_abort": False,
            "event_unsafe_lane_change_reject": False,
        }

        if self._overtake_state != "idle":
            if self._overtake_aborted:
                planner_state = "abort_return"
            elif current_lane_id is not None and current_lane_id == self._overtake_target_lane_id:
                self._overtake_state = "pass_vehicle"
                planner_state = "pass_vehicle"
            else:
                planner_state = self._overtake_state

            if self._overtake_state in {"lane_change_out", "abort_return"}:
                lane_change_target_speed_kmh = min(
                    self.config.target_speed_kmh,
                    max(follow_target_speed_kmh, lead_speed_kmh + self.config.overtake_speed_delta_kmh),
                )
                target_speed_kmh = min(target_speed_kmh, lane_change_target_speed_kmh)

            if self._overtake_state in {"lane_change_out", "pass_vehicle"} and self._should_stop_for_light(
                active_light,
                current_speed_mps,
            ):
                self._set_rejoin_plan(route_index)
                self._overtake_state = "abort_return"
                self._overtake_aborted = True
                planner_state = "abort_return"
                event_flags["event_overtake_abort"] = True

            if (
                current_lane_id is not None
                and self._overtake_origin_lane_id is not None
                and current_lane_id == self._overtake_origin_lane_id
                and self._overtake_state in {"pass_vehicle", "abort_return"}
            ):
                if not self._overtake_aborted:
                    event_flags["event_overtake_success"] = True
                self._overtake_state = "idle"
                self._overtake_direction = None
                self._overtake_origin_lane_id = None
                self._overtake_target_lane_id = None
                self._overtake_aborted = False
                self._overtake_waypoints.clear()
                planner_state = "nominal_cruise"

        stop_for_light = self._should_stop_for_light(active_light, current_speed_mps)
        stop_target_distance_m = self._traffic_light_stop_target_distance_m(active_light)
        if stop_for_light and self._overtake_state == "idle":
            planner_state = "traffic_light_stop"
            target_speed_kmh = self._traffic_light_stop_target_speed_kmh(active_light)
        elif lead_vehicle is not None and not self.config.ignore_vehicles and self._overtake_state == "idle":
            should_overtake = (
                self.config.allow_overtake
                and lead_distance_m is not None
                and lead_distance_m <= self.config.overtake_trigger_distance_m
                and lead_speed_kmh <= (self.config.target_speed_kmh - self.config.overtake_speed_delta_kmh)
            )
            if should_overtake:
                direction, lane_safe_or_open = self._choose_overtake_direction(scene_state, active_light)
                if direction is not None:
                    ego_waypoint = self._map.get_waypoint(self._vehicle.get_location())
                    target_waypoint = ego_waypoint.get_left_lane() if direction == "left" else ego_waypoint.get_right_lane()
                    self._overtake_origin_lane_id = current_lane_id
                    self._overtake_target_lane_id = _lane_id(target_waypoint)
                    if self._start_overtake(direction, route_index):
                        self._overtake_state = "lane_change_out"
                        self._overtake_direction = direction
                        self._overtake_aborted = False
                        planner_state = "lane_change_out"
                        event_flags["event_overtake_attempt"] = True
                    else:
                        event_flags["event_unsafe_lane_change_reject"] = True
                        planner_state = "car_follow"
                        target_speed_kmh = follow_target_speed_kmh
                elif lane_safe_or_open:
                    planner_state = "car_follow"
                    target_speed_kmh = follow_target_speed_kmh
                else:
                    event_flags["event_unsafe_lane_change_reject"] = True
                    planner_state = "car_follow"
                    target_speed_kmh = follow_target_speed_kmh
            else:
                planner_state = "car_follow"
                target_speed_kmh = follow_target_speed_kmh

        if planner_state == "car_follow" and not self._car_follow_active:
            event_flags["event_car_follow_start"] = True
        self._car_follow_active = planner_state == "car_follow"

        if planner_state == "traffic_light_stop" and not self._waiting_on_light:
            event_flags["event_traffic_light_stop"] = True
        if planner_state != "traffic_light_stop" and self._waiting_on_light:
            event_flags["event_traffic_light_resume"] = True
        self._waiting_on_light = planner_state == "traffic_light_stop"

        self._agent.set_target_speed(max(0.0, target_speed_kmh))
        target_waypoint = self._consume_overtake_waypoint() if self._overtake_state != "idle" else None
        if target_waypoint is not None:
            control = self._overtake_controller.run_step(max(0.0, target_speed_kmh), target_waypoint)
        else:
            control = self._agent.run_step()

        emergency_stop = (
            not self.config.ignore_vehicles
            and lead_distance_m is not None
            and (
                lead_distance_m <= self.config.lead_brake_distance_m
                or (math.isfinite(min_ttc) and min_ttc <= self.config.ttc_emergency_threshold_s)
            )
        )
        imminent_red_light = (
            planner_state == "traffic_light_stop"
            and active_light is not None
            and active_light.state == "red"
            and active_light.stop_line_distance_m is not None
            and active_light.stop_line_distance_m <= self._stopping_distance_m(current_speed_mps)
            and current_speed_mps > 2.0
        )
        if planner_state == "traffic_light_stop":
            if stop_target_distance_m is None or stop_target_distance_m <= 0.5:
                self._traffic_light_speed_error_kmh = 0.0
                control.throttle = 0.0
                control.brake = max(float(control.brake), 1.0 if imminent_red_light else 0.45)
            else:
                traffic_light_throttle, traffic_light_brake = self._traffic_light_longitudinal_control(
                    current_speed_mps=current_speed_mps,
                    target_speed_kmh=target_speed_kmh,
                )
                control.throttle = traffic_light_throttle
                control.brake = max(float(control.brake), traffic_light_brake)
                if imminent_red_light:
                    control.throttle = 0.0
                    control.brake = max(float(control.brake), 1.0)
        else:
            self._traffic_light_speed_error_kmh = 0.0
        if emergency_stop:
            planner_state = "emergency_brake"
            control.throttle = 0.0
            control.brake = max(float(control.brake), 0.8)

        if (
            active_light is not None
            and active_light.state == "red"
            and active_light.stop_line_distance_m is not None
            and active_light.stop_line_distance_m <= 0.5
            and current_speed_mps > 2.0
        ):
            event_flags["traffic_light_violation"] = True
        else:
            event_flags["traffic_light_violation"] = False

        behavior = self.current_behavior()
        return ControlDecision(
            command=VehicleCommand(
                steer=float(control.steer),
                throttle=float(control.throttle),
                brake=float(control.brake),
                hand_brake=bool(control.hand_brake),
                reverse=bool(control.reverse),
            ).bounded(),
            behavior=behavior,
            planner_state=planner_state,
            debug={
                "remaining_waypoints": len(self._agent.get_local_planner().get_plan()),
                "route_progress_index": route_index,
                "max_route_index": self._max_route_index,
                "traffic_light_state": active_light.state if active_light is not None else None,
                "traffic_light_red_latched": bool(
                    active_light is not None
                    and self._latched_red_light is not None
                    and active_light.actor_id == self._latched_red_light.actor_id
                    and active_light.state == "red"
                    and scene_state.timestamp_s <= self._latched_red_until_s
                ),
                "traffic_light_stop_buffer_m": self.config.traffic_light_stop_buffer_m,
                "traffic_light_stop_target_distance_m": stop_target_distance_m,
                "target_speed_kmh": target_speed_kmh,
                "lead_vehicle_id": lead_vehicle.actor_id if lead_vehicle is not None else None,
                "lead_vehicle_distance_m": lead_distance_m,
                "overtake_state": self._overtake_state,
                "target_lane_id": target_lane_id,
                "min_ttc": None if not math.isfinite(min_ttc) else float(min_ttc),
                **event_flags,
            },
        )
