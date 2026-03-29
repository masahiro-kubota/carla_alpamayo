from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Literal

from ad_stack.agents.base import ControlDecision, VehicleCommand
from ad_stack.overtake import (
    AdjacentLaneGapSnapshot,
    LaneChangePlanPoint,
    OvertakeLeadSnapshot,
    OvertakeMemory,
    StoppedObstacleTargetSnapshot,
    StoppedObstacleContext,
    build_stopped_obstacle_targets,
    build_route_aligned_lane_change_plan,
    choose_overtake_action,
    evaluate_pass_progress,
    should_begin_rejoin,
)
from libs.carla_utils import ensure_carla_agents_on_path, road_option_name

if TYPE_CHECKING:
    from ad_stack.world_model import DynamicVehicleStateView, SceneState, TrafficLightStateView


def _lane_id(waypoint: Any | None) -> str | None:
    if waypoint is None:
        return None
    return f"{waypoint.road_id}:{waypoint.lane_id}"


def _speed_kmh(speed_mps: float) -> float:
    return speed_mps * 3.6


@dataclass(slots=True)
class _RouteAlignedWaypoint:
    transform: Any


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
    traffic_light_brake_start_distance_m: float = 10.0
    traffic_light_creep_resume_distance_m: float = 1.0
    traffic_light_creep_speed_kmh: float = 4.0
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
    overtake_cluster_merge_gap_m: float = 10.0
    overtake_cluster_max_member_speed_mps: float = 0.5
    stopped_speed_threshold_mps: float = 0.3


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
        import carla

        ensure_carla_agents_on_path()
        from agents.navigation.basic_agent import BasicAgent
        from agents.navigation.controller import VehiclePIDController

        self._carla = carla
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
        self._route_point_progress_m: list[float] = []
        self._max_route_index: int = 0
        self._overtake_state = "idle"
        self._overtake_direction: Literal["left", "right"] | None = None
        self._overtake_origin_lane_id: str | None = None
        self._overtake_target_lane_id: str | None = None
        self._overtake_target_actor_id: int | None = None
        self._overtake_target_member_actor_ids: tuple[int, ...] = ()
        self._overtake_aborted = False
        self._overtake_waypoints: deque[Any] = deque()
        self._overtake_memory = OvertakeMemory()
        self._lane_change_path_available = False
        self._lane_change_path_failure_reason: str | None = None
        self._car_follow_active = False
        self._waiting_on_light = False
        self._longitudinal_speed_error_kmh = 0.0
        self._previous_planner_state: str | None = None
        self._latched_red_light: TrafficLightStateView | None = None
        self._latched_red_until_s: float = -1.0

    def set_global_plan(self, trace: list[tuple[Any, Any]]) -> None:
        self._base_trace = list(trace)
        self._route_point_to_trace_index = []
        self._route_point_progress_m = []
        last_point: tuple[float, float] | None = None
        cumulative_progress_m = 0.0
        for trace_index, (waypoint, _option) in enumerate(self._base_trace):
            location = waypoint.transform.location
            point = (location.x, location.y)
            if (
                last_point is None
                or math.hypot(point[0] - last_point[0], point[1] - last_point[1]) > 0.05
            ):
                if last_point is not None:
                    cumulative_progress_m += math.hypot(
                        point[0] - last_point[0],
                        point[1] - last_point[1],
                    )
                self._route_point_to_trace_index.append(trace_index)
                self._route_point_progress_m.append(cumulative_progress_m)
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
        self._overtake_target_actor_id = None
        self._overtake_target_member_actor_ids = ()
        self._overtake_aborted = False
        self._overtake_waypoints.clear()
        self._overtake_memory = OvertakeMemory()
        self._lane_change_path_available = False
        self._lane_change_path_failure_reason = None
        self._car_follow_active = False
        self._waiting_on_light = False
        self._longitudinal_speed_error_kmh = 0.0
        self._previous_planner_state = None
        self._latched_red_light = None
        self._latched_red_until_s = -1.0

    def _speed_control(
        self,
        *,
        current_speed_mps: float,
        target_speed_kmh: float,
        max_throttle: float = 0.75,
        max_brake: float = 1.0,
    ) -> tuple[float, float]:
        current_speed_kmh = _speed_kmh(current_speed_mps)
        error_kmh = target_speed_kmh - current_speed_kmh
        derivative_kmh = error_kmh - self._longitudinal_speed_error_kmh
        self._longitudinal_speed_error_kmh = error_kmh

        if error_kmh >= 0.0:
            throttle = min(
                max_throttle,
                max(0.0, (0.06 * error_kmh) + (0.01 * derivative_kmh)),
            )
            return throttle, 0.0

        brake = min(max_brake, max(0.0, 0.08 * (-error_kmh)))
        return 0.0, brake

    def _stopping_distance_m(self, current_speed_mps: float) -> float:
        return self.config.reaction_margin_m + (
            (current_speed_mps * current_speed_mps)
            / max(2.0 * self.config.preferred_deceleration_mps2, 1e-3)
        )

    def _route_trace_index(self, route_index: int | None) -> int:
        if not self._route_point_to_trace_index:
            return 0
        if route_index is None:
            return self._route_point_to_trace_index[0]
        bounded_index = max(0, min(route_index, len(self._route_point_to_trace_index) - 1))
        return self._route_point_to_trace_index[bounded_index]

    def _set_rejoin_plan(self, route_index: int | None) -> None:
        if self._start_rejoin(route_index):
            return
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

    def _resume_base_route(self, route_index: int | None) -> None:
        if not self._base_trace:
            return
        trace_index = min(self._route_trace_index(route_index) + 1, len(self._base_trace) - 1)
        remaining_plan = self._base_trace[trace_index:]
        self._agent.set_global_plan(
            remaining_plan,
            stop_waypoint_creation=True,
            clean_queue=True,
        )

    def _adjacent_lane_waypoint(
        self, origin_waypoint: Any, direction: Literal["left", "right"]
    ) -> Any | None:
        adjacent_waypoint = (
            origin_waypoint.get_left_lane()
            if direction == "left"
            else origin_waypoint.get_right_lane()
        )
        if adjacent_waypoint is None:
            return None
        if adjacent_waypoint.lane_type != self._carla.LaneType.Driving:
            return None
        return adjacent_waypoint

    def _route_aligned_waypoint(self, waypoint: Any, heading_source_waypoint: Any) -> Any:
        location = waypoint.transform.location
        rotation = heading_source_waypoint.transform.rotation
        transform = self._carla.Transform(
            self._carla.Location(x=location.x, y=location.y, z=location.z),
            self._carla.Rotation(
                pitch=rotation.pitch,
                yaw=rotation.yaw,
                roll=rotation.roll,
            ),
        )
        return _RouteAlignedWaypoint(transform=transform)

    def _interpolate_waypoint(self, start_waypoint: Any, end_waypoint: Any, alpha: float) -> Any:
        start_location = start_waypoint.transform.location
        end_location = end_waypoint.transform.location
        rotation = start_waypoint.transform.rotation
        transform = self._carla.Transform(
            self._carla.Location(
                x=(start_location.x * (1.0 - alpha)) + (end_location.x * alpha),
                y=(start_location.y * (1.0 - alpha)) + (end_location.y * alpha),
                z=(start_location.z * (1.0 - alpha)) + (end_location.z * alpha),
            ),
            self._carla.Rotation(
                pitch=rotation.pitch,
                yaw=rotation.yaw,
                roll=rotation.roll,
            ),
        )
        return _RouteAlignedWaypoint(transform=transform)

    def _build_route_aligned_lane_samples(
        self,
        *,
        direction: Literal["left", "right"],
        route_index: int | None,
    ) -> tuple[
        list[LaneChangePlanPoint],
        list[LaneChangePlanPoint],
        dict[tuple[int, str], Any],
        str | None,
    ]:
        origin_samples: list[LaneChangePlanPoint] = []
        target_samples: list[LaneChangePlanPoint] = []
        waypoint_lookup: dict[tuple[int, str], Any] = {}
        target_lane_id: str | None = None

        start_point_index = max(0, route_index or 0)
        if start_point_index >= len(self._route_point_to_trace_index):
            return origin_samples, target_samples, waypoint_lookup, target_lane_id

        previous_location: Any | None = None
        progress_m = 0.0
        for point_index in range(start_point_index, len(self._route_point_to_trace_index)):
            trace_index = self._route_point_to_trace_index[point_index]
            origin_waypoint = self._base_trace[trace_index][0]
            origin_location = origin_waypoint.transform.location
            if previous_location is not None:
                progress_m += origin_location.distance(previous_location)
            previous_location = origin_location

            origin_lane_id = _lane_id(origin_waypoint)
            if origin_lane_id is None:
                continue
            origin_samples.append(
                LaneChangePlanPoint(
                    route_index=point_index,
                    lane_id=origin_lane_id,
                    progress_m=progress_m,
                )
            )
            waypoint_lookup[(point_index, origin_lane_id)] = self._route_aligned_waypoint(
                origin_waypoint, origin_waypoint
            )

            adjacent_waypoint = self._adjacent_lane_waypoint(origin_waypoint, direction)
            adjacent_lane_id = _lane_id(adjacent_waypoint)
            if adjacent_waypoint is None or adjacent_lane_id is None:
                continue
            if target_lane_id is None:
                target_lane_id = adjacent_lane_id
            target_samples.append(
                LaneChangePlanPoint(
                    route_index=point_index,
                    lane_id=adjacent_lane_id,
                    progress_m=progress_m,
                )
            )
            waypoint_lookup[(point_index, adjacent_lane_id)] = self._route_aligned_waypoint(
                adjacent_waypoint, origin_waypoint
            )

        return origin_samples, target_samples, waypoint_lookup, target_lane_id

    def _materialize_lane_change_waypoints(
        self,
        *,
        start_samples: list[LaneChangePlanPoint],
        destination_samples: list[LaneChangePlanPoint],
        waypoint_lookup: dict[tuple[int, str], Any],
        distance_same_lane_m: float,
        lane_change_distance_m: float,
        distance_other_lane_m: float,
    ) -> list[Any]:
        if not start_samples or not destination_samples:
            return []

        start_progress_m = start_samples[0].progress_m
        same_lane_end_m = start_progress_m + distance_same_lane_m
        lane_change_end_m = same_lane_end_m + lane_change_distance_m
        destination_lane_end_m = lane_change_end_m + distance_other_lane_m
        destination_by_route_index = {sample.route_index: sample for sample in destination_samples}
        materialized: list[Any] = []
        for point in start_samples:
            if point.progress_m > destination_lane_end_m + 1e-6:
                break
            start_waypoint = waypoint_lookup.get((point.route_index, point.lane_id))
            if start_waypoint is None:
                continue

            destination_point = destination_by_route_index.get(point.route_index)
            destination_waypoint = (
                waypoint_lookup.get((destination_point.route_index, destination_point.lane_id))
                if destination_point is not None
                else None
            )
            if point.progress_m <= same_lane_end_m + 1e-6 or destination_waypoint is None:
                waypoint = start_waypoint
            elif point.progress_m < lane_change_end_m - 1e-6:
                alpha = (point.progress_m - same_lane_end_m) / max(lane_change_distance_m, 1e-6)
                waypoint = self._interpolate_waypoint(
                    start_waypoint,
                    destination_waypoint,
                    max(0.0, min(1.0, alpha)),
                )
            else:
                waypoint = destination_waypoint

            if materialized:
                tail_location = materialized[-1].transform.location
                if tail_location.distance(waypoint.transform.location) <= 0.05:
                    continue
            materialized.append(waypoint)
        return materialized

    def _consume_overtake_waypoint(self) -> Any | None:
        if not self._overtake_waypoints:
            return None
        vehicle_location = self._vehicle.get_location()
        # Overtake paths include large lateral shifts. Using the default 3 m acceptance radius lets
        # the controller skip lane-center waypoints before the ego vehicle has actually settled into
        # the adjacent lane, which leaves the car straddling the lane boundary during pass/rejoin.
        acceptance_radius_m = min(1.0, max(0.6, self.config.sampling_resolution_m * 0.4))
        while len(self._overtake_waypoints) > 1:
            next_waypoint = self._overtake_waypoints[0]
            if vehicle_location.distance(next_waypoint.transform.location) > acceptance_radius_m:
                break
            self._overtake_waypoints.popleft()
        return self._overtake_waypoints[0]

    def _start_overtake(self, direction: Literal["left", "right"], route_index: int | None) -> bool:
        if not self._base_trace:
            self._lane_change_path_available = False
            self._lane_change_path_failure_reason = "missing_base_trace"
            return False

        (
            origin_samples,
            target_samples,
            waypoint_lookup,
            target_lane_id,
        ) = self._build_route_aligned_lane_samples(direction=direction, route_index=route_index)
        if not origin_samples or not target_samples or target_lane_id is None:
            self._lane_change_path_available = False
            self._lane_change_path_failure_reason = "adjacent_lane_sample_missing"
            return False

        lane_change_plan = build_route_aligned_lane_change_plan(
            origin_samples,
            target_samples,
            distance_same_lane_m=self.config.lane_change_same_lane_distance_m,
            lane_change_distance_m=self.config.lane_change_distance_m,
            distance_other_lane_m=self.config.overtake_hold_distance_m,
        )
        if not lane_change_plan.available:
            self._lane_change_path_available = False
            self._lane_change_path_failure_reason = lane_change_plan.failure_reason
            return False

        lane_change_end_m = (
            origin_samples[0].progress_m
            + self.config.lane_change_same_lane_distance_m
            + self.config.lane_change_distance_m
        )
        target_lane_follow_distance_m = max(
            self.config.overtake_hold_distance_m,
            target_samples[-1].progress_m - lane_change_end_m,
        )
        materialized = self._materialize_lane_change_waypoints(
            start_samples=origin_samples,
            destination_samples=target_samples,
            waypoint_lookup=waypoint_lookup,
            distance_same_lane_m=self.config.lane_change_same_lane_distance_m,
            lane_change_distance_m=self.config.lane_change_distance_m,
            distance_other_lane_m=target_lane_follow_distance_m,
        )
        if not materialized:
            self._lane_change_path_available = False
            self._lane_change_path_failure_reason = "lane_change_materialization_failed"
            return False

        self._overtake_waypoints = deque(materialized)
        self._overtake_target_lane_id = target_lane_id
        self._lane_change_path_available = True
        self._lane_change_path_failure_reason = None
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

    def _same_lane_stopped_targets(
        self,
        tracked_objects: tuple[DynamicVehicleStateView, ...],
    ) -> list[StoppedObstacleTargetSnapshot]:
        leads = [
            OvertakeLeadSnapshot(
                actor_id=actor.actor_id,
                lane_id=actor.lane_id,
                distance_m=float(actor.longitudinal_distance_m),
                speed_mps=float(actor.speed_mps),
                relative_speed_mps=0.0,
                is_stopped=float(actor.speed_mps) <= self.config.stopped_speed_threshold_mps,
            )
            for actor in tracked_objects
            if (
                actor.relation == "same_lane"
                and actor.is_ahead
                and actor.longitudinal_distance_m is not None
                and float(actor.longitudinal_distance_m) > 0.0
            )
        ]
        return build_stopped_obstacle_targets(
            leads,
            cluster_merge_gap_m=self.config.overtake_cluster_merge_gap_m,
            cluster_max_member_speed_mps=self.config.overtake_cluster_max_member_speed_mps,
        )

    def _route_relative_progress_to_actor(
        self,
        *,
        actor: DynamicVehicleStateView,
        reference_route_index: int,
        search_back_points: int = 24,
        search_forward_points: int = 120,
    ) -> tuple[float | None, float, str | None]:
        if not self._route_point_to_trace_index:
            return None, float("inf"), None
        bounded_reference_index = max(
            0, min(reference_route_index, len(self._route_point_to_trace_index) - 1)
        )
        start_index = max(0, bounded_reference_index - search_back_points)
        end_index = min(
            len(self._route_point_to_trace_index),
            bounded_reference_index + search_forward_points + 1,
        )
        best_route_index: int | None = None
        best_distance_m = float("inf")
        for route_point_index in range(start_index, end_index):
            trace_index = self._route_point_to_trace_index[route_point_index]
            route_location = self._base_trace[trace_index][0].transform.location
            distance_m = math.hypot(route_location.x - actor.x_m, route_location.y - actor.y_m)
            if distance_m < best_distance_m:
                best_distance_m = distance_m
                best_route_index = route_point_index
        if best_route_index is None:
            return None, best_distance_m, None
        trace_index = self._route_point_to_trace_index[best_route_index]
        route_lane_id = _lane_id(self._base_trace[trace_index][0])
        relative_progress_m = (
            self._route_point_progress_m[best_route_index]
            - self._route_point_progress_m[bounded_reference_index]
        )
        return float(relative_progress_m), best_distance_m, route_lane_id

    def _route_aligned_stopped_targets(
        self,
        tracked_objects: tuple[DynamicVehicleStateView, ...],
        *,
        route_index: int | None,
    ) -> list[StoppedObstacleTargetSnapshot]:
        if (
            route_index is None
            or not self._base_trace
            or not self._route_point_to_trace_index
            or not self._route_point_progress_m
        ):
            return []

        bounded_route_index = max(0, min(route_index, len(self._route_point_to_trace_index) - 1))
        leads: list[OvertakeLeadSnapshot] = []
        for actor in tracked_objects:
            if actor.actor_id is None or actor.lane_id is None:
                continue
            if float(actor.speed_mps) > self.config.stopped_speed_threshold_mps:
                continue
            (
                route_distance_m,
                route_distance_to_centerline_m,
                route_lane_id,
            ) = self._route_relative_progress_to_actor(
                actor=actor,
                reference_route_index=bounded_route_index,
                search_back_points=0,
            )
            if route_distance_m is None or route_distance_to_centerline_m > 4.0:
                continue
            if route_lane_id != actor.lane_id:
                continue
            if route_distance_m <= 0.0:
                continue
            leads.append(
                OvertakeLeadSnapshot(
                    actor_id=actor.actor_id,
                    lane_id=actor.lane_id,
                    distance_m=float(route_distance_m),
                    speed_mps=float(actor.speed_mps),
                    relative_speed_mps=0.0,
                    is_stopped=True,
                )
            )
        targets = build_stopped_obstacle_targets(
            leads,
            cluster_merge_gap_m=self.config.overtake_cluster_merge_gap_m,
            cluster_max_member_speed_mps=self.config.overtake_cluster_max_member_speed_mps,
        )
        enriched_targets: list[StoppedObstacleTargetSnapshot] = []
        for target in targets:
            matching_actor = next(
                (actor for actor in tracked_objects if actor.actor_id == target.primary_actor_id),
                None,
            )
            adjacent_lane_available = True
            if matching_actor is not None:
                target_waypoint = self._map.get_waypoint(
                    self._carla.Location(x=matching_actor.x_m, y=matching_actor.y_m, z=0.0),
                    lane_type=self._carla.LaneType.Driving,
                )
                target_left_waypoint = (
                    target_waypoint.get_left_lane() if target_waypoint is not None else None
                )
                target_right_waypoint = (
                    target_waypoint.get_right_lane() if target_waypoint is not None else None
                )
                adjacent_lane_available = bool(
                    (
                        target_left_waypoint is not None
                        and target_left_waypoint.lane_type == self._carla.LaneType.Driving
                    )
                    or (
                        target_right_waypoint is not None
                        and target_right_waypoint.lane_type == self._carla.LaneType.Driving
                    )
                )
            enriched_targets.append(
                StoppedObstacleTargetSnapshot(
                    kind=target.kind,
                    primary_actor_id=target.primary_actor_id,
                    member_actor_ids=target.member_actor_ids,
                    lane_id=target.lane_id,
                    entry_distance_m=target.entry_distance_m,
                    exit_distance_m=target.exit_distance_m,
                    speed_mps=target.speed_mps,
                    is_stopped=target.is_stopped,
                    adjacent_lane_available=adjacent_lane_available,
                )
            )
        return enriched_targets

    def _visible_overtake_target_actors(
        self,
        tracked_objects: tuple[DynamicVehicleStateView, ...],
    ) -> tuple[DynamicVehicleStateView | None, tuple[DynamicVehicleStateView, ...]]:
        if self._overtake_target_actor_id is None:
            return None, ()
        member_ids = set(self._overtake_target_member_actor_ids) or {self._overtake_target_actor_id}
        visible_members = tuple(
            actor for actor in tracked_objects if actor.actor_id in member_ids
        )
        primary_actor = next(
            (actor for actor in visible_members if actor.actor_id == self._overtake_target_actor_id),
            None,
        )
        return primary_actor, visible_members

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
                float(light.stop_line_distance_m)
                if light.stop_line_distance_m is not None
                else float(light.distance_m)
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
                self._latched_red_until_s = (
                    timestamp_s + self.config.traffic_light_red_latch_seconds
                )
            else:
                self._latched_red_light = None
                self._latched_red_until_s = -1.0
            return active_light

        if self._latched_red_light is not None and timestamp_s <= self._latched_red_until_s:
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

    @staticmethod
    def _lane_gaps_for_lane_id(
        tracked_objects: tuple[DynamicVehicleStateView, ...],
        lane_id: str | None,
    ) -> tuple[float, float]:
        if lane_id is None:
            return float("inf"), float("inf")
        front_gap_m = float("inf")
        rear_gap_m = float("inf")
        for actor in tracked_objects:
            if actor.lane_id != lane_id or actor.longitudinal_distance_m is None:
                continue
            longitudinal_distance = float(actor.longitudinal_distance_m)
            if longitudinal_distance >= 0.0:
                front_gap_m = min(front_gap_m, longitudinal_distance)
            else:
                rear_gap_m = min(rear_gap_m, abs(longitudinal_distance))
        return front_gap_m, rear_gap_m

    def _start_rejoin(self, route_index: int | None) -> bool:
        if (
            not self._base_trace
            or self._overtake_direction is None
            or self._overtake_origin_lane_id is None
            or self._overtake_target_lane_id is None
        ):
            self._lane_change_path_available = False
            self._lane_change_path_failure_reason = "missing_rejoin_context"
            return False

        (
            origin_samples,
            target_samples,
            waypoint_lookup,
            target_lane_id,
        ) = self._build_route_aligned_lane_samples(
            direction=self._overtake_direction,
            route_index=route_index,
        )
        if (
            not origin_samples
            or not target_samples
            or target_lane_id is None
        ):
            self._lane_change_path_available = False
            self._lane_change_path_failure_reason = "rejoin_lane_sample_missing"
            return False

        rejoin_plan = build_route_aligned_lane_change_plan(
            target_samples,
            origin_samples,
            distance_same_lane_m=0.0,
            lane_change_distance_m=self.config.lane_change_distance_m,
            distance_other_lane_m=max(self.config.sampling_resolution_m, 0.1),
        )
        if not rejoin_plan.available:
            self._lane_change_path_available = False
            self._lane_change_path_failure_reason = rejoin_plan.failure_reason
            return False

        lane_change_end_m = target_samples[0].progress_m + self.config.lane_change_distance_m
        origin_lane_follow_distance_m = max(
            self.config.sampling_resolution_m,
            origin_samples[-1].progress_m - lane_change_end_m,
        )
        materialized = self._materialize_lane_change_waypoints(
            start_samples=target_samples,
            destination_samples=origin_samples,
            waypoint_lookup=waypoint_lookup,
            distance_same_lane_m=0.0,
            lane_change_distance_m=self.config.lane_change_distance_m,
            distance_other_lane_m=origin_lane_follow_distance_m,
        )
        if not materialized:
            self._lane_change_path_available = False
            self._lane_change_path_failure_reason = "rejoin_materialization_failed"
            return False

        self._overtake_waypoints = deque(materialized)
        self._overtake_target_lane_id = self._overtake_origin_lane_id
        self._lane_change_path_available = True
        self._lane_change_path_failure_reason = None
        return True

    def _choose_overtake_direction(
        self,
        scene_state: SceneState,
        active_light: TrafficLightStateView | None,
    ) -> tuple[Literal["left", "right"] | None, bool, str | None]:
        if not self.config.allow_overtake:
            return None, False, "overtake_disabled"
        if self.config.ignore_vehicles:
            return None, False, "ignore_vehicles_enabled"
        if active_light is not None and active_light.state in {"red", "yellow"}:
            stop_distance = active_light.stop_line_distance_m or active_light.distance_m
            if stop_distance <= self.config.overtake_signal_suppression_distance_m:
                return None, False, "signal_suppressed"

        ordered_directions: tuple[Literal["left", "right"], Literal["left", "right"]]
        if self.config.preferred_overtake_direction == "left_first":
            ordered_directions = ("left", "right")
        else:
            ordered_directions = ("right", "left")

        any_lane_open = False
        reject_reason: str | None = None
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
                return direction, True, None
            if front_gap_m < self.config.overtake_min_front_gap_m:
                reject_reason = "adjacent_front_gap_insufficient"
            elif rear_gap_m < self.config.overtake_min_rear_gap_m:
                reject_reason = "adjacent_rear_gap_insufficient"
            else:
                reject_reason = "adjacent_lane_gap_insufficient"
        if not any_lane_open:
            reject_reason = "adjacent_lane_closed"
        return None, any_lane_open, reject_reason

    def _should_stop_for_light(
        self,
        light: TrafficLightStateView | None,
        current_speed_mps: float,
    ) -> bool:
        if self.config.ignore_traffic_lights or light is None:
            return False

        raw_stop_distance = (
            light.stop_line_distance_m
            if light.stop_line_distance_m is not None
            else light.distance_m
        )
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

    def _traffic_light_stop_target_distance_m(
        self, light: TrafficLightStateView | None
    ) -> float | None:
        if light is None:
            return None
        stop_distance = (
            light.stop_line_distance_m
            if light.stop_line_distance_m is not None
            else light.distance_m
        )
        return max(0.0, stop_distance - self.config.traffic_light_stop_buffer_m)

    def _traffic_light_stop_target_speed_kmh(
        self,
        light: TrafficLightStateView | None,
        *,
        current_speed_mps: float,
    ) -> float:
        target_distance = self._traffic_light_stop_target_distance_m(light)
        if target_distance is None or target_distance <= 0.0:
            return 0.0

        brake_start_distance_m = max(self.config.traffic_light_brake_start_distance_m, 1e-3)
        if target_distance >= brake_start_distance_m:
            target_speed_kmh = self.config.target_speed_kmh
        else:
            target_speed_kmh = self.config.target_speed_kmh * max(
                0.0, min(1.0, target_distance / brake_start_distance_m)
            )

        if (
            current_speed_mps <= 0.2
            and target_distance > self.config.traffic_light_creep_resume_distance_m
        ):
            target_speed_kmh = max(target_speed_kmh, self.config.traffic_light_creep_speed_kmh)

        return min(self.config.target_speed_kmh, max(0.0, target_speed_kmh))

    def _traffic_light_stop_control(
        self,
        *,
        current_speed_mps: float,
        light: TrafficLightStateView | None,
        stop_target_distance_m: float | None,
        target_speed_kmh: float,
    ) -> tuple[float, float]:
        if light is None:
            self._longitudinal_speed_error_kmh = 0.0
            return 0.0, 0.0

        if stop_target_distance_m is None:
            self._longitudinal_speed_error_kmh = 0.0
            return 0.0, 0.45 if current_speed_mps > 0.1 else 0.0

        if light.state == "red":
            if stop_target_distance_m <= 0.5:
                self._longitudinal_speed_error_kmh = 0.0
                return (
                    0.0,
                    1.0 if current_speed_mps > 2.0 else 0.45 if current_speed_mps > 0.1 else 0.0,
                )

            stopping_distance_m = self._stopping_distance_m(current_speed_mps)
            if stop_target_distance_m <= stopping_distance_m:
                self._longitudinal_speed_error_kmh = 0.0
                late_ratio = 1.0 - min(1.0, stop_target_distance_m / max(stopping_distance_m, 1e-3))
                return 0.0, min(1.0, max(0.45, 0.45 + (0.55 * late_ratio)))

        throttle, brake = self._speed_control(
            current_speed_mps=current_speed_mps,
            target_speed_kmh=target_speed_kmh,
            max_throttle=0.75,
            max_brake=1.0,
        )
        return throttle, brake

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
        same_lane_stopped_targets = self._same_lane_stopped_targets(scene_state.tracked_objects)
        route_aligned_stopped_targets = self._route_aligned_stopped_targets(
            scene_state.tracked_objects,
            route_index=route_index,
        )
        stopped_targets = (
            same_lane_stopped_targets if same_lane_stopped_targets else route_aligned_stopped_targets
        )
        active_overtake_target = stopped_targets[0] if stopped_targets else None
        same_lane_lead_distance_m = (
            float(lead_vehicle.longitudinal_distance_m)
            if lead_vehicle and lead_vehicle.longitudinal_distance_m is not None
            else None
        )
        same_lane_lead_speed_mps = float(lead_vehicle.speed_mps) if lead_vehicle is not None else 0.0
        lead_distance_m = (
            same_lane_lead_distance_m
            if same_lane_lead_distance_m is not None
            else (
                float(active_overtake_target.entry_distance_m)
                if active_overtake_target is not None
                else None
            )
        )
        lead_speed_mps = (
            same_lane_lead_speed_mps
            if same_lane_lead_distance_m is not None
            else (
                float(active_overtake_target.speed_mps)
                if active_overtake_target is not None
                else 0.0
            )
        )
        lead_speed_kmh = _speed_kmh(lead_speed_mps) if lead_distance_m is not None else 0.0
        closing_speed_mps = current_speed_mps - lead_speed_mps if lead_distance_m is not None else 0.0
        min_ttc = float("inf")
        if lead_distance_m is not None and closing_speed_mps > 1e-3:
            min_ttc = lead_distance_m / closing_speed_mps
        same_lane_min_ttc = float("inf")
        same_lane_closing_speed_mps = (
            current_speed_mps - same_lane_lead_speed_mps
            if same_lane_lead_distance_m is not None
            else 0.0
        )
        if same_lane_lead_distance_m is not None and same_lane_closing_speed_mps > 1e-3:
            same_lane_min_ttc = same_lane_lead_distance_m / same_lane_closing_speed_mps
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
        ego_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        lane_center_offset_m = (
            float(self._vehicle.get_location().distance(ego_waypoint.transform.location))
            if ego_waypoint is not None
            else None
        )
        left_front_gap_m, left_rear_gap_m = self._lane_gaps(scene_state.tracked_objects, "left_lane")
        right_front_gap_m, right_rear_gap_m = self._lane_gaps(
            scene_state.tracked_objects, "right_lane"
        )
        rejoin_front_gap_m = float("inf")
        rejoin_rear_gap_m = float("inf")
        overtake_reject_reason: str | None = None
        overtake_considered = False
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
            if (
                self._overtake_state == "lane_change_out"
                and current_lane_id is not None
                and current_lane_id == self._overtake_target_lane_id
            ):
                self._overtake_state = "pass_vehicle"
                planner_state = "pass_vehicle"
            elif self._overtake_state == "lane_change_back":
                planner_state = "abort_return" if self._overtake_aborted else "lane_change_back"
            else:
                planner_state = "abort_return" if self._overtake_aborted else self._overtake_state

            if self._overtake_state in {"lane_change_out", "abort_return"}:
                lane_change_target_speed_kmh = min(
                    self.config.target_speed_kmh,
                    max(
                        follow_target_speed_kmh,
                        lead_speed_kmh + self.config.overtake_speed_delta_kmh,
                    ),
                )
                target_speed_kmh = min(target_speed_kmh, lane_change_target_speed_kmh)

            if self._overtake_state in {
                "lane_change_out",
                "pass_vehicle",
            } and self._should_stop_for_light(
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
                and self._overtake_state in {"lane_change_back", "abort_return"}
                and (lane_center_offset_m is None or lane_center_offset_m <= 0.75)
            ):
                if not self._overtake_aborted:
                    event_flags["event_overtake_success"] = True
                self._overtake_state = "idle"
                self._overtake_direction = None
                self._overtake_origin_lane_id = None
                self._overtake_target_lane_id = None
                self._overtake_target_actor_id = None
                self._overtake_target_member_actor_ids = ()
                self._overtake_aborted = False
                self._overtake_waypoints.clear()
                self._overtake_memory = OvertakeMemory()
                self._resume_base_route(route_index)
                planner_state = "nominal_cruise"

        stop_for_light = self._should_stop_for_light(active_light, current_speed_mps)
        stop_target_distance_m = self._traffic_light_stop_target_distance_m(active_light)
        if stop_for_light and self._overtake_state == "idle":
            planner_state = "traffic_light_stop"
            target_speed_kmh = self._traffic_light_stop_target_speed_kmh(
                active_light,
                current_speed_mps=current_speed_mps,
            )
            if lead_vehicle is not None and not self.config.ignore_vehicles:
                # Red-light stopping still needs to respect a stopped or slow lead in the same lane.
                # Without this cap, the light-stop profile can keep throttle applied until the ego
                # reaches the stop line even when a stationary obstacle is sitting well before it.
                target_speed_kmh = min(target_speed_kmh, follow_target_speed_kmh)
                if (
                    self.config.allow_overtake
                    and active_light is not None
                    and active_light.state in {"red", "yellow"}
                ):
                    stop_distance = (
                        active_light.stop_line_distance_m
                        if active_light.stop_line_distance_m is not None
                        else active_light.distance_m
                    )
                    if stop_distance <= self.config.overtake_signal_suppression_distance_m:
                        overtake_considered = True
                        overtake_reject_reason = "signal_suppressed"
                        event_flags["event_unsafe_lane_change_reject"] = True
        elif (
            (lead_vehicle is not None or active_overtake_target is not None)
            and not self.config.ignore_vehicles
            and self._overtake_state == "idle"
        ):
            left_lane_waypoint = ego_waypoint.get_left_lane() if ego_waypoint is not None else None
            right_lane_waypoint = ego_waypoint.get_right_lane() if ego_waypoint is not None else None
            left_lane_snapshot = AdjacentLaneGapSnapshot(
                lane_id=_lane_id(left_lane_waypoint),
                front_gap_m=None if not math.isfinite(left_front_gap_m) else float(left_front_gap_m),
                rear_gap_m=None if not math.isfinite(left_rear_gap_m) else float(left_rear_gap_m),
                lane_open=bool(scene_state.ego.adjacent_lanes_open.get("left", False)),
            )
            right_lane_snapshot = AdjacentLaneGapSnapshot(
                lane_id=_lane_id(right_lane_waypoint),
                front_gap_m=None
                if not math.isfinite(right_front_gap_m)
                else float(right_front_gap_m),
                rear_gap_m=None if not math.isfinite(right_rear_gap_m) else float(right_rear_gap_m),
                lane_open=bool(scene_state.ego.adjacent_lanes_open.get("right", False)),
            )
            overtake_considered = True
            overtake_decision = choose_overtake_action(
                StoppedObstacleContext(
                    timestamp_s=scene_state.timestamp_s,
                    current_lane_id=current_lane_id,
                    origin_lane_id=current_lane_id,
                    route_target_lane_id=scene_state.route.target_lane_id,
                    target_speed_kmh=self.config.target_speed_kmh,
                    stopped_speed_threshold_mps=0.3,
                    lead=OvertakeLeadSnapshot(
                        actor_id=(
                            lead_vehicle.actor_id
                            if lead_vehicle is not None
                            else active_overtake_target.primary_actor_id
                            if active_overtake_target is not None
                            else None
                        ),
                        lane_id=(
                            lead_vehicle.lane_id
                            if lead_vehicle is not None
                            else active_overtake_target.lane_id
                            if active_overtake_target is not None
                            else None
                        ),
                        distance_m=lead_distance_m,
                        speed_mps=lead_speed_mps,
                        relative_speed_mps=closing_speed_mps,
                        is_stopped=lead_speed_mps <= 0.3,
                    ),
                    obstacle_target=active_overtake_target,
                    left_lane=left_lane_snapshot,
                    right_lane=right_lane_snapshot,
                    active_signal_state=active_light.state if active_light is not None else None,
                    signal_stop_distance_m=(
                        active_light.stop_line_distance_m if active_light is not None else None
                    ),
                    allow_overtake=self.config.allow_overtake,
                    preferred_direction=self.config.preferred_overtake_direction,
                ),
                overtake_trigger_distance_m=self.config.overtake_trigger_distance_m,
                overtake_speed_delta_kmh=self.config.overtake_speed_delta_kmh,
                overtake_min_front_gap_m=self.config.overtake_min_front_gap_m,
                overtake_min_rear_gap_m=self.config.overtake_min_rear_gap_m,
                signal_suppression_distance_m=self.config.overtake_signal_suppression_distance_m,
            )
            if overtake_decision.planner_state == "lane_change_out" and overtake_decision.direction is not None:
                self._overtake_origin_lane_id = current_lane_id
                if self._start_overtake(overtake_decision.direction, route_index):
                    self._overtake_state = "lane_change_out"
                    self._overtake_direction = overtake_decision.direction
                    self._overtake_aborted = False
                    self._overtake_target_actor_id = (
                        active_overtake_target.primary_actor_id
                        if active_overtake_target is not None
                        else lead_vehicle.actor_id
                    )
                    self._overtake_target_member_actor_ids = (
                        active_overtake_target.member_actor_ids
                        if active_overtake_target is not None
                        else (lead_vehicle.actor_id,)
                    )
                    self._overtake_memory = OvertakeMemory(
                        state="lane_change_out",
                        direction=overtake_decision.direction,
                        origin_lane_id=current_lane_id,
                        target_lane_id=self._overtake_target_lane_id,
                        target_actor_id=self._overtake_target_actor_id,
                        target_actor_lane_id=(
                            active_overtake_target.lane_id
                            if active_overtake_target is not None
                            else lead_vehicle.lane_id
                        ),
                        target_kind=(
                            active_overtake_target.kind
                            if active_overtake_target is not None
                            else "single_actor"
                        ),
                        target_member_actor_ids=self._overtake_target_member_actor_ids,
                        target_exit_distance_m=(
                            active_overtake_target.exit_distance_m
                            if active_overtake_target is not None
                            else lead_distance_m
                        ),
                    )
                    planner_state = "lane_change_out"
                    event_flags["event_overtake_attempt"] = True
                    overtake_reject_reason = None
                else:
                    event_flags["event_unsafe_lane_change_reject"] = True
                    planner_state = "car_follow"
                    target_speed_kmh = follow_target_speed_kmh
                    self._overtake_target_actor_id = None
                    self._overtake_target_member_actor_ids = ()
                    self._overtake_memory = OvertakeMemory()
                    overtake_reject_reason = (
                        self._lane_change_path_failure_reason or "lane_change_path_failed"
                    )
            else:
                overtake_reject_reason = overtake_decision.reject_reason
                if (
                    overtake_reject_reason == "lead_out_of_range"
                    and self.config.allow_overtake
                    and lead_distance_m is not None
                    and lead_distance_m > self.config.overtake_trigger_distance_m
                    and lead_speed_mps <= 0.3
                ):
                    planner_state = "nominal_cruise"
                    target_speed_kmh = self.config.target_speed_kmh
                else:
                    planner_state = "car_follow"
                    target_speed_kmh = follow_target_speed_kmh
                    if overtake_reject_reason in {
                        "adjacent_front_gap_insufficient",
                        "adjacent_rear_gap_insufficient",
                        "adjacent_lane_closed",
                        "signal_suppressed",
                    }:
                        event_flags["event_unsafe_lane_change_reject"] = True

        target_actor = None
        target_actor_visible = False
        if self._overtake_target_actor_id is not None:
            target_actor, visible_target_actors = self._visible_overtake_target_actors(
                scene_state.tracked_objects
            )
            target_actor_visible = bool(visible_target_actors)
            target_longitudinal_distance_m = None
            target_exit_longitudinal_distance_m = None
            visible_route_relative_progress_m = []
            if route_index is not None:
                for visible_actor in visible_target_actors:
                    route_relative_progress_m, _distance_to_centerline_m, _route_lane_id = (
                        self._route_relative_progress_to_actor(
                            actor=visible_actor,
                            reference_route_index=route_index,
                            search_back_points=48,
                            search_forward_points=96,
                        )
                    )
                    if route_relative_progress_m is not None:
                        visible_route_relative_progress_m.append(route_relative_progress_m)
                    if (
                        target_actor is not None
                        and visible_actor.actor_id == target_actor.actor_id
                        and route_relative_progress_m is not None
                    ):
                        target_longitudinal_distance_m = route_relative_progress_m
            if target_longitudinal_distance_m is None:
                target_longitudinal_distance_m = (
                    float(target_actor.longitudinal_distance_m)
                    if target_actor is not None and target_actor.longitudinal_distance_m is not None
                    else None
                )
            if visible_route_relative_progress_m:
                target_exit_longitudinal_distance_m = max(visible_route_relative_progress_m)
            else:
                visible_longitudinal_distances = [
                    float(actor.longitudinal_distance_m)
                    for actor in visible_target_actors
                    if actor.longitudinal_distance_m is not None
                ]
                if visible_longitudinal_distances:
                    target_exit_longitudinal_distance_m = max(visible_longitudinal_distances)
            self._overtake_memory = evaluate_pass_progress(
                self._overtake_memory,
                timestamp_s=scene_state.timestamp_s,
                target_actor_visible=target_actor_visible,
                target_longitudinal_distance_m=target_longitudinal_distance_m,
                overtake_resume_front_gap_m=self.config.overtake_resume_front_gap_m,
                target_kind=self._overtake_memory.target_kind,
                target_exit_longitudinal_distance_m=target_exit_longitudinal_distance_m,
            )
            if self._overtake_origin_lane_id is not None:
                rejoin_front_gap_m, rejoin_rear_gap_m = self._lane_gaps_for_lane_id(
                    scene_state.tracked_objects,
                    self._overtake_origin_lane_id,
                )
            if (
                self._overtake_state == "pass_vehicle"
                and not self._overtake_aborted
                and should_begin_rejoin(
                    self._overtake_memory,
                    rejoin_front_gap_m=(
                        None if not math.isfinite(rejoin_front_gap_m) else rejoin_front_gap_m
                    ),
                    rejoin_rear_gap_m=(
                        None if not math.isfinite(rejoin_rear_gap_m) else rejoin_rear_gap_m
                    ),
                    overtake_min_front_gap_m=self.config.overtake_min_front_gap_m,
                    overtake_min_rear_gap_m=self.config.overtake_min_rear_gap_m,
                )
                and self._start_rejoin(route_index)
            ):
                self._overtake_state = "lane_change_back"
                planner_state = "lane_change_back"
            self._overtake_memory.state = self._overtake_state

        if planner_state == "car_follow" and not self._car_follow_active:
            event_flags["event_car_follow_start"] = True
        self._car_follow_active = planner_state == "car_follow"

        if planner_state == "traffic_light_stop" and not self._waiting_on_light:
            event_flags["event_traffic_light_stop"] = True
        if planner_state != "traffic_light_stop" and self._waiting_on_light:
            event_flags["event_traffic_light_resume"] = True
        self._waiting_on_light = planner_state == "traffic_light_stop"

        if planner_state != self._previous_planner_state:
            self._longitudinal_speed_error_kmh = 0.0
        self._previous_planner_state = planner_state

        self._agent.set_target_speed(max(0.0, target_speed_kmh))
        target_waypoint = (
            self._consume_overtake_waypoint() if self._overtake_state != "idle" else None
        )
        if target_waypoint is not None:
            control = self._overtake_controller.run_step(
                max(0.0, target_speed_kmh), target_waypoint
            )
        else:
            control = self._agent.run_step()

        emergency_stop = (
            not self.config.ignore_vehicles
            and same_lane_lead_distance_m is not None
            and (
                same_lane_lead_distance_m <= self.config.lead_brake_distance_m
                or (
                    math.isfinite(same_lane_min_ttc)
                    and same_lane_min_ttc <= self.config.ttc_emergency_threshold_s
                )
            )
        )
        if planner_state == "traffic_light_stop":
            traffic_light_throttle, traffic_light_brake = self._traffic_light_stop_control(
                current_speed_mps=current_speed_mps,
                light=active_light,
                stop_target_distance_m=stop_target_distance_m,
                target_speed_kmh=target_speed_kmh,
            )
            control.throttle = traffic_light_throttle
            control.brake = traffic_light_brake
        else:
            control.throttle, control.brake = self._speed_control(
                current_speed_mps=current_speed_mps,
                target_speed_kmh=target_speed_kmh,
                max_throttle=0.75,
                max_brake=0.8,
            )
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
                "current_lane_id": current_lane_id,
                "lane_center_offset_m": lane_center_offset_m,
                "route_target_lane_id": scene_state.route.target_lane_id,
                "left_lane_open": bool(scene_state.ego.adjacent_lanes_open.get("left", False)),
                "right_lane_open": bool(scene_state.ego.adjacent_lanes_open.get("right", False)),
                "traffic_light_actor_id": active_light.actor_id
                if active_light is not None
                else None,
                "traffic_light_state": active_light.state if active_light is not None else None,
                "traffic_light_distance_m": active_light.distance_m
                if active_light is not None
                else None,
                "traffic_light_stop_line_distance_m": (
                    active_light.stop_line_distance_m if active_light is not None else None
                ),
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
                "lead_vehicle_id": (
                    lead_vehicle.actor_id
                    if lead_vehicle is not None
                    else active_overtake_target.primary_actor_id
                    if active_overtake_target is not None
                    else None
                ),
                "lead_vehicle_distance_m": lead_distance_m,
                "lead_vehicle_speed_mps": lead_speed_mps if lead_vehicle is not None else None,
                "lead_vehicle_relative_speed_mps": closing_speed_mps
                if lead_vehicle is not None
                else None,
                "lead_vehicle_lane_id": (
                    lead_vehicle.lane_id
                    if lead_vehicle is not None
                    else active_overtake_target.lane_id
                    if active_overtake_target is not None
                    else None
                ),
                "left_lane_front_gap_m": None
                if not math.isfinite(left_front_gap_m)
                else float(left_front_gap_m),
                "left_lane_rear_gap_m": None
                if not math.isfinite(left_rear_gap_m)
                else float(left_rear_gap_m),
                "right_lane_front_gap_m": None
                if not math.isfinite(right_front_gap_m)
                else float(right_front_gap_m),
                "right_lane_rear_gap_m": None
                if not math.isfinite(right_rear_gap_m)
                else float(right_rear_gap_m),
                "rejoin_front_gap_m": None
                if not math.isfinite(rejoin_front_gap_m)
                else float(rejoin_front_gap_m),
                "rejoin_rear_gap_m": None
                if not math.isfinite(rejoin_rear_gap_m)
                else float(rejoin_rear_gap_m),
                "overtake_considered": overtake_considered,
                "overtake_reject_reason": overtake_reject_reason,
                "overtake_state": self._overtake_state,
                "overtake_direction": self._overtake_direction,
                "overtake_origin_lane_id": self._overtake_origin_lane_id,
                "overtake_target_actor_id": self._overtake_target_actor_id,
                "overtake_target_kind": self._overtake_memory.target_kind,
                "overtake_target_member_actor_ids": list(
                    self._overtake_target_member_actor_ids
                ),
                "overtake_target_lane_id": self._overtake_target_lane_id,
                "target_passed": self._overtake_memory.target_passed,
                "distance_past_target_m": self._overtake_memory.target_pass_distance_m,
                "target_actor_visible": target_actor_visible,
                "target_actor_last_seen_s": self._overtake_memory.target_actor_last_seen_s,
                "lane_change_path_available": self._lane_change_path_available,
                "lane_change_path_failed_reason": self._lane_change_path_failure_reason,
                "target_lane_id": target_lane_id,
                "min_ttc": None if not math.isfinite(min_ttc) else float(min_ttc),
                "emergency_stop": emergency_stop,
                **event_flags,
            },
        )
