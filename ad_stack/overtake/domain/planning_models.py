from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

BehaviorState = Literal[
    "lane_follow",
    "car_follow",
    "lane_change_out",
    "pass_vehicle",
    "lane_change_back",
    "abort_return",
    "signal_stop",
]
RouteCommand = Literal["lane_follow", "left", "right", "straight"]
ActiveTargetKind = Literal["single_actor", "cluster"]
TrafficLightState = Literal["red", "yellow", "green", "unknown"]


@dataclass(frozen=True, slots=True)
class Pose3D:
    x: float
    y: float
    z: float
    yaw_deg: float


@dataclass(frozen=True, slots=True)
class RouteTracePoint:
    x: float
    y: float
    z: float
    yaw_deg: float
    lane_id: str
    road_option: RouteCommand


@dataclass(frozen=True, slots=True)
class RouteBackbone:
    trace: tuple[RouteTracePoint, ...]
    xy_points: tuple[tuple[float, float], ...]
    progress_m: tuple[float, ...]
    route_index_to_trace_index: tuple[int, ...]
    route_index_to_road_option: tuple[RouteCommand, ...]
    route_index_to_lane_id: tuple[str, ...]

    def __post_init__(self) -> None:
        trace_length = len(self.trace)
        if trace_length < 2:
            raise ValueError("route backbone requires at least two trace points")
        if len(self.xy_points) != trace_length or len(self.progress_m) != trace_length:
            raise ValueError("trace, xy_points, and progress_m must have the same length")
        if len(self.route_index_to_trace_index) == 0:
            raise ValueError("route_index_to_trace_index must not be empty")
        if len(self.route_index_to_road_option) != len(self.route_index_to_trace_index):
            raise ValueError("route_index_to_road_option length must match route indices")
        if len(self.route_index_to_lane_id) != len(self.route_index_to_trace_index):
            raise ValueError("route_index_to_lane_id length must match route indices")
        previous_progress = self.progress_m[0]
        for progress in self.progress_m[1:]:
            if progress <= previous_progress:
                raise ValueError("route backbone progress must be strictly increasing")
            previous_progress = progress
        for trace_index in self.route_index_to_trace_index:
            if trace_index < 0 or trace_index >= trace_length:
                raise ValueError("route index mapping references an out-of-range trace index")

    def road_option_for_index(self, route_index: int) -> RouteCommand:
        return self.route_index_to_road_option[route_index]

    def lane_id_for_index(self, route_index: int) -> str:
        return self.route_index_to_lane_id[route_index]

    def trace_index_for_route_index(self, route_index: int) -> int:
        return self.route_index_to_trace_index[route_index]


@dataclass(frozen=True, slots=True)
class TrackedTarget:
    actor_id: int
    lane_id: str
    longitudinal_distance_m: float
    speed_mps: float
    target_kind: ActiveTargetKind = "single_actor"
    affects_route: bool = True


@dataclass(frozen=True, slots=True)
class TrafficLightObservation:
    actor_id: int
    state: TrafficLightState
    affects_ego: bool
    stop_line_distance_m: float | None


@dataclass(frozen=True, slots=True)
class AdjacentLaneAvailability:
    left_open: bool
    right_open: bool


@dataclass(frozen=True, slots=True)
class PlanningScene:
    ego_pose: Pose3D
    ego_speed_mps: float
    route_index: int
    route_progress_m: float
    current_lane_id: str
    tracked_targets: tuple[TrackedTarget, ...]
    traffic_lights: tuple[TrafficLightObservation, ...]
    adjacent_lane_availability: AdjacentLaneAvailability

    def __post_init__(self) -> None:
        if self.route_index < 0:
            raise ValueError("route_index must be non-negative")
        if self.route_progress_m < 0.0:
            raise ValueError("route_progress_m must be non-negative")
        if self.ego_speed_mps < 0.0:
            raise ValueError("ego_speed_mps must be non-negative")


@dataclass(frozen=True, slots=True)
class BehaviorPlan:
    state: BehaviorState
    route_command: RouteCommand
    active_target_id: int | None = None
    active_target_kind: ActiveTargetKind | None = None
    origin_lane_id: str | None = None
    target_lane_id: str | None = None
    reject_reason: str | None = None

    def __post_init__(self) -> None:
        if (self.active_target_id is None) != (self.active_target_kind is None):
            raise ValueError("active target id and kind must be provided together")


@dataclass(frozen=True, slots=True)
class TrajectoryPoint:
    x: float
    y: float
    z: float
    yaw_deg: float
    longitudinal_velocity_mps: float


@dataclass(frozen=True, slots=True)
class Trajectory:
    points: tuple[TrajectoryPoint, ...]
    trajectory_id: str
    origin_lane_id: str | None = None
    target_lane_id: str | None = None
    source_route_start_index: int | None = None
    source_route_end_index: int | None = None

    def __post_init__(self) -> None:
        if len(self.points) < 2:
            raise ValueError("trajectory requires at least two points")
        if self.source_route_start_index is not None and self.source_route_start_index < 0:
            raise ValueError("source_route_start_index must be non-negative")
        if self.source_route_end_index is not None and self.source_route_end_index < 0:
            raise ValueError("source_route_end_index must be non-negative")
        if (
            self.source_route_start_index is not None
            and self.source_route_end_index is not None
            and self.source_route_end_index < self.source_route_start_index
        ):
            raise ValueError("source_route_end_index must be >= source_route_start_index")

        for point in self.points:
            if point.longitudinal_velocity_mps < 0.0:
                raise ValueError("trajectory velocity must be non-negative")

        for previous, current in zip(self.points, self.points[1:]):
            distance_m = math.hypot(current.x - previous.x, current.y - previous.y)
            if not 0.8 <= distance_m <= 1.2:
                raise ValueError("trajectory spacing must stay near 1.0m")
            heading_deg = math.degrees(math.atan2(current.y - previous.y, current.x - previous.x))
            if _angle_difference_deg(heading_deg, previous.yaw_deg) > 45.0:
                raise ValueError("trajectory yaw must align with forward direction")


@dataclass(frozen=True, slots=True)
class BehaviorTrajectoryTelemetry:
    behavior_state: BehaviorState
    route_command: RouteCommand
    active_target_id: int | None
    active_target_kind: ActiveTargetKind | None
    reject_reason: str | None
    desired_speed_mps: float
    applied_speed_mps: float
    lateral_error_m: float | None
    heading_error_deg: float | None
    lookahead_distance_m: float
    controller_steer_raw: float
    controller_steer_applied: float


def _angle_difference_deg(a_deg: float, b_deg: float) -> float:
    wrapped = (a_deg - b_deg + 180.0) % 360.0 - 180.0
    return abs(wrapped)
