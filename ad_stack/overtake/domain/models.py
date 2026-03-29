from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

PlannerState = Literal[
    "nominal_cruise",
    "car_follow",
    "lane_change_out",
    "pass_vehicle",
    "lane_change_back",
    "abort_return",
]
PreferredDirection = Literal["left_first", "right_first"]
TargetKind = Literal["single_actor", "cluster"]
MotionProfile = Literal["stopped", "slow", "moving"]


@dataclass(slots=True)
class OvertakeLeadSnapshot:
    actor_id: int | None
    lane_id: str | None
    distance_m: float | None
    speed_mps: float
    relative_speed_mps: float
    motion_profile: MotionProfile


@dataclass(slots=True)
class OvertakeTargetSnapshot:
    kind: TargetKind
    primary_actor_id: int
    member_actor_ids: tuple[int, ...]
    lane_id: str
    entry_distance_m: float
    exit_distance_m: float
    speed_mps: float
    motion_profile: MotionProfile
    adjacent_lane_available: bool = True


@dataclass(slots=True)
class OvertakeTargetCandidates:
    same_lane: list[OvertakeLeadSnapshot]
    route_aligned: list[OvertakeLeadSnapshot]


@dataclass(slots=True)
class AdjacentLaneGapSnapshot:
    lane_id: str | None
    front_gap_m: float | None
    rear_gap_m: float | None
    lane_open: bool


@dataclass(slots=True)
class OvertakeContext:
    timestamp_s: float
    current_lane_id: str | None
    origin_lane_id: str | None
    route_target_lane_id: str | None
    target_speed_kmh: float
    lead: OvertakeLeadSnapshot | None
    left_lane: AdjacentLaneGapSnapshot | None
    right_lane: AdjacentLaneGapSnapshot | None
    active_signal_state: str | None
    signal_stop_distance_m: float | None
    allow_overtake: bool
    preferred_direction: PreferredDirection
    active_target: OvertakeTargetSnapshot | None = None


@dataclass(slots=True)
class OvertakeMemory:
    state: PlannerState | Literal["idle"] = "idle"
    direction: Literal["left", "right"] | None = None
    origin_lane_id: str | None = None
    target_lane_id: str | None = None
    target_actor_id: int | None = None
    target_actor_lane_id: str | None = None
    target_actor_last_seen_s: float | None = None
    target_actor_last_seen_longitudinal_m: float | None = None
    target_actor_visibility_timeout_s: float = 1.0
    target_kind: TargetKind = "single_actor"
    target_member_actor_ids: tuple[int, ...] = ()
    target_exit_distance_m: float | None = None
    pass_started_s: float | None = None
    pass_started_route_index: int | None = None
    target_passed: bool = False
    target_pass_distance_m: float | None = None
    abort_reason: str | None = None


@dataclass(slots=True)
class OvertakeDecision:
    planner_state: PlannerState
    target_lane_id: str | None = None
    direction: Literal["left", "right"] | None = None
    reject_reason: str | None = None


@dataclass(slots=True)
class OvertakeEventFlags:
    traffic_light_stop: bool = False
    traffic_light_resume: bool = False
    car_follow_start: bool = False
    overtake_attempt: bool = False
    overtake_success: bool = False
    overtake_abort: bool = False
    unsafe_lane_change_reject: bool = False
    traffic_light_violation: bool = False


@dataclass(slots=True)
class OvertakeCoreTelemetry:
    behavior_state: str
    route_command: str
    remaining_waypoints: int
    route_progress_index: int | None
    max_route_index: int
    current_lane_id: str | None
    lane_center_offset_m: float | None
    route_target_lane_id: str | None
    left_lane_open: bool
    right_lane_open: bool
    traffic_light_actor_id: int | None
    traffic_light_state: str | None
    traffic_light_distance_m: float | None
    traffic_light_stop_line_distance_m: float | None
    traffic_light_red_latched: bool
    traffic_light_stop_buffer_m: float
    traffic_light_stop_target_distance_m: float | None
    target_speed_kmh: float
    desired_speed_mps: float
    applied_speed_mps: float
    lookahead_distance_m: float
    lateral_error_m: float | None
    heading_error_deg: float | None
    controller_steer_raw: float
    controller_steer_applied: float
    target_lane_id: str | None
    min_ttc: float | None
    event_flags: OvertakeEventFlags


@dataclass(slots=True)
class OvertakeTargetTelemetry:
    follow_target_id: int | None
    follow_target_distance_m: float | None
    follow_target_speed_mps: float | None
    follow_target_relative_speed_mps: float | None
    follow_target_lane_id: str | None
    follow_target_motion_profile: MotionProfile | None
    left_lane_front_gap_m: float | None
    left_lane_rear_gap_m: float | None
    right_lane_front_gap_m: float | None
    right_lane_rear_gap_m: float | None
    rejoin_front_gap_m: float | None
    rejoin_rear_gap_m: float | None
    overtake_considered: bool
    overtake_reject_reason: str | None
    overtake_state: str
    overtake_direction: str | None
    overtake_origin_lane_id: str | None
    overtake_target_actor_id: int | None
    overtake_target_kind: str
    overtake_target_member_actor_ids: tuple[int, ...]
    overtake_target_lane_id: str | None
    overtake_target_motion_profile: MotionProfile | None
    target_passed: bool
    distance_past_target_m: float | None
    target_actor_visible: bool
    target_actor_last_seen_s: float | None
    lane_change_path_available: bool
    lane_change_path_failed_reason: str | None


@dataclass(slots=True)
class OvertakePlanningDebug:
    core: OvertakeCoreTelemetry
    target: OvertakeTargetTelemetry


@dataclass(slots=True)
class LaneChangePlanPoint:
    route_index: int
    lane_id: str
    progress_m: float


@dataclass(slots=True)
class LaneChangePlanResult:
    available: bool
    points: list[LaneChangePlanPoint]
    failure_reason: str | None = None
