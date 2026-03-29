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
ScenarioKind = Literal[
    "clear",
    "blocked_static",
    "blocked_oncoming",
    "signal_suppressed",
    "rejoin_blocked_then_release",
    "adjacent_lane_closed",
    "double_stopped_separated",
    "double_stopped_clustered",
    "curve_clear",
    "near_junction_preflight_reject",
]
TargetKind = Literal["single_actor", "cluster"]


@dataclass(slots=True)
class OvertakeLeadSnapshot:
    actor_id: int | None
    lane_id: str | None
    distance_m: float | None
    speed_mps: float
    relative_speed_mps: float
    is_stopped: bool


@dataclass(slots=True)
class OvertakeTargetSnapshot:
    kind: TargetKind
    primary_actor_id: int
    member_actor_ids: tuple[int, ...]
    lane_id: str
    entry_distance_m: float
    exit_distance_m: float
    speed_mps: float
    is_stopped: bool
    adjacent_lane_available: bool = True


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
    stopped_speed_threshold_mps: float
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
class LaneChangePlanPoint:
    route_index: int
    lane_id: str
    progress_m: float


@dataclass(slots=True)
class LaneChangePlanResult:
    available: bool
    points: list[LaneChangePlanPoint]
    failure_reason: str | None = None
