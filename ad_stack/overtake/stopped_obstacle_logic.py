from __future__ import annotations

from dataclasses import dataclass, replace
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
    "double_stopped_obstacle",
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
class StoppedObstacleTargetSnapshot:
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
class StoppedObstacleContext:
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
    obstacle_target: StoppedObstacleTargetSnapshot | None = None


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


@dataclass(slots=True)
class PreflightValidationInput:
    scenario_kind: ScenarioKind
    ego_lane_id: str | None
    obstacle_lane_id: str | None
    obstacle_route_lane_id: str | None = None
    blocker_lane_id: str | None = None
    ego_to_obstacle_longitudinal_distance_m: float | None = None
    ego_to_blocker_longitudinal_distance_m: float | None = None
    left_lane_is_driving: bool = False
    right_lane_is_driving: bool = False
    route_target_lane_id: str | None = None
    nearest_signal_distance_m: float | None = None
    nearest_junction_distance_m: float | None = None
    route_aligned_adjacent_lane_available: bool = False


@dataclass(slots=True)
class ScenarioValidationResult:
    scenario_kind: ScenarioKind
    errors: list[str]
    warnings: list[str]

    @property
    def is_valid(self) -> bool:
        return not self.errors


def _lane_gap_value(gap_m: float | None) -> float:
    return float("inf") if gap_m is None else float(gap_m)


def lane_gap_for_lane_id(
    lane_gaps_by_lane_id: dict[str, AdjacentLaneGapSnapshot], lane_id: str | None
) -> AdjacentLaneGapSnapshot | None:
    if lane_id is None:
        return None
    return lane_gaps_by_lane_id.get(lane_id)


def choose_overtake_action(
    context: StoppedObstacleContext,
    *,
    overtake_trigger_distance_m: float,
    overtake_speed_delta_kmh: float,
    overtake_min_front_gap_m: float,
    overtake_min_rear_gap_m: float,
    signal_suppression_distance_m: float,
) -> OvertakeDecision:
    active_target = context.obstacle_target
    target_distance_m = active_target.entry_distance_m if active_target is not None else None
    target_speed_mps = active_target.speed_mps if active_target is not None else None
    target_is_stopped = active_target.is_stopped if active_target is not None else None
    if not context.allow_overtake:
        return OvertakeDecision("car_follow", reject_reason="overtake_disabled")
    if target_distance_m is None and context.lead is not None:
        target_distance_m = context.lead.distance_m
        target_speed_mps = context.lead.speed_mps
        target_is_stopped = context.lead.is_stopped
    if target_distance_m is None:
        return OvertakeDecision("car_follow", reject_reason="lead_distance_unavailable")
    if target_distance_m > overtake_trigger_distance_m:
        return OvertakeDecision("car_follow", reject_reason="lead_out_of_range")
    if not bool(target_is_stopped) and (float(target_speed_mps or 0.0) * 3.6) > (
        context.target_speed_kmh - overtake_speed_delta_kmh
    ):
        return OvertakeDecision("car_follow", reject_reason="lead_not_slow_enough")
    if (
        context.active_signal_state in {"red", "yellow"}
        and context.signal_stop_distance_m is not None
        and context.signal_stop_distance_m <= signal_suppression_distance_m
    ):
        return OvertakeDecision("car_follow", reject_reason="signal_suppressed")
    if active_target is not None and not active_target.adjacent_lane_available:
        return OvertakeDecision("car_follow", reject_reason="adjacent_lane_closed")

    candidates: list[tuple[Literal["left", "right"], AdjacentLaneGapSnapshot | None]] = []
    if context.preferred_direction == "left_first":
        candidates = [("left", context.left_lane), ("right", context.right_lane)]
    else:
        candidates = [("right", context.right_lane), ("left", context.left_lane)]

    any_lane_open = False
    reject_reason: str | None = None
    for direction, lane in candidates:
        if lane is None or not lane.lane_open:
            continue
        any_lane_open = True
        if _lane_gap_value(lane.front_gap_m) < overtake_min_front_gap_m:
            reject_reason = "adjacent_front_gap_insufficient"
            continue
        if _lane_gap_value(lane.rear_gap_m) < overtake_min_rear_gap_m:
            reject_reason = "adjacent_rear_gap_insufficient"
            continue
        return OvertakeDecision(
            planner_state="lane_change_out",
            target_lane_id=lane.lane_id,
            direction=direction,
        )

    if not any_lane_open:
        reject_reason = "adjacent_lane_closed"
    return OvertakeDecision("car_follow", reject_reason=reject_reason)


def evaluate_pass_progress(
    memory: OvertakeMemory,
    *,
    timestamp_s: float,
    target_actor_visible: bool,
    target_longitudinal_distance_m: float | None,
    overtake_resume_front_gap_m: float,
    target_kind: TargetKind | None = None,
    target_exit_longitudinal_distance_m: float | None = None,
) -> OvertakeMemory:
    updated = replace(memory)
    effective_target_kind = target_kind or memory.target_kind
    effective_target_longitudinal_distance_m = target_longitudinal_distance_m
    if effective_target_kind == "cluster" and target_exit_longitudinal_distance_m is not None:
        effective_target_longitudinal_distance_m = float(target_exit_longitudinal_distance_m)

    if target_actor_visible and effective_target_longitudinal_distance_m is not None:
        updated.target_kind = effective_target_kind
        updated.target_actor_last_seen_s = float(timestamp_s)
        updated.target_actor_last_seen_longitudinal_m = float(effective_target_longitudinal_distance_m)
        updated.target_exit_distance_m = effective_target_longitudinal_distance_m
        if effective_target_longitudinal_distance_m < 0.0:
            distance_past_target_m = abs(float(effective_target_longitudinal_distance_m))
            updated.target_pass_distance_m = distance_past_target_m
            if distance_past_target_m >= overtake_resume_front_gap_m:
                updated.target_passed = True
        return updated

    if (
        updated.target_actor_last_seen_s is not None
        and (timestamp_s - updated.target_actor_last_seen_s) > updated.target_actor_visibility_timeout_s
    ):
        updated.target_passed = False
    return updated


def build_stopped_obstacle_targets(
    leads: list[OvertakeLeadSnapshot],
    *,
    cluster_merge_gap_m: float,
    cluster_max_member_speed_mps: float,
) -> list[StoppedObstacleTargetSnapshot]:
    stopped_leads = [
        lead
        for lead in leads
        if (
            lead.actor_id is not None
            and lead.lane_id is not None
            and lead.distance_m is not None
            and lead.is_stopped
        )
    ]
    stopped_leads.sort(key=lambda lead: (lead.lane_id or "", float(lead.distance_m or 0.0)))
    targets: list[StoppedObstacleTargetSnapshot] = []
    current_cluster: list[OvertakeLeadSnapshot] = []
    for lead in stopped_leads:
        if not current_cluster:
            current_cluster = [lead]
            continue
        previous = current_cluster[-1]
        same_lane = previous.lane_id == lead.lane_id
        gap_m = float(lead.distance_m) - float(previous.distance_m)
        speed_ok = max(previous.speed_mps, lead.speed_mps) <= cluster_max_member_speed_mps
        if same_lane and gap_m <= cluster_merge_gap_m and speed_ok:
            current_cluster.append(lead)
            continue
        targets.append(_cluster_to_target(current_cluster))
        current_cluster = [lead]
    if current_cluster:
        targets.append(_cluster_to_target(current_cluster))
    return targets


def next_stopped_obstacle_target(
    targets: list[StoppedObstacleTargetSnapshot],
    *,
    current_primary_actor_id: int | None,
) -> StoppedObstacleTargetSnapshot | None:
    if current_primary_actor_id is None:
        return targets[0] if targets else None
    for index, target in enumerate(targets):
        if target.primary_actor_id == current_primary_actor_id:
            return targets[index + 1] if index + 1 < len(targets) else None
    return targets[0] if targets else None


def _cluster_to_target(cluster: list[OvertakeLeadSnapshot]) -> StoppedObstacleTargetSnapshot:
    first = cluster[0]
    last = cluster[-1]
    member_actor_ids = tuple(
        lead.actor_id for lead in cluster if lead.actor_id is not None
    )
    kind: TargetKind = "cluster" if len(member_actor_ids) > 1 else "single_actor"
    return StoppedObstacleTargetSnapshot(
        kind=kind,
        primary_actor_id=int(first.actor_id),
        member_actor_ids=member_actor_ids,
        lane_id=str(first.lane_id),
        entry_distance_m=float(first.distance_m),
        exit_distance_m=float(last.distance_m),
        speed_mps=max(lead.speed_mps for lead in cluster),
        is_stopped=all(lead.is_stopped for lead in cluster),
    )


def should_begin_rejoin(
    memory: OvertakeMemory,
    *,
    rejoin_front_gap_m: float | None,
    rejoin_rear_gap_m: float | None,
    overtake_min_front_gap_m: float,
    overtake_min_rear_gap_m: float,
) -> bool:
    if not memory.target_passed:
        return False
    if _lane_gap_value(rejoin_front_gap_m) < overtake_min_front_gap_m:
        return False
    if _lane_gap_value(rejoin_rear_gap_m) < overtake_min_rear_gap_m:
        return False
    return True


def build_route_aligned_lane_change_plan(
    origin_lane_samples: list[LaneChangePlanPoint],
    target_lane_samples: list[LaneChangePlanPoint],
    *,
    distance_same_lane_m: float,
    lane_change_distance_m: float,
    distance_other_lane_m: float,
) -> LaneChangePlanResult:
    if not origin_lane_samples or not target_lane_samples:
        return LaneChangePlanResult(False, [], "missing_lane_samples")
    start_progress_m = origin_lane_samples[0].progress_m
    for previous, current in zip(origin_lane_samples, origin_lane_samples[1:], strict=False):
        if current.progress_m < previous.progress_m:
            return LaneChangePlanResult(False, [], "origin_progress_not_monotonic")
    for previous, current in zip(target_lane_samples, target_lane_samples[1:], strict=False):
        if current.progress_m < previous.progress_m:
            return LaneChangePlanResult(False, [], "target_progress_not_monotonic")

    same_lane_end_m = start_progress_m + distance_same_lane_m
    lane_change_end_m = same_lane_end_m + lane_change_distance_m
    target_lane_end_m = lane_change_end_m + distance_other_lane_m

    plan_points = [
        sample for sample in origin_lane_samples if sample.progress_m <= same_lane_end_m + 1e-6
    ]
    target_points = [
        sample
        for sample in target_lane_samples
        if sample.progress_m >= lane_change_end_m - 1e-6
        and sample.progress_m <= target_lane_end_m + 1e-6
    ]
    if not target_points:
        return LaneChangePlanResult(False, plan_points, "adjacent_lane_sample_missing")
    plan_points.extend(target_points)
    for previous, current in zip(plan_points, plan_points[1:], strict=False):
        if current.progress_m < previous.progress_m:
            return LaneChangePlanResult(False, plan_points, "plan_progress_reversed")
    return LaneChangePlanResult(True, plan_points)


def validate_preflight(snapshot: PreflightValidationInput) -> ScenarioValidationResult:
    errors: list[str] = []
    warnings: list[str] = []

    obstacle_aligned_with_route = (
        snapshot.obstacle_route_lane_id is not None
        and snapshot.obstacle_lane_id == snapshot.obstacle_route_lane_id
    )
    obstacle_expected_in_current_or_future_lane = snapshot.obstacle_lane_id == snapshot.ego_lane_id
    if snapshot.scenario_kind in {"curve_clear", "adjacent_lane_closed"}:
        obstacle_expected_in_current_or_future_lane = (
            obstacle_expected_in_current_or_future_lane or obstacle_aligned_with_route
        )
    if not obstacle_expected_in_current_or_future_lane:
        errors.append("obstacle_not_in_ego_lane")
    if (
        snapshot.ego_to_obstacle_longitudinal_distance_m is None
        or snapshot.ego_to_obstacle_longitudinal_distance_m <= 0.0
    ):
        errors.append("obstacle_not_ahead")
    if snapshot.scenario_kind != "adjacent_lane_closed":
        if not (snapshot.left_lane_is_driving or snapshot.right_lane_is_driving):
            errors.append("no_adjacent_driving_lane")
        if not snapshot.route_aligned_adjacent_lane_available:
            errors.append("route_aligned_adjacent_lane_unavailable")
    if (
        snapshot.route_target_lane_id is not None
        and snapshot.ego_lane_id is not None
        and snapshot.route_target_lane_id != snapshot.ego_lane_id
    ):
        errors.append("route_target_lane_conflict")
    signal_nearby = (
        snapshot.nearest_signal_distance_m is not None
        and snapshot.nearest_signal_distance_m < 25.0
    )
    junction_nearby = (
        snapshot.nearest_junction_distance_m is not None
        and snapshot.nearest_junction_distance_m < 30.0
    )
    if signal_nearby:
        warnings.append("signal_nearby")
    if junction_nearby:
        warnings.append("junction_nearby")

    if snapshot.scenario_kind == "blocked_static":
        if snapshot.blocker_lane_id is None:
            errors.append("blocker_missing")
        elif snapshot.blocker_lane_id == snapshot.ego_lane_id:
            errors.append("blocker_not_in_opposite_lane")

    if snapshot.scenario_kind == "blocked_oncoming":
        if snapshot.blocker_lane_id is None:
            errors.append("oncoming_blocker_missing")

    if snapshot.scenario_kind in {"double_stopped_separated", "double_stopped_clustered"}:
        if snapshot.obstacle_lane_id is None:
            errors.append("obstacle_actor_missing")

    if snapshot.scenario_kind == "adjacent_lane_closed":
        if snapshot.route_aligned_adjacent_lane_available or (
            snapshot.left_lane_is_driving or snapshot.right_lane_is_driving
        ):
            errors.append("adjacent_lane_not_closed")

    if snapshot.scenario_kind == "near_junction_preflight_reject":
        if signal_nearby:
            errors.append("signal_nearby")
        if junction_nearby:
            errors.append("junction_nearby")

    return ScenarioValidationResult(
        scenario_kind=snapshot.scenario_kind,
        errors=errors,
        warnings=warnings,
    )
