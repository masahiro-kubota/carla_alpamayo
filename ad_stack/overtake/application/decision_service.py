from __future__ import annotations

from dataclasses import replace

from ad_stack.overtake.domain import (
    OvertakeDecision,
    OvertakeContext,
    OvertakeMemory,
    TargetKind,
)
from ad_stack.overtake.policies import TargetAcceptancePolicy, TargetAcceptanceRequest


def _lane_gap_value(gap_m: float | None) -> float:
    return float("inf") if gap_m is None else float(gap_m)


def choose_overtake_action(
    context: OvertakeContext,
    *,
    overtake_trigger_distance_m: float,
    overtake_speed_delta_kmh: float,
    overtake_min_front_gap_m: float,
    overtake_min_rear_gap_m: float,
    signal_suppression_distance_m: float,
    target_acceptance_policy: TargetAcceptancePolicy,
) -> OvertakeDecision:
    active_target = context.active_target
    target_distance_m = active_target.entry_distance_m if active_target is not None else None
    target_speed_mps = active_target.speed_mps if active_target is not None else None
    if not context.allow_overtake:
        return OvertakeDecision("car_follow", reject_reason="overtake_disabled")
    if target_distance_m is None and context.lead is not None:
        target_distance_m = context.lead.distance_m
        target_speed_mps = context.lead.speed_mps
    target_acceptance = target_acceptance_policy(
        TargetAcceptanceRequest(
            context=context,
            lead=context.lead,
            active_target=active_target,
            target_distance_m=target_distance_m,
            target_speed_mps=target_speed_mps,
            overtake_trigger_distance_m=overtake_trigger_distance_m,
            overtake_speed_delta_kmh=overtake_speed_delta_kmh,
        )
    )
    if not target_acceptance.accepted:
        return OvertakeDecision(
            target_acceptance.planner_state_on_reject,
            reject_reason=target_acceptance.reject_reason,
        )
    if (
        context.active_signal_state in {"red", "yellow"}
        and context.signal_stop_distance_m is not None
        and context.signal_stop_distance_m <= signal_suppression_distance_m
    ):
        return OvertakeDecision("car_follow", reject_reason="signal_suppressed")
    if active_target is not None and not active_target.adjacent_lane_available:
        return OvertakeDecision("car_follow", reject_reason="adjacent_lane_closed")

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
        updated.target_actor_last_seen_longitudinal_m = float(
            effective_target_longitudinal_distance_m
        )
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
