from __future__ import annotations

from .target_acceptance_policy import TargetAcceptanceRequest, TargetAcceptanceResult


def accept_stopped_overtake_target(
    request: TargetAcceptanceRequest,
) -> TargetAcceptanceResult:
    target_distance_m = request.target_distance_m
    if target_distance_m is None:
        return TargetAcceptanceResult(
            accepted=False,
            reject_reason="follow_target_distance_unavailable",
        )
    if target_distance_m > request.overtake_trigger_distance_m:
        planner_state_on_reject = "car_follow"
        if bool(request.active_target is not None and request.active_target.motion_profile == "stopped"):
            planner_state_on_reject = "nominal_cruise"
        elif bool(request.lead is not None and request.lead.motion_profile == "stopped"):
            planner_state_on_reject = "nominal_cruise"
        return TargetAcceptanceResult(
            accepted=False,
            reject_reason="target_out_of_range",
            planner_state_on_reject=planner_state_on_reject,
        )
    target_speed_kmh = float(request.target_speed_mps or 0.0) * 3.6
    target_is_stopped = (
        request.active_target.motion_profile == "stopped"
        if request.active_target is not None
        else request.lead.motion_profile == "stopped"
        if request.lead is not None
        else False
    )
    if (not target_is_stopped) and target_speed_kmh > (
        request.context.target_speed_kmh - request.overtake_speed_delta_kmh
    ):
        return TargetAcceptanceResult(
            accepted=False,
            reject_reason="target_not_slow_enough",
        )
    return TargetAcceptanceResult(accepted=True)
