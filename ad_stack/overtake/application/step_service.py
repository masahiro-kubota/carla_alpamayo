from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

from ad_stack.overtake.application.control_profile import traffic_light_stop_target_speed_kmh
from ad_stack.overtake.application.decision_service import (
    choose_overtake_action,
    should_begin_rejoin,
)
from ad_stack.overtake.application.runtime_state import OvertakeRuntimeState
from ad_stack.overtake.application.runtime_transition import (
    _lane_change_entry_target_speed_kmh,
    resolve_overtake_runtime_transition,
)
from ad_stack.overtake.domain import OvertakeContext, OvertakeEventFlags
from ad_stack.overtake.policies import TargetAcceptancePolicy


@dataclass(frozen=True, slots=True)
class OvertakeStepRequest:
    runtime_state: OvertakeRuntimeState
    decision_context: OvertakeContext
    target_acceptance_policy: TargetAcceptancePolicy
    stop_for_light: bool
    ignore_traffic_lights: bool
    ignore_vehicles: bool
    lead_vehicle_present: bool
    active_target_present: bool
    lead_distance_m: float | None
    lead_speed_mps: float
    follow_target_speed_kmh: float
    current_speed_mps: float
    current_lane_id: str | None
    route_target_lane_id: str | None
    lane_center_offset_m: float | None
    target_speed_kmh: float
    active_light_state: str | None
    signal_stop_distance_m: float | None
    traffic_light_brake_start_distance_m: float
    traffic_light_creep_resume_distance_m: float
    traffic_light_creep_speed_kmh: float
    overtake_target_speed_kmh: float
    overtake_trigger_distance_m: float
    overtake_min_front_gap_m: float
    overtake_min_rear_gap_m: float
    overtake_signal_suppression_distance_m: float
    rejoin_front_gap_m: float | None = None
    rejoin_rear_gap_m: float | None = None


@dataclass(frozen=True, slots=True)
class OvertakeStepDecision:
    planner_state: str
    target_speed_kmh: float
    overtake_considered: bool = False
    overtake_reject_reason: str | None = None
    next_runtime_state: str | None = None
    next_aborted: bool | None = None
    completed: bool = False
    request_overtake_direction: Literal["left", "right"] | None = None
    request_prepare_abort_return: bool = False
    request_rejoin: bool = False
    event_flags: OvertakeEventFlags = field(default_factory=OvertakeEventFlags)


def resolve_overtake_step(request: OvertakeStepRequest) -> OvertakeStepDecision:
    runtime_state = request.runtime_state
    if runtime_state.state != "idle":
        transition = resolve_overtake_runtime_transition(
            state=runtime_state.state,
            aborted=runtime_state.aborted,
            current_lane_id=request.current_lane_id,
            target_lane_id=runtime_state.memory.target_lane_id,
            origin_lane_id=runtime_state.origin_lane_id,
            route_target_lane_id=request.route_target_lane_id,
            lane_center_offset_m=request.lane_center_offset_m,
            should_stop_for_light=request.stop_for_light,
            target_speed_kmh=request.target_speed_kmh,
            follow_target_speed_kmh=request.follow_target_speed_kmh,
            lead_speed_kmh=request.lead_speed_mps * 3.6,
            overtake_target_speed_kmh=request.overtake_target_speed_kmh,
        )
        target_speed_kmh = request.target_speed_kmh
        if transition.phase_target_speed_kmh is not None:
            target_speed_kmh = transition.phase_target_speed_kmh
        request_rejoin = (
            transition.state == "pass_vehicle"
            and not transition.aborted
            and should_begin_rejoin(
                runtime_state.memory,
                rejoin_front_gap_m=request.rejoin_front_gap_m,
                rejoin_rear_gap_m=request.rejoin_rear_gap_m,
                overtake_min_front_gap_m=request.overtake_min_front_gap_m,
                overtake_min_rear_gap_m=request.overtake_min_rear_gap_m,
            )
        )
        return OvertakeStepDecision(
            planner_state=transition.planner_state,
            target_speed_kmh=target_speed_kmh,
            next_runtime_state=transition.state,
            next_aborted=transition.aborted,
            completed=transition.completed,
            request_prepare_abort_return=transition.should_prepare_abort_return,
            request_rejoin=request_rejoin,
            event_flags=OvertakeEventFlags(
                overtake_abort=transition.event_overtake_abort,
                overtake_success=transition.event_overtake_success,
            ),
        )

    if request.stop_for_light:
        target_speed_kmh = traffic_light_stop_target_speed_kmh(
            stop_target_distance_m=request.signal_stop_distance_m,
            current_speed_mps=request.current_speed_mps,
            target_speed_kmh=request.target_speed_kmh,
            brake_start_distance_m=request.traffic_light_brake_start_distance_m,
            creep_resume_distance_m=request.traffic_light_creep_resume_distance_m,
            creep_speed_kmh=request.traffic_light_creep_speed_kmh,
        )
        event_flags = OvertakeEventFlags(traffic_light_stop=True)
        overtake_reject_reason: str | None = None
        overtake_considered = False
        if request.lead_vehicle_present and not request.ignore_vehicles:
            target_speed_kmh = min(target_speed_kmh, request.follow_target_speed_kmh)
            if (
                request.decision_context.allow_overtake
                and request.active_light_state in {"red", "yellow"}
                and request.signal_stop_distance_m is not None
                and request.signal_stop_distance_m
                <= request.overtake_signal_suppression_distance_m
            ):
                overtake_considered = True
                overtake_reject_reason = "signal_suppressed"
                event_flags = OvertakeEventFlags(
                    traffic_light_stop=True,
                    unsafe_lane_change_reject=True,
                )
        return OvertakeStepDecision(
            planner_state="traffic_light_stop",
            target_speed_kmh=target_speed_kmh,
            overtake_considered=overtake_considered,
            overtake_reject_reason=overtake_reject_reason,
            event_flags=event_flags,
        )

    if (
        (request.lead_vehicle_present or request.active_target_present)
        and not request.ignore_vehicles
    ):
        overtake_decision = choose_overtake_action(
            request.decision_context,
            overtake_trigger_distance_m=request.overtake_trigger_distance_m,
            overtake_target_speed_kmh=request.overtake_target_speed_kmh,
            overtake_min_front_gap_m=request.overtake_min_front_gap_m,
            overtake_min_rear_gap_m=request.overtake_min_rear_gap_m,
            signal_suppression_distance_m=request.overtake_signal_suppression_distance_m,
            target_acceptance_policy=request.target_acceptance_policy,
        )
        if (
            overtake_decision.planner_state == "lane_change_out"
            and overtake_decision.direction is not None
        ):
            return OvertakeStepDecision(
                planner_state="lane_change_out",
                target_speed_kmh=_lane_change_entry_target_speed_kmh(
                    overtake_target_speed_kmh=request.overtake_target_speed_kmh,
                    follow_target_speed_kmh=request.follow_target_speed_kmh,
                    lead_speed_kmh=request.lead_speed_mps * 3.6,
                ),
                overtake_considered=True,
                request_overtake_direction=overtake_decision.direction,
            )

        reject_reason = overtake_decision.reject_reason
        if overtake_decision.planner_state == "nominal_cruise":
            return OvertakeStepDecision(
                planner_state="nominal_cruise",
                target_speed_kmh=request.target_speed_kmh,
                overtake_considered=True,
                overtake_reject_reason=reject_reason,
            )

        event_flags = OvertakeEventFlags()
        if reject_reason in {
            "adjacent_front_gap_insufficient",
            "adjacent_rear_gap_insufficient",
            "adjacent_lane_closed",
            "signal_suppressed",
        }:
            event_flags = OvertakeEventFlags(unsafe_lane_change_reject=True)
        return OvertakeStepDecision(
            planner_state="car_follow",
            target_speed_kmh=request.follow_target_speed_kmh,
            overtake_considered=True,
            overtake_reject_reason=reject_reason,
            event_flags=event_flags,
        )

    return OvertakeStepDecision(
        planner_state="nominal_cruise",
        target_speed_kmh=request.target_speed_kmh,
    )
