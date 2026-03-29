from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from ad_stack.overtake.domain import PlannerState


@dataclass(slots=True)
class OvertakeRuntimeTransition:
    state: PlannerState | Literal["idle"]
    planner_state: PlannerState | Literal["nominal_cruise"]
    aborted: bool
    phase_target_speed_kmh: float | None = None
    should_prepare_abort_return: bool = False
    completed: bool = False
    event_overtake_abort: bool = False
    event_overtake_success: bool = False


def resolve_overtake_runtime_transition(
    *,
    state: PlannerState | Literal["idle"],
    aborted: bool,
    current_lane_id: str | None,
    target_lane_id: str | None,
    origin_lane_id: str | None,
    route_target_lane_id: str | None,
    lane_center_offset_m: float | None,
    should_stop_for_light: bool,
    target_speed_kmh: float,
    follow_target_speed_kmh: float,
    lead_speed_kmh: float,
    overtake_target_speed_kmh: float,
) -> OvertakeRuntimeTransition:
    if state == "idle":
        return OvertakeRuntimeTransition(
            state="idle",
            planner_state="nominal_cruise",
            aborted=aborted,
        )

    next_state = state
    planner_state: PlannerState | Literal["nominal_cruise"] = (
        "abort_return" if aborted else state
    )

    if next_state == "lane_change_out" and current_lane_id is not None and current_lane_id == target_lane_id:
        next_state = "pass_vehicle"
        planner_state = "pass_vehicle"
    elif next_state == "lane_change_back":
        planner_state = "abort_return" if aborted else "lane_change_back"

    phase_target_speed_kmh: float | None = None
    if next_state in {"lane_change_out", "pass_vehicle", "lane_change_back"}:
        phase_target_speed_kmh = overtake_target_speed_kmh
    elif next_state == "abort_return":
        phase_target_speed_kmh = min(
            overtake_target_speed_kmh,
            max(follow_target_speed_kmh, lead_speed_kmh),
        )

    should_prepare_abort_return = False
    event_overtake_abort = False
    if next_state in {"lane_change_out", "pass_vehicle"} and should_stop_for_light:
        next_state = "abort_return"
        planner_state = "abort_return"
        aborted = True
        should_prepare_abort_return = True
        event_overtake_abort = True

    completed = (
        current_lane_id is not None
        and next_state in {"lane_change_back", "abort_return"}
        and (
            (origin_lane_id is not None and current_lane_id == origin_lane_id)
            or (route_target_lane_id is not None and current_lane_id == route_target_lane_id)
        )
        and (lane_center_offset_m is None or lane_center_offset_m <= 0.75)
    )
    if completed:
        return OvertakeRuntimeTransition(
            state="idle",
            planner_state="nominal_cruise",
            aborted=False,
            phase_target_speed_kmh=phase_target_speed_kmh,
            completed=True,
            event_overtake_abort=event_overtake_abort,
            event_overtake_success=not aborted,
        )

    return OvertakeRuntimeTransition(
        state=next_state,
        planner_state=planner_state,
        aborted=aborted,
        phase_target_speed_kmh=phase_target_speed_kmh,
        should_prepare_abort_return=should_prepare_abort_return,
        event_overtake_abort=event_overtake_abort,
    )
