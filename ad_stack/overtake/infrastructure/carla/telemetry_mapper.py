from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from ad_stack.overtake.domain import OvertakeEventFlags, OvertakePlanningDebug
from libs.schemas import EgoStateSample, EpisodeRecord

_PLANNING_DEBUG_MCAP_OMIT_KEYS = frozenset(
    {
        "planner_state",
        "traffic_light_state",
        "overtake_state",
        "target_lane_id",
        "min_ttc",
    }
)


@dataclass(slots=True)
class RouteLoopTelemetryAccumulator:
    traffic_light_stop_count: int = 0
    traffic_light_resume_count: int = 0
    traffic_light_violation_count: int = 0
    car_follow_event_count: int = 0
    overtake_attempt_count: int = 0
    overtake_success_count: int = 0
    overtake_abort_count: int = 0
    unsafe_lane_change_reject_count: int = 0
    min_ttc: float = float("inf")
    min_lead_distance_m: float = float("inf")
    first_target_passed_s: float | None = None
    first_rejoin_started_s: float | None = None
    first_rejoin_front_gap_m: float | None = None
    first_rejoin_rear_gap_m: float | None = None
    _traffic_light_violation_active: bool = False

    def observe(self, *, planning_debug: OvertakePlanningDebug, elapsed_seconds: float) -> None:
        self.traffic_light_stop_count += int(planning_debug.event_flags.traffic_light_stop)
        self.traffic_light_resume_count += int(planning_debug.event_flags.traffic_light_resume)
        self.car_follow_event_count += int(planning_debug.event_flags.car_follow_start)
        self.overtake_attempt_count += int(planning_debug.event_flags.overtake_attempt)
        self.overtake_success_count += int(planning_debug.event_flags.overtake_success)
        self.overtake_abort_count += int(planning_debug.event_flags.overtake_abort)
        self.unsafe_lane_change_reject_count += int(
            planning_debug.event_flags.unsafe_lane_change_reject
        )

        if planning_debug.event_flags.traffic_light_violation and not self._traffic_light_violation_active:
            self.traffic_light_violation_count += 1
            self._traffic_light_violation_active = True
        elif not planning_debug.event_flags.traffic_light_violation:
            self._traffic_light_violation_active = False

        if planning_debug.min_ttc is not None:
            self.min_ttc = min(self.min_ttc, float(planning_debug.min_ttc))
        if planning_debug.lead_vehicle_distance_m is not None:
            self.min_lead_distance_m = min(
                self.min_lead_distance_m, float(planning_debug.lead_vehicle_distance_m)
            )
        if self.first_target_passed_s is None and planning_debug.target_passed is True:
            self.first_target_passed_s = elapsed_seconds
        if (
            self.first_rejoin_started_s is None
            and planning_debug.overtake_state == "lane_change_back"
        ):
            self.first_rejoin_started_s = elapsed_seconds
            self.first_rejoin_front_gap_m = _as_optional_float(planning_debug.rejoin_front_gap_m)
            self.first_rejoin_rear_gap_m = _as_optional_float(planning_debug.rejoin_rear_gap_m)

    def summary_fields(self) -> dict[str, Any]:
        return {
            "traffic_light_stop_count": self.traffic_light_stop_count,
            "traffic_light_resume_count": self.traffic_light_resume_count,
            "traffic_light_violation_count": self.traffic_light_violation_count,
            "car_follow_event_count": self.car_follow_event_count,
            "overtake_attempt_count": self.overtake_attempt_count,
            "overtake_success_count": self.overtake_success_count,
            "overtake_abort_count": self.overtake_abort_count,
            "unsafe_lane_change_reject_count": self.unsafe_lane_change_reject_count,
            "min_ttc": _rounded_or_none(self.min_ttc),
            "min_lead_distance_m": _rounded_or_none(self.min_lead_distance_m),
            "first_target_passed_s": _rounded_or_none(self.first_target_passed_s),
            "first_rejoin_started_s": _rounded_or_none(self.first_rejoin_started_s),
            "rejoin_wait_after_target_passed_s": (
                None
                if self.first_target_passed_s is None or self.first_rejoin_started_s is None
                else round(self.first_rejoin_started_s - self.first_target_passed_s, 3)
            ),
            "first_rejoin_front_gap_m": _rounded_or_none(self.first_rejoin_front_gap_m),
            "first_rejoin_rear_gap_m": _rounded_or_none(self.first_rejoin_rear_gap_m),
        }


def planning_debug_to_dict(planning_debug: OvertakePlanningDebug) -> dict[str, Any]:
    return {
        "remaining_waypoints": planning_debug.remaining_waypoints,
        "route_progress_index": planning_debug.route_progress_index,
        "max_route_index": planning_debug.max_route_index,
        "current_lane_id": planning_debug.current_lane_id,
        "lane_center_offset_m": planning_debug.lane_center_offset_m,
        "route_target_lane_id": planning_debug.route_target_lane_id,
        "left_lane_open": planning_debug.left_lane_open,
        "right_lane_open": planning_debug.right_lane_open,
        "traffic_light_actor_id": planning_debug.traffic_light_actor_id,
        "traffic_light_state": planning_debug.traffic_light_state,
        "traffic_light_distance_m": planning_debug.traffic_light_distance_m,
        "traffic_light_stop_line_distance_m": planning_debug.traffic_light_stop_line_distance_m,
        "traffic_light_red_latched": planning_debug.traffic_light_red_latched,
        "traffic_light_stop_buffer_m": planning_debug.traffic_light_stop_buffer_m,
        "traffic_light_stop_target_distance_m": planning_debug.traffic_light_stop_target_distance_m,
        "target_speed_kmh": planning_debug.target_speed_kmh,
        "lead_vehicle_id": planning_debug.lead_vehicle_id,
        "lead_vehicle_distance_m": planning_debug.lead_vehicle_distance_m,
        "lead_vehicle_speed_mps": planning_debug.lead_vehicle_speed_mps,
        "lead_vehicle_relative_speed_mps": planning_debug.lead_vehicle_relative_speed_mps,
        "lead_vehicle_lane_id": planning_debug.lead_vehicle_lane_id,
        "left_lane_front_gap_m": planning_debug.left_lane_front_gap_m,
        "left_lane_rear_gap_m": planning_debug.left_lane_rear_gap_m,
        "right_lane_front_gap_m": planning_debug.right_lane_front_gap_m,
        "right_lane_rear_gap_m": planning_debug.right_lane_rear_gap_m,
        "rejoin_front_gap_m": planning_debug.rejoin_front_gap_m,
        "rejoin_rear_gap_m": planning_debug.rejoin_rear_gap_m,
        "overtake_considered": planning_debug.overtake_considered,
        "overtake_reject_reason": planning_debug.overtake_reject_reason,
        "overtake_state": planning_debug.overtake_state,
        "overtake_direction": planning_debug.overtake_direction,
        "overtake_origin_lane_id": planning_debug.overtake_origin_lane_id,
        "overtake_target_actor_id": planning_debug.overtake_target_actor_id,
        "overtake_target_kind": planning_debug.overtake_target_kind,
        "overtake_target_member_actor_ids": list(planning_debug.overtake_target_member_actor_ids),
        "overtake_target_lane_id": planning_debug.overtake_target_lane_id,
        "target_passed": planning_debug.target_passed,
        "distance_past_target_m": planning_debug.distance_past_target_m,
        "target_actor_visible": planning_debug.target_actor_visible,
        "target_actor_last_seen_s": planning_debug.target_actor_last_seen_s,
        "lane_change_path_available": planning_debug.lane_change_path_available,
        "lane_change_path_failed_reason": planning_debug.lane_change_path_failed_reason,
        "target_lane_id": planning_debug.target_lane_id,
        "min_ttc": planning_debug.min_ttc,
        "emergency_stop": planning_debug.emergency_stop,
        "event_traffic_light_stop": planning_debug.event_flags.traffic_light_stop,
        "event_traffic_light_resume": planning_debug.event_flags.traffic_light_resume,
        "event_car_follow_start": planning_debug.event_flags.car_follow_start,
        "event_overtake_attempt": planning_debug.event_flags.overtake_attempt,
        "event_overtake_success": planning_debug.event_flags.overtake_success,
        "event_overtake_abort": planning_debug.event_flags.overtake_abort,
        "event_unsafe_lane_change_reject": planning_debug.event_flags.unsafe_lane_change_reject,
        "traffic_light_violation": planning_debug.event_flags.traffic_light_violation,
    }


def build_planning_debug_mcap_payload(planning_debug: OvertakePlanningDebug) -> dict[str, Any]:
    return {
        key: value
        for key, value in planning_debug_to_dict(planning_debug).items()
        if key not in _PLANNING_DEBUG_MCAP_OMIT_KEYS
    }


def build_overtake_planning_debug(
    *,
    remaining_waypoints: int,
    route_index: int | None,
    max_route_index: int,
    current_lane_id: str | None,
    lane_center_offset_m: float | None,
    route_target_lane_id: str | None,
    left_lane_open: bool,
    right_lane_open: bool,
    active_light: Any | None,
    red_light_latched: bool,
    traffic_light_stop_buffer_m: float,
    traffic_light_stop_target_distance_m: float | None,
    target_speed_kmh: float,
    lead_vehicle: Any | None,
    active_target: Any | None,
    lead_distance_m: float | None,
    lead_speed_mps: float | None,
    closing_speed_mps: float | None,
    left_lane_front_gap_m: float,
    left_lane_rear_gap_m: float,
    right_lane_front_gap_m: float,
    right_lane_rear_gap_m: float,
    rejoin_front_gap_m: float,
    rejoin_rear_gap_m: float,
    overtake_considered: bool,
    overtake_reject_reason: str | None,
    overtake_state: str,
    overtake_direction: str | None,
    overtake_origin_lane_id: str | None,
    overtake_target_actor_id: int | None,
    overtake_target_kind: str,
    overtake_target_member_actor_ids: tuple[int, ...],
    overtake_target_lane_id: str | None,
    target_passed: bool,
    distance_past_target_m: float | None,
    target_actor_visible: bool,
    target_actor_last_seen_s: float | None,
    lane_change_path_available: bool,
    lane_change_path_failed_reason: str | None,
    target_lane_id: str | None,
    min_ttc: float,
    emergency_stop: bool,
    event_flags: dict[str, Any],
) -> OvertakePlanningDebug:
    return OvertakePlanningDebug(
        remaining_waypoints=remaining_waypoints,
        route_progress_index=route_index,
        max_route_index=max_route_index,
        current_lane_id=current_lane_id,
        lane_center_offset_m=lane_center_offset_m,
        route_target_lane_id=route_target_lane_id,
        left_lane_open=left_lane_open,
        right_lane_open=right_lane_open,
        traffic_light_actor_id=active_light.actor_id if active_light is not None else None,
        traffic_light_state=active_light.state if active_light is not None else None,
        traffic_light_distance_m=active_light.distance_m if active_light is not None else None,
        traffic_light_stop_line_distance_m=(
            active_light.stop_line_distance_m if active_light is not None else None
        ),
        traffic_light_red_latched=red_light_latched,
        traffic_light_stop_buffer_m=traffic_light_stop_buffer_m,
        traffic_light_stop_target_distance_m=traffic_light_stop_target_distance_m,
        target_speed_kmh=target_speed_kmh,
        lead_vehicle_id=(
            lead_vehicle.actor_id
            if lead_vehicle is not None
            else active_target.primary_actor_id
            if active_target is not None
            else None
        ),
        lead_vehicle_distance_m=lead_distance_m,
        lead_vehicle_speed_mps=lead_speed_mps if lead_vehicle is not None else None,
        lead_vehicle_relative_speed_mps=closing_speed_mps if lead_vehicle is not None else None,
        lead_vehicle_lane_id=(
            lead_vehicle.lane_id
            if lead_vehicle is not None
            else active_target.lane_id
            if active_target is not None
            else None
        ),
        left_lane_front_gap_m=_finite_or_none(left_lane_front_gap_m),
        left_lane_rear_gap_m=_finite_or_none(left_lane_rear_gap_m),
        right_lane_front_gap_m=_finite_or_none(right_lane_front_gap_m),
        right_lane_rear_gap_m=_finite_or_none(right_lane_rear_gap_m),
        rejoin_front_gap_m=_finite_or_none(rejoin_front_gap_m),
        rejoin_rear_gap_m=_finite_or_none(rejoin_rear_gap_m),
        overtake_considered=overtake_considered,
        overtake_reject_reason=overtake_reject_reason,
        overtake_state=overtake_state,
        overtake_direction=overtake_direction,
        overtake_origin_lane_id=overtake_origin_lane_id,
        overtake_target_actor_id=overtake_target_actor_id,
        overtake_target_kind=overtake_target_kind,
        overtake_target_member_actor_ids=overtake_target_member_actor_ids,
        overtake_target_lane_id=overtake_target_lane_id,
        target_passed=target_passed,
        distance_past_target_m=distance_past_target_m,
        target_actor_visible=target_actor_visible,
        target_actor_last_seen_s=target_actor_last_seen_s,
        lane_change_path_available=lane_change_path_available,
        lane_change_path_failed_reason=lane_change_path_failed_reason,
        target_lane_id=target_lane_id,
        min_ttc=_finite_or_none(min_ttc),
        emergency_stop=emergency_stop,
        event_flags=OvertakeEventFlags(
            traffic_light_stop=bool(event_flags.get("event_traffic_light_stop")),
            traffic_light_resume=bool(event_flags.get("event_traffic_light_resume")),
            car_follow_start=bool(event_flags.get("event_car_follow_start")),
            overtake_attempt=bool(event_flags.get("event_overtake_attempt")),
            overtake_success=bool(event_flags.get("event_overtake_success")),
            overtake_abort=bool(event_flags.get("event_overtake_abort")),
            unsafe_lane_change_reject=bool(event_flags.get("event_unsafe_lane_change_reject")),
            traffic_light_violation=bool(event_flags.get("traffic_light_violation")),
        ),
    )


def build_ego_state_sample(
    *,
    episode_id: str,
    frame_id: int,
    timestamp_s: float,
    elapsed_seconds: float,
    speed_mps: float,
    behavior: str | None,
    route_completion_ratio: float,
    distance_to_goal_m: float,
    planner_state: str | None,
    traffic_light_state: str | None,
    lead_vehicle_distance_m: float | None,
    overtake_state: str | None,
    target_lane_id: str | None,
    min_ttc: float | None,
    pose: dict[str, float],
    control: dict[str, float],
    planning_debug: OvertakePlanningDebug,
) -> EgoStateSample:
    return EgoStateSample(
        episode_id=episode_id,
        frame_id=frame_id,
        timestamp_s=timestamp_s,
        elapsed_seconds=elapsed_seconds,
        speed_mps=speed_mps,
        behavior=behavior,
        route_completion_ratio=route_completion_ratio,
        distance_to_goal_m=distance_to_goal_m,
        planner_state=planner_state,
        traffic_light_state=traffic_light_state,
        lead_vehicle_distance_m=lead_vehicle_distance_m,
        overtake_state=overtake_state,
        target_lane_id=target_lane_id,
        min_ttc=min_ttc,
        pose=pose,
        control=control,
        planning_debug=build_planning_debug_mcap_payload(planning_debug),
    )


def build_episode_record(
    *,
    episode_id: str,
    frame_id: int,
    town_id: str,
    route_id: str,
    weather_id: str,
    timestamp: float,
    speed: float,
    command: str,
    steer: float,
    throttle: float,
    brake: float,
    collision: bool,
    lane_invasion: bool,
    success: bool,
    vehicle_x: float,
    vehicle_y: float,
    vehicle_z: float,
    vehicle_yaw_deg: float,
    route_completion_ratio: float,
    distance_to_goal_m: float,
    expert_steer: float | None,
    route_target_x: float | None,
    route_target_y: float | None,
    planner_state: str | None,
    planning_debug: OvertakePlanningDebug,
    mcap_segment_index: int | None,
    mcap_segment_path: str | None,
) -> EpisodeRecord:
    return EpisodeRecord(
        episode_id=episode_id,
        frame_id=frame_id,
        town_id=town_id,
        route_id=route_id,
        weather_id=weather_id,
        timestamp=timestamp,
        front_rgb_path=None,
        speed=speed,
        command=command,
        steer=steer,
        throttle=throttle,
        brake=brake,
        collision=collision,
        lane_invasion=lane_invasion,
        success=success,
        vehicle_x=vehicle_x,
        vehicle_y=vehicle_y,
        vehicle_z=vehicle_z,
        vehicle_yaw_deg=vehicle_yaw_deg,
        route_completion_ratio=route_completion_ratio,
        distance_to_goal_m=distance_to_goal_m,
        expert_steer=expert_steer,
        route_target_x=route_target_x,
        route_target_y=route_target_y,
        planner_state=planner_state,
        traffic_light_state=planning_debug.traffic_light_state,
        traffic_light_actor_id=planning_debug.traffic_light_actor_id,
        traffic_light_distance_m=planning_debug.traffic_light_distance_m,
        traffic_light_stop_line_distance_m=planning_debug.traffic_light_stop_line_distance_m,
        traffic_light_violation=planning_debug.event_flags.traffic_light_violation,
        lead_vehicle_distance_m=planning_debug.lead_vehicle_distance_m,
        lead_vehicle_id=planning_debug.lead_vehicle_id,
        lead_vehicle_speed_mps=planning_debug.lead_vehicle_speed_mps,
        lead_vehicle_relative_speed_mps=planning_debug.lead_vehicle_relative_speed_mps,
        lead_vehicle_lane_id=planning_debug.lead_vehicle_lane_id,
        left_lane_front_gap_m=planning_debug.left_lane_front_gap_m,
        left_lane_rear_gap_m=planning_debug.left_lane_rear_gap_m,
        right_lane_front_gap_m=planning_debug.right_lane_front_gap_m,
        right_lane_rear_gap_m=planning_debug.right_lane_rear_gap_m,
        rejoin_front_gap_m=planning_debug.rejoin_front_gap_m,
        rejoin_rear_gap_m=planning_debug.rejoin_rear_gap_m,
        overtake_state=planning_debug.overtake_state,
        overtake_considered=planning_debug.overtake_considered,
        overtake_direction=planning_debug.overtake_direction,
        overtake_reject_reason=planning_debug.overtake_reject_reason,
        overtake_target_actor_id=planning_debug.overtake_target_actor_id,
        overtake_target_kind=planning_debug.overtake_target_kind,
        overtake_target_member_actor_ids=list(planning_debug.overtake_target_member_actor_ids),
        overtake_target_lane_id=planning_debug.overtake_target_lane_id,
        target_passed=planning_debug.target_passed,
        distance_past_target_m=planning_debug.distance_past_target_m,
        target_actor_visible=planning_debug.target_actor_visible,
        target_actor_last_seen_s=planning_debug.target_actor_last_seen_s,
        current_lane_id=planning_debug.current_lane_id,
        route_target_lane_id=planning_debug.route_target_lane_id,
        target_lane_id=planning_debug.target_lane_id,
        target_speed_kmh=planning_debug.target_speed_kmh,
        emergency_stop=planning_debug.emergency_stop,
        min_ttc=planning_debug.min_ttc,
        mcap_segment_index=mcap_segment_index,
        mcap_segment_path=mcap_segment_path,
    )


def _rounded_or_none(value: float | None) -> float | None:
    if value is None or not value < float("inf"):
        return None
    return round(value, 3)


def _as_optional_float(value: Any) -> float | None:
    if value is None:
        return None
    return float(value)


def _finite_or_none(value: float | None) -> float | None:
    if value is None:
        return None
    return float(value) if math.isfinite(float(value)) else None
