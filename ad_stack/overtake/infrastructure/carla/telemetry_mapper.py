from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from ad_stack.overtake.domain import (
    OvertakeCoreTelemetry,
    OvertakeEventFlags,
    OvertakeLeadSnapshot,
    OvertakePlanningDebug,
    OvertakeTargetTelemetry,
    OVERTAKE_PLANNING_DEBUG_CORE_FIELDS,
    OVERTAKE_PLANNING_DEBUG_TARGET_FIELDS,
    project_telemetry_section,
)
from libs.schemas import EgoStateSample, EpisodeRecord


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
    min_follow_target_distance_m: float = float("inf")
    first_target_passed_s: float | None = None
    first_rejoin_started_s: float | None = None
    first_rejoin_front_gap_m: float | None = None
    first_rejoin_rear_gap_m: float | None = None
    _traffic_light_violation_active: bool = False

    def observe(self, *, planning_debug: OvertakePlanningDebug, elapsed_seconds: float) -> None:
        self.traffic_light_stop_count += int(planning_debug.core.event_flags.traffic_light_stop)
        self.traffic_light_resume_count += int(planning_debug.core.event_flags.traffic_light_resume)
        self.car_follow_event_count += int(planning_debug.core.event_flags.car_follow_start)
        self.overtake_attempt_count += int(planning_debug.core.event_flags.overtake_attempt)
        self.overtake_success_count += int(planning_debug.core.event_flags.overtake_success)
        self.overtake_abort_count += int(planning_debug.core.event_flags.overtake_abort)
        self.unsafe_lane_change_reject_count += int(
            planning_debug.core.event_flags.unsafe_lane_change_reject
        )

        if planning_debug.core.event_flags.traffic_light_violation and not self._traffic_light_violation_active:
            self.traffic_light_violation_count += 1
            self._traffic_light_violation_active = True
        elif not planning_debug.core.event_flags.traffic_light_violation:
            self._traffic_light_violation_active = False

        if planning_debug.core.min_ttc is not None:
            self.min_ttc = min(self.min_ttc, float(planning_debug.core.min_ttc))
        if planning_debug.target.follow_target_distance_m is not None:
            self.min_follow_target_distance_m = min(
                self.min_follow_target_distance_m,
                float(planning_debug.target.follow_target_distance_m),
            )
        if self.first_target_passed_s is None and planning_debug.target.target_passed is True:
            self.first_target_passed_s = elapsed_seconds
        if (
            self.first_rejoin_started_s is None
            and planning_debug.target.overtake_state == "lane_change_back"
        ):
            self.first_rejoin_started_s = elapsed_seconds
            self.first_rejoin_front_gap_m = _as_optional_float(
                planning_debug.target.rejoin_front_gap_m
            )
            self.first_rejoin_rear_gap_m = _as_optional_float(
                planning_debug.target.rejoin_rear_gap_m
            )

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
            "min_follow_target_distance_m": _rounded_or_none(self.min_follow_target_distance_m),
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


@dataclass(frozen=True, slots=True)
class RouteLoopFrameTelemetryRequest:
    episode_id: str
    frame_id: int
    town_id: str
    route_id: str
    weather_id: str
    timestamp_s: float
    elapsed_seconds: float
    speed_mps: float
    behavior: str | None
    route_completion_ratio: float
    distance_to_goal_m: float
    pose: dict[str, float]
    control: dict[str, float]
    collision: bool
    lane_invasion: bool
    success: bool
    expert_steer: float | None
    route_target_x: float | None
    route_target_y: float | None
    planning_debug: OvertakePlanningDebug
    mcap_segment_index: int | None
    mcap_segment_path: str | None


@dataclass(frozen=True, slots=True)
class RouteLoopFrameTelemetry:
    ego_state: EgoStateSample
    episode_record: EpisodeRecord


def _build_episode_record_extra_fields(
    planning_debug: OvertakePlanningDebug,
) -> dict[str, Any]:
    extra_fields = project_telemetry_section(
        planning_debug.core,
        OVERTAKE_PLANNING_DEBUG_CORE_FIELDS,
        include_for="manifest",
    )
    extra_fields.update(
        project_telemetry_section(
            planning_debug.target,
            OVERTAKE_PLANNING_DEBUG_TARGET_FIELDS,
            include_for="manifest",
        )
    )
    return extra_fields


def _build_planning_debug_mcap_payload(planning_debug: OvertakePlanningDebug) -> dict[str, Any]:
    return {
        "core": project_telemetry_section(
            planning_debug.core,
            OVERTAKE_PLANNING_DEBUG_CORE_FIELDS,
            include_for="mcap",
        ),
        "target": project_telemetry_section(
            planning_debug.target,
            OVERTAKE_PLANNING_DEBUG_TARGET_FIELDS,
            include_for="mcap",
        ),
    }


def build_overtake_planning_debug(
    *,
    behavior_state: str,
    route_command: str,
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
    desired_speed_mps: float,
    applied_speed_mps: float,
    lookahead_distance_m: float,
    lateral_error_m: float | None,
    heading_error_deg: float | None,
    controller_steer_raw: float,
    controller_steer_applied: float,
    follow_lead: OvertakeLeadSnapshot | None,
    active_target: Any | None,
    follow_distance_m: float | None,
    follow_speed_mps: float | None,
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
    event_flags: dict[str, Any],
) -> OvertakePlanningDebug:
    core_event_flags = OvertakeEventFlags(
        traffic_light_stop=bool(event_flags.get("event_traffic_light_stop")),
        traffic_light_resume=bool(event_flags.get("event_traffic_light_resume")),
        car_follow_start=bool(event_flags.get("event_car_follow_start")),
        overtake_attempt=bool(event_flags.get("event_overtake_attempt")),
        overtake_success=bool(event_flags.get("event_overtake_success")),
        overtake_abort=bool(event_flags.get("event_overtake_abort")),
        unsafe_lane_change_reject=bool(event_flags.get("event_unsafe_lane_change_reject")),
        traffic_light_violation=bool(event_flags.get("traffic_light_violation")),
    )
    return OvertakePlanningDebug(
        core=OvertakeCoreTelemetry(
            behavior_state=behavior_state,
            route_command=route_command,
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
            desired_speed_mps=desired_speed_mps,
            applied_speed_mps=applied_speed_mps,
            lookahead_distance_m=lookahead_distance_m,
            lateral_error_m=lateral_error_m,
            heading_error_deg=heading_error_deg,
            controller_steer_raw=controller_steer_raw,
            controller_steer_applied=controller_steer_applied,
            target_lane_id=target_lane_id,
            min_ttc=_finite_or_none(min_ttc),
            event_flags=core_event_flags,
        ),
        target=OvertakeTargetTelemetry(
            follow_target_id=(
                follow_lead.actor_id
                if follow_lead is not None
                else active_target.primary_actor_id
                if active_target is not None
                else None
            ),
            follow_target_distance_m=follow_distance_m,
            follow_target_speed_mps=follow_speed_mps if follow_lead is not None else None,
            follow_target_relative_speed_mps=closing_speed_mps if follow_lead is not None else None,
            follow_target_lane_id=(
                follow_lead.lane_id
                if follow_lead is not None
                else active_target.lane_id
                if active_target is not None
                else None
            ),
            follow_target_motion_profile=(
                follow_lead.motion_profile
                if follow_lead is not None
                else active_target.motion_profile
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
            overtake_target_motion_profile=(
                active_target.motion_profile if active_target is not None else None
            ),
            target_passed=target_passed,
            distance_past_target_m=distance_past_target_m,
            target_actor_visible=target_actor_visible,
            target_actor_last_seen_s=target_actor_last_seen_s,
            lane_change_path_available=lane_change_path_available,
            lane_change_path_failed_reason=lane_change_path_failed_reason,
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
        planner_state=planning_debug.core.behavior_state,
        traffic_light_state=planning_debug.core.traffic_light_state,
        follow_target_distance_m=planning_debug.target.follow_target_distance_m,
        overtake_state=planning_debug.target.overtake_state,
        target_lane_id=planning_debug.core.target_lane_id,
        min_ttc=planning_debug.core.min_ttc,
        pose=pose,
        control=control,
        planning_debug=_build_planning_debug_mcap_payload(planning_debug),
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
        command=planning_debug.core.route_command,
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
        mcap_segment_index=mcap_segment_index,
        mcap_segment_path=mcap_segment_path,
        extra_fields=_build_episode_record_extra_fields(planning_debug),
    )


def build_frame_telemetry(
    request: RouteLoopFrameTelemetryRequest,
) -> RouteLoopFrameTelemetry:
    ego_state = build_ego_state_sample(
        episode_id=request.episode_id,
        frame_id=request.frame_id,
        timestamp_s=request.timestamp_s,
        elapsed_seconds=request.elapsed_seconds,
        speed_mps=request.speed_mps,
        behavior=request.behavior,
        route_completion_ratio=request.route_completion_ratio,
        distance_to_goal_m=request.distance_to_goal_m,
        pose=request.pose,
        control=request.control,
        planning_debug=request.planning_debug,
    )
    episode_record = build_episode_record(
        episode_id=request.episode_id,
        frame_id=request.frame_id,
        town_id=request.town_id,
        route_id=request.route_id,
        weather_id=request.weather_id,
        timestamp=request.timestamp_s,
        speed=request.speed_mps,
        steer=request.control["steer"],
        throttle=request.control["throttle"],
        brake=request.control["brake"],
        collision=request.collision,
        lane_invasion=request.lane_invasion,
        success=request.success,
        vehicle_x=request.pose["x"],
        vehicle_y=request.pose["y"],
        vehicle_z=request.pose["z"],
        vehicle_yaw_deg=request.pose["yaw_deg"],
        route_completion_ratio=request.route_completion_ratio,
        distance_to_goal_m=request.distance_to_goal_m,
        expert_steer=request.expert_steer,
        route_target_x=request.route_target_x,
        route_target_y=request.route_target_y,
        planner_state=request.planner_state,
        planning_debug=request.planning_debug,
        mcap_segment_index=request.mcap_segment_index,
        mcap_segment_path=request.mcap_segment_path,
    )
    return RouteLoopFrameTelemetry(
        ego_state=ego_state,
        episode_record=episode_record,
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
