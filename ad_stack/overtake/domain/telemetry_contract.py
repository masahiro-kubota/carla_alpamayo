from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True, slots=True)
class TelemetryFieldSpec:
    name: str
    attr_path: tuple[str, ...]
    json_types: tuple[str, ...]
    include_in_mcap: bool = True
    include_in_manifest: bool = True
    sequence_as_list: bool = False


@dataclass(frozen=True, slots=True)
class TelemetrySectionSpec:
    name: str
    fields: tuple[TelemetryFieldSpec, ...]


OVERTAKE_PLANNING_DEBUG_CORE_FIELDS: tuple[TelemetryFieldSpec, ...] = (
    TelemetryFieldSpec("remaining_waypoints", ("remaining_waypoints",), ("integer",)),
    TelemetryFieldSpec("route_progress_index", ("route_progress_index",), ("integer", "null")),
    TelemetryFieldSpec("max_route_index", ("max_route_index",), ("integer",)),
    TelemetryFieldSpec("current_lane_id", ("current_lane_id",), ("string", "null")),
    TelemetryFieldSpec("lane_center_offset_m", ("lane_center_offset_m",), ("number", "null")),
    TelemetryFieldSpec("route_target_lane_id", ("route_target_lane_id",), ("string", "null")),
    TelemetryFieldSpec("left_lane_open", ("left_lane_open",), ("boolean",)),
    TelemetryFieldSpec("right_lane_open", ("right_lane_open",), ("boolean",)),
    TelemetryFieldSpec("traffic_light_actor_id", ("traffic_light_actor_id",), ("integer", "null")),
    TelemetryFieldSpec(
        "traffic_light_state",
        ("traffic_light_state",),
        ("string", "null"),
        include_in_mcap=False,
    ),
    TelemetryFieldSpec("traffic_light_distance_m", ("traffic_light_distance_m",), ("number", "null")),
    TelemetryFieldSpec(
        "traffic_light_stop_line_distance_m",
        ("traffic_light_stop_line_distance_m",),
        ("number", "null"),
    ),
    TelemetryFieldSpec("traffic_light_red_latched", ("traffic_light_red_latched",), ("boolean",)),
    TelemetryFieldSpec("traffic_light_stop_buffer_m", ("traffic_light_stop_buffer_m",), ("number",)),
    TelemetryFieldSpec(
        "traffic_light_stop_target_distance_m",
        ("traffic_light_stop_target_distance_m",),
        ("number", "null"),
    ),
    TelemetryFieldSpec("target_speed_kmh", ("target_speed_kmh",), ("number",)),
    TelemetryFieldSpec(
        "target_lane_id",
        ("target_lane_id",),
        ("string", "null"),
        include_in_mcap=False,
    ),
    TelemetryFieldSpec("min_ttc", ("min_ttc",), ("number", "null"), include_in_mcap=False),
    TelemetryFieldSpec("event_traffic_light_stop", ("event_flags", "traffic_light_stop"), ("boolean",)),
    TelemetryFieldSpec(
        "event_traffic_light_resume",
        ("event_flags", "traffic_light_resume"),
        ("boolean",),
    ),
    TelemetryFieldSpec("event_car_follow_start", ("event_flags", "car_follow_start"), ("boolean",)),
    TelemetryFieldSpec("event_overtake_attempt", ("event_flags", "overtake_attempt"), ("boolean",)),
    TelemetryFieldSpec("event_overtake_success", ("event_flags", "overtake_success"), ("boolean",)),
    TelemetryFieldSpec("event_overtake_abort", ("event_flags", "overtake_abort"), ("boolean",)),
    TelemetryFieldSpec(
        "event_unsafe_lane_change_reject",
        ("event_flags", "unsafe_lane_change_reject"),
        ("boolean",),
    ),
    TelemetryFieldSpec(
        "traffic_light_violation",
        ("event_flags", "traffic_light_violation"),
        ("boolean",),
    ),
)

OVERTAKE_PLANNING_DEBUG_TARGET_FIELDS: tuple[TelemetryFieldSpec, ...] = (
    TelemetryFieldSpec("follow_target_id", ("follow_target_id",), ("integer", "null")),
    TelemetryFieldSpec("follow_target_distance_m", ("follow_target_distance_m",), ("number", "null")),
    TelemetryFieldSpec("follow_target_speed_mps", ("follow_target_speed_mps",), ("number", "null")),
    TelemetryFieldSpec(
        "follow_target_relative_speed_mps",
        ("follow_target_relative_speed_mps",),
        ("number", "null"),
    ),
    TelemetryFieldSpec("follow_target_lane_id", ("follow_target_lane_id",), ("string", "null")),
    TelemetryFieldSpec(
        "follow_target_motion_profile",
        ("follow_target_motion_profile",),
        ("string", "null"),
    ),
    TelemetryFieldSpec("left_lane_front_gap_m", ("left_lane_front_gap_m",), ("number", "null")),
    TelemetryFieldSpec("left_lane_rear_gap_m", ("left_lane_rear_gap_m",), ("number", "null")),
    TelemetryFieldSpec("right_lane_front_gap_m", ("right_lane_front_gap_m",), ("number", "null")),
    TelemetryFieldSpec("right_lane_rear_gap_m", ("right_lane_rear_gap_m",), ("number", "null")),
    TelemetryFieldSpec("rejoin_front_gap_m", ("rejoin_front_gap_m",), ("number", "null")),
    TelemetryFieldSpec("rejoin_rear_gap_m", ("rejoin_rear_gap_m",), ("number", "null")),
    TelemetryFieldSpec("overtake_considered", ("overtake_considered",), ("boolean",)),
    TelemetryFieldSpec("overtake_reject_reason", ("overtake_reject_reason",), ("string", "null")),
    TelemetryFieldSpec(
        "overtake_state",
        ("overtake_state",),
        ("string",),
        include_in_mcap=False,
    ),
    TelemetryFieldSpec("overtake_direction", ("overtake_direction",), ("string", "null")),
    TelemetryFieldSpec("overtake_origin_lane_id", ("overtake_origin_lane_id",), ("string", "null")),
    TelemetryFieldSpec("overtake_target_actor_id", ("overtake_target_actor_id",), ("integer", "null")),
    TelemetryFieldSpec("overtake_target_kind", ("overtake_target_kind",), ("string",)),
    TelemetryFieldSpec(
        "overtake_target_member_actor_ids",
        ("overtake_target_member_actor_ids",),
        ("integer",),
        sequence_as_list=True,
    ),
    TelemetryFieldSpec("overtake_target_lane_id", ("overtake_target_lane_id",), ("string", "null")),
    TelemetryFieldSpec(
        "overtake_target_motion_profile",
        ("overtake_target_motion_profile",),
        ("string", "null"),
    ),
    TelemetryFieldSpec("target_passed", ("target_passed",), ("boolean",)),
    TelemetryFieldSpec("distance_past_target_m", ("distance_past_target_m",), ("number", "null")),
    TelemetryFieldSpec("target_actor_visible", ("target_actor_visible",), ("boolean",)),
    TelemetryFieldSpec("target_actor_last_seen_s", ("target_actor_last_seen_s",), ("number", "null")),
    TelemetryFieldSpec("lane_change_path_available", ("lane_change_path_available",), ("boolean",)),
    TelemetryFieldSpec(
        "lane_change_path_failed_reason",
        ("lane_change_path_failed_reason",),
        ("string", "null"),
    ),
)

OVERTAKE_PLANNING_DEBUG_SECTIONS: tuple[TelemetrySectionSpec, ...] = (
    TelemetrySectionSpec(name="core", fields=OVERTAKE_PLANNING_DEBUG_CORE_FIELDS),
    TelemetrySectionSpec(name="target", fields=OVERTAKE_PLANNING_DEBUG_TARGET_FIELDS),
)


def project_telemetry_section(
    source: Any,
    fields: tuple[TelemetryFieldSpec, ...],
    *,
    include_for: str,
) -> dict[str, Any]:
    if include_for not in {"mcap", "manifest", "all"}:
        raise ValueError(f"unsupported telemetry projection target: {include_for}")
    payload: dict[str, Any] = {}
    for field in fields:
        if include_for == "mcap" and not field.include_in_mcap:
            continue
        if include_for == "manifest" and not field.include_in_manifest:
            continue
        value = _resolve_attr_path(source, field.attr_path)
        if field.sequence_as_list and value is not None:
            value = list(value)
        payload[field.name] = value
    return payload


def build_planning_debug_message_schema(timestamp_schema: dict[str, Any]) -> dict[str, Any]:
    return {
        "type": "object",
        "properties": {
            "timestamp": timestamp_schema,
            "episode_id": {"type": "string"},
            "frame_id": {"type": "integer"},
            "elapsed_seconds": {"type": "number"},
            **{
                section.name: _build_section_schema(section.fields, include_for="mcap")
                for section in OVERTAKE_PLANNING_DEBUG_SECTIONS
            },
        },
        "required": ["timestamp", "episode_id", "frame_id", "elapsed_seconds", "core", "target"],
        "additionalProperties": False,
    }


def _build_section_schema(
    fields: tuple[TelemetryFieldSpec, ...],
    *,
    include_for: str,
) -> dict[str, Any]:
    properties: dict[str, Any] = {}
    required: list[str] = []
    for field in fields:
        if include_for == "mcap" and not field.include_in_mcap:
            continue
        field_schema = _field_schema(field)
        properties[field.name] = field_schema
        required.append(field.name)
    return {
        "type": "object",
        "properties": properties,
        "required": required,
        "additionalProperties": False,
    }


def _field_schema(field: TelemetryFieldSpec) -> dict[str, Any]:
    if field.sequence_as_list:
        return {
            "type": "array",
            "items": {"type": field.json_types[0]},
        }
    if len(field.json_types) == 1:
        return {"type": field.json_types[0]}
    return {"type": list(field.json_types)}


def _resolve_attr_path(source: Any, attr_path: tuple[str, ...]) -> Any:
    value = source
    for attr_name in attr_path:
        value = getattr(value, attr_name)
    return value
