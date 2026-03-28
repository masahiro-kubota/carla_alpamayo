from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from pathlib import Path


@dataclass(slots=True)
class EpisodeRecord:
    episode_id: str
    frame_id: int
    town_id: str
    route_id: str
    weather_id: str
    timestamp: float
    front_rgb_path: str | None
    speed: float
    command: str
    steer: float
    throttle: float
    brake: float
    collision: bool
    lane_invasion: bool
    success: bool
    vehicle_x: float | None = None
    vehicle_y: float | None = None
    vehicle_z: float | None = None
    vehicle_yaw_deg: float | None = None
    route_completion_ratio: float | None = None
    distance_to_goal_m: float | None = None
    expert_steer: float | None = None
    route_target_x: float | None = None
    route_target_y: float | None = None
    planner_state: str | None = None
    traffic_light_state: str | None = None
    traffic_light_actor_id: int | None = None
    traffic_light_distance_m: float | None = None
    traffic_light_stop_line_distance_m: float | None = None
    traffic_light_violation: bool | None = None
    lead_vehicle_distance_m: float | None = None
    lead_vehicle_id: int | None = None
    lead_vehicle_speed_mps: float | None = None
    lead_vehicle_relative_speed_mps: float | None = None
    lead_vehicle_lane_id: str | None = None
    left_lane_front_gap_m: float | None = None
    left_lane_rear_gap_m: float | None = None
    right_lane_front_gap_m: float | None = None
    right_lane_rear_gap_m: float | None = None
    overtake_state: str | None = None
    overtake_considered: bool | None = None
    overtake_direction: str | None = None
    overtake_reject_reason: str | None = None
    overtake_target_actor_id: int | None = None
    overtake_target_kind: str | None = None
    overtake_target_member_actor_ids: list[int] | None = None
    overtake_target_lane_id: str | None = None
    target_passed: bool | None = None
    distance_past_target_m: float | None = None
    target_actor_visible: bool | None = None
    target_actor_last_seen_s: float | None = None
    current_lane_id: str | None = None
    route_target_lane_id: str | None = None
    target_lane_id: str | None = None
    target_speed_kmh: float | None = None
    emergency_stop: bool | None = None
    min_ttc: float | None = None
    mcap_segment_index: int | None = None
    mcap_segment_path: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False)


def append_jsonl(path: Path, record: EpisodeRecord) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(record.to_json())
        handle.write("\n")
