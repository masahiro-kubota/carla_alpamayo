from __future__ import annotations

import json
from dataclasses import dataclass, field
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
    mcap_segment_index: int | None = None
    mcap_segment_path: str | None = None
    extra_fields: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        payload = {
            "episode_id": self.episode_id,
            "frame_id": self.frame_id,
            "town_id": self.town_id,
            "route_id": self.route_id,
            "weather_id": self.weather_id,
            "timestamp": self.timestamp,
            "front_rgb_path": self.front_rgb_path,
            "speed": self.speed,
            "command": self.command,
            "steer": self.steer,
            "throttle": self.throttle,
            "brake": self.brake,
            "collision": self.collision,
            "lane_invasion": self.lane_invasion,
            "success": self.success,
            "vehicle_x": self.vehicle_x,
            "vehicle_y": self.vehicle_y,
            "vehicle_z": self.vehicle_z,
            "vehicle_yaw_deg": self.vehicle_yaw_deg,
            "route_completion_ratio": self.route_completion_ratio,
            "distance_to_goal_m": self.distance_to_goal_m,
            "expert_steer": self.expert_steer,
            "route_target_x": self.route_target_x,
            "route_target_y": self.route_target_y,
            "planner_state": self.planner_state,
            "mcap_segment_index": self.mcap_segment_index,
            "mcap_segment_path": self.mcap_segment_path,
        }
        payload.update(self.extra_fields)
        return payload

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False)


def append_jsonl(path: Path, record: EpisodeRecord) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(record.to_json())
        handle.write("\n")
