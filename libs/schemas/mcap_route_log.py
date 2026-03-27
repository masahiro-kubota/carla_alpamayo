from __future__ import annotations

from dataclasses import asdict, dataclass
import io
import json
from pathlib import Path
from typing import Any

from mcap.writer import CompressionType, Writer
from PIL import Image


_EGO_STATE_JSON_SCHEMA = {
    "type": "object",
    "properties": {
        "episode_id": {"type": "string"},
        "frame_id": {"type": "integer"},
        "timestamp_s": {"type": "number"},
        "elapsed_seconds": {"type": "number"},
        "speed_mps": {"type": "number"},
        "behavior": {"type": ["string", "null"]},
        "route_completion_ratio": {"type": "number"},
        "distance_to_goal_m": {"type": "number"},
        "planner_state": {"type": ["string", "null"]},
        "traffic_light_state": {"type": ["string", "null"]},
        "lead_vehicle_distance_m": {"type": ["number", "null"]},
        "overtake_state": {"type": ["string", "null"]},
        "target_lane_id": {"type": ["string", "null"]},
        "min_ttc": {"type": ["number", "null"]},
        "pose": {
            "type": "object",
            "properties": {
                "x": {"type": "number"},
                "y": {"type": "number"},
                "z": {"type": "number"},
                "yaw_deg": {"type": "number"},
                "pitch_deg": {"type": "number"},
                "roll_deg": {"type": "number"},
            },
            "required": ["x", "y", "z", "yaw_deg", "pitch_deg", "roll_deg"],
        },
        "control": {
            "type": "object",
            "properties": {
                "steer": {"type": "number"},
                "throttle": {"type": "number"},
                "brake": {"type": "number"},
            },
            "required": ["steer", "throttle", "brake"],
        },
    },
    "required": [
        "episode_id",
        "frame_id",
        "timestamp_s",
        "elapsed_seconds",
        "speed_mps",
        "route_completion_ratio",
        "distance_to_goal_m",
        "pose",
        "control",
    ],
}


@dataclass(slots=True)
class EgoStateSample:
    episode_id: str
    frame_id: int
    timestamp_s: float
    elapsed_seconds: float
    speed_mps: float
    behavior: str | None
    route_completion_ratio: float
    distance_to_goal_m: float
    planner_state: str | None
    traffic_light_state: str | None
    lead_vehicle_distance_m: float | None
    overtake_state: str | None
    target_lane_id: str | None
    min_ttc: float | None
    pose: dict[str, float]
    control: dict[str, float]

    def to_bytes(self) -> bytes:
        return json.dumps(asdict(self), ensure_ascii=False, separators=(",", ":")).encode("utf-8")


class RouteLoopMcapWriter:
    def __init__(
        self,
        *,
        path: Path,
        episode_id: str,
        route_name: str,
        town: str,
        weather: str,
        camera_width: int,
        camera_height: int,
        jpeg_quality: int = 85,
    ) -> None:
        self.path = path
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = self.path.open("wb")
        self._writer = Writer(self._stream, compression=CompressionType.ZSTD)
        self._writer.start(profile="carla_alpamayo.route_loop", library="carla_alpamayo")
        ego_schema_id = self._writer.register_schema(
            name="carla_alpamayo.EgoStateSample",
            encoding="jsonschema",
            data=json.dumps(_EGO_STATE_JSON_SCHEMA, ensure_ascii=False).encode("utf-8"),
        )
        self._front_rgb_channel_id = self._writer.register_channel(
            topic="/sensors/front_rgb/compressed",
            message_encoding="jpeg",
            schema_id=0,
            metadata={
                "camera_width": str(camera_width),
                "camera_height": str(camera_height),
                "jpeg_quality": str(jpeg_quality),
            },
        )
        self._ego_state_channel_id = self._writer.register_channel(
            topic="/state/ego",
            message_encoding="jsonschema",
            schema_id=ego_schema_id,
        )
        self._writer.add_metadata(
            "episode",
            {
                "episode_id": episode_id,
                "route_name": route_name,
                "town": town,
                "weather": weather,
            },
        )
        self._jpeg_quality = jpeg_quality
        self._closed = False

    def write_frame(
        self,
        *,
        current_rgb: Any,
        ego_state: EgoStateSample,
    ) -> None:
        log_time_ns = int(round(ego_state.timestamp_s * 1_000_000_000))
        jpeg_buffer = io.BytesIO()
        Image.fromarray(current_rgb).save(jpeg_buffer, format="JPEG", quality=self._jpeg_quality)
        self._writer.add_message(
            channel_id=self._front_rgb_channel_id,
            log_time=log_time_ns,
            publish_time=log_time_ns,
            sequence=ego_state.frame_id,
            data=jpeg_buffer.getvalue(),
        )
        self._writer.add_message(
            channel_id=self._ego_state_channel_id,
            log_time=log_time_ns,
            publish_time=log_time_ns,
            sequence=ego_state.frame_id,
            data=ego_state.to_bytes(),
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._writer.finish()
        self._stream.close()
