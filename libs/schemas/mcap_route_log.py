from __future__ import annotations

from base64 import b64encode
from dataclasses import dataclass
import io
import json
from math import cos, radians, sin
from pathlib import Path
from typing import Any

from mcap.writer import CompressionType, Writer
from PIL import Image


def _carla_point_to_foxglove(*, x: float, y: float, z: float) -> dict[str, float]:
    return {"x": float(x), "y": float(-y), "z": float(z)}


def _carla_rpy_to_foxglove(*, roll_deg: float, pitch_deg: float, yaw_deg: float) -> tuple[float, float, float]:
    return float(roll_deg), float(-pitch_deg), float(-yaw_deg)


def _foxglove_time_schema() -> dict[str, Any]:
    return {
        "type": "object",
        "properties": {
            "sec": {"type": "integer"},
            "nsec": {"type": "integer"},
        },
        "required": ["sec", "nsec"],
        "additionalProperties": False,
    }


_FOXGLOVE_COMPRESSED_IMAGE_SCHEMA = {
    "type": "object",
    "properties": {
        "timestamp": _foxglove_time_schema(),
        "frame_id": {"type": "string"},
        "data": {"type": "string", "contentEncoding": "base64"},
        "format": {"type": "string"},
    },
    "required": ["timestamp", "frame_id", "data", "format"],
    "additionalProperties": False,
}

_EGO_STATE_JSON_SCHEMA = {
    "type": "object",
    "properties": {
        "timestamp": _foxglove_time_schema(),
        "episode_id": {"type": "string"},
        "frame_id": {"type": "integer"},
        "elapsed_seconds": {"type": "number"},
        "speed_mps": {"type": "number"},
        "route_completion_ratio": {"type": "number"},
        "distance_to_goal_m": {"type": "number"},
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
            "additionalProperties": False,
        },
        "control": {
            "type": "object",
            "properties": {
                "steer": {"type": "number"},
                "throttle": {"type": "number"},
                "brake": {"type": "number"},
            },
            "required": ["steer", "throttle", "brake"],
            "additionalProperties": False,
        },
    },
    "required": [
        "timestamp",
        "episode_id",
        "frame_id",
        "elapsed_seconds",
        "speed_mps",
        "route_completion_ratio",
        "distance_to_goal_m",
        "pose",
        "control",
    ],
    "additionalProperties": False,
}

_EGO_CONTROL_JSON_SCHEMA = {
    "type": "object",
    "properties": {
        "timestamp": _foxglove_time_schema(),
        "episode_id": {"type": "string"},
        "frame_id": {"type": "integer"},
        "elapsed_seconds": {"type": "number"},
        "steer": {"type": "number"},
        "throttle": {"type": "number"},
        "brake": {"type": "number"},
    },
    "required": [
        "timestamp",
        "episode_id",
        "frame_id",
        "elapsed_seconds",
        "steer",
        "throttle",
        "brake",
    ],
    "additionalProperties": False,
}

_EGO_PLANNING_JSON_SCHEMA = {
    "type": "object",
    "properties": {
        "timestamp": _foxglove_time_schema(),
        "episode_id": {"type": "string"},
        "frame_id": {"type": "integer"},
        "elapsed_seconds": {"type": "number"},
        "behavior": {"type": ["string", "null"]},
        "planner_state": {"type": ["string", "null"]},
        "traffic_light_state": {"type": ["string", "null"]},
        "overtake_state": {"type": ["string", "null"]},
        "target_lane_id": {"type": ["string", "null"]},
        "min_ttc": {"type": ["number", "null"]},
    },
    "required": [
        "timestamp",
        "episode_id",
        "frame_id",
        "elapsed_seconds",
        "behavior",
        "planner_state",
        "traffic_light_state",
        "overtake_state",
        "target_lane_id",
        "min_ttc",
    ],
    "additionalProperties": False,
}

_FOXGLOVE_SCENE_UPDATE_SCHEMA = {
    "type": "object",
    "properties": {
        "deletions": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "timestamp": _foxglove_time_schema(),
                    "type": {"type": "integer"},
                    "id": {"type": "string"},
                },
                "required": ["timestamp", "type", "id"],
                "additionalProperties": False,
            },
        },
        "entities": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "timestamp": _foxglove_time_schema(),
                    "frame_id": {"type": "string"},
                    "id": {"type": "string"},
                    "lifetime": _foxglove_time_schema(),
                    "frame_locked": {"type": "boolean"},
                    "metadata": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "key": {"type": "string"},
                                "value": {"type": "string"},
                            },
                            "required": ["key", "value"],
                            "additionalProperties": False,
                        },
                    },
                    "lines": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "type": {"type": "integer"},
                                "pose": {
                                    "type": "object",
                                    "properties": {
                                        "position": {
                                            "type": "object",
                                            "properties": {
                                                "x": {"type": "number"},
                                                "y": {"type": "number"},
                                                "z": {"type": "number"},
                                            },
                                            "required": ["x", "y", "z"],
                                            "additionalProperties": False,
                                        },
                                        "orientation": {
                                            "type": "object",
                                            "properties": {
                                                "x": {"type": "number"},
                                                "y": {"type": "number"},
                                                "z": {"type": "number"},
                                                "w": {"type": "number"},
                                            },
                                            "required": ["x", "y", "z", "w"],
                                            "additionalProperties": False,
                                        },
                                    },
                                    "required": ["position", "orientation"],
                                    "additionalProperties": False,
                                },
                                "thickness": {"type": "number"},
                                "scale_invariant": {"type": "boolean"},
                                "points": {
                                    "type": "array",
                                    "items": {
                                        "type": "object",
                                        "properties": {
                                            "x": {"type": "number"},
                                            "y": {"type": "number"},
                                            "z": {"type": "number"},
                                        },
                                        "required": ["x", "y", "z"],
                                        "additionalProperties": False,
                                    },
                                },
                                "color": {
                                    "type": "object",
                                    "properties": {
                                        "r": {"type": "number"},
                                        "g": {"type": "number"},
                                        "b": {"type": "number"},
                                        "a": {"type": "number"},
                                    },
                                    "required": ["r", "g", "b", "a"],
                                    "additionalProperties": False,
                                },
                                "indices": {
                                    "type": "array",
                                    "items": {"type": "integer"},
                                },
                            },
                            "required": ["type", "pose", "thickness", "scale_invariant", "points", "color"],
                            "additionalProperties": False,
                        },
                    },
                },
                "required": ["timestamp", "frame_id", "id", "lifetime", "frame_locked", "metadata", "lines"],
                "additionalProperties": False,
            },
        },
    },
    "required": ["deletions", "entities"],
    "additionalProperties": False,
}

_FOXGLOVE_FRAME_TRANSFORMS_SCHEMA = {
    "type": "object",
    "properties": {
        "transforms": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "timestamp": _foxglove_time_schema(),
                    "parent_frame_id": {"type": "string"},
                    "child_frame_id": {"type": "string"},
                    "translation": {
                        "type": "object",
                        "properties": {
                            "x": {"type": "number"},
                            "y": {"type": "number"},
                            "z": {"type": "number"},
                        },
                        "required": ["x", "y", "z"],
                        "additionalProperties": False,
                    },
                    "rotation": {
                        "type": "object",
                        "properties": {
                            "x": {"type": "number"},
                            "y": {"type": "number"},
                            "z": {"type": "number"},
                            "w": {"type": "number"},
                        },
                        "required": ["x", "y", "z", "w"],
                        "additionalProperties": False,
                    },
                },
                "required": ["timestamp", "parent_frame_id", "child_frame_id", "translation", "rotation"],
                "additionalProperties": False,
            },
        }
    },
    "required": ["transforms"],
    "additionalProperties": False,
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


def _foxglove_time(timestamp_s: float) -> dict[str, int]:
    sec = int(timestamp_s)
    nsec = int(round((timestamp_s - sec) * 1_000_000_000))
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    return {"sec": sec, "nsec": nsec}


def _euler_degrees_to_quaternion(*, roll_deg: float, pitch_deg: float, yaw_deg: float) -> dict[str, float]:
    roll = radians(roll_deg)
    pitch = radians(pitch_deg)
    yaw = radians(yaw_deg)
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    return {
        "x": sr * cp * cy - cr * sp * sy,
        "y": cr * sp * cy + sr * cp * sy,
        "z": cr * cp * sy - sr * sp * cy,
        "w": cr * cp * cy + sr * sp * sy,
    }


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
        self._writer = Writer(
            self._stream,
            compression=CompressionType.ZSTD,
            use_chunking=True,
        )
        self._writer.start(profile="carla_alpamayo.route_loop", library="carla_alpamayo")

        compressed_image_schema_id = self._writer.register_schema(
            name="foxglove.CompressedImage",
            encoding="jsonschema",
            data=json.dumps(_FOXGLOVE_COMPRESSED_IMAGE_SCHEMA, ensure_ascii=False).encode("utf-8"),
        )
        scene_update_schema_id = self._writer.register_schema(
            name="foxglove.SceneUpdate",
            encoding="jsonschema",
            data=json.dumps(_FOXGLOVE_SCENE_UPDATE_SCHEMA, ensure_ascii=False).encode("utf-8"),
        )
        frame_transforms_schema_id = self._writer.register_schema(
            name="foxglove.FrameTransforms",
            encoding="jsonschema",
            data=json.dumps(_FOXGLOVE_FRAME_TRANSFORMS_SCHEMA, ensure_ascii=False).encode("utf-8"),
        )
        ego_state_schema_id = self._writer.register_schema(
            name="carla_alpamayo.EgoState",
            encoding="jsonschema",
            data=json.dumps(_EGO_STATE_JSON_SCHEMA, ensure_ascii=False).encode("utf-8"),
        )
        ego_control_schema_id = self._writer.register_schema(
            name="carla_alpamayo.EgoControl",
            encoding="jsonschema",
            data=json.dumps(_EGO_CONTROL_JSON_SCHEMA, ensure_ascii=False).encode("utf-8"),
        )
        ego_planning_schema_id = self._writer.register_schema(
            name="carla_alpamayo.EgoPlanning",
            encoding="jsonschema",
            data=json.dumps(_EGO_PLANNING_JSON_SCHEMA, ensure_ascii=False).encode("utf-8"),
        )

        self._front_rgb_channel_id = self._writer.register_channel(
            topic="/camera/front/compressed",
            message_encoding="json",
            schema_id=compressed_image_schema_id,
            metadata={
                "frame_id": "ego/front_camera",
                "camera_width": str(camera_width),
                "camera_height": str(camera_height),
                "jpeg_quality": str(jpeg_quality),
            },
        )
        self._map_scene_channel_id = self._writer.register_channel(
            topic="/map/scene",
            message_encoding="json",
            schema_id=scene_update_schema_id,
            metadata={"frame_id": "map"},
        )
        self._frame_transforms_channel_id = self._writer.register_channel(
            topic="/tf",
            message_encoding="json",
            schema_id=frame_transforms_schema_id,
        )
        self._ego_state_channel_id = self._writer.register_channel(
            topic="/ego/state",
            message_encoding="json",
            schema_id=ego_state_schema_id,
        )
        self._ego_control_channel_id = self._writer.register_channel(
            topic="/ego/control",
            message_encoding="json",
            schema_id=ego_control_schema_id,
        )
        self._ego_planning_channel_id = self._writer.register_channel(
            topic="/ego/planning",
            message_encoding="json",
            schema_id=ego_planning_schema_id,
        )

        self._writer.add_metadata(
            "episode",
            {
                "episode_id": episode_id,
                "route_name": route_name,
                "town": town,
                "weather": weather,
                "compression": "zstd_chunked",
            },
        )
        self._jpeg_quality = jpeg_quality
        self._closed = False

    def write_static_scene(
        self,
        *,
        timestamp_s: float,
        route_name: str,
        route_points: list[tuple[float, float, float]],
        lane_centerlines: list[list[tuple[float, float, float]]],
    ) -> None:
        log_time_ns = int(round(timestamp_s * 1_000_000_000))
        timestamp = _foxglove_time(timestamp_s)
        zero_duration = {"sec": 0, "nsec": 0}
        identity_pose = {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        }

        lane_lines = [
            {
                "type": 0,
                "pose": identity_pose,
                "thickness": 0.12,
                "scale_invariant": False,
                "points": [_carla_point_to_foxglove(x=x, y=y, z=z) for x, y, z in line],
                "color": {"r": 0.55, "g": 0.55, "b": 0.55, "a": 0.65},
                "indices": [],
            }
            for line in lane_centerlines
            if len(line) >= 2
        ]

        route_lines = [
            {
                "type": 0,
                "pose": identity_pose,
                "thickness": 0.3,
                "scale_invariant": False,
                "points": [_carla_point_to_foxglove(x=x, y=y, z=z) for x, y, z in route_points],
                "color": {"r": 0.08, "g": 0.72, "b": 1.0, "a": 0.95},
                "indices": [],
            }
        ] if len(route_points) >= 2 else []

        scene_update = {
            "deletions": [],
            "entities": [
                {
                    "timestamp": timestamp,
                    "frame_id": "map",
                    "id": "lane_centerlines",
                    "lifetime": zero_duration,
                    "frame_locked": True,
                    "metadata": [
                        {"key": "kind", "value": "lane_centerlines"},
                        {"key": "route_name", "value": route_name},
                    ],
                    "lines": lane_lines,
                },
                {
                    "timestamp": timestamp,
                    "frame_id": "map",
                    "id": "planned_route",
                    "lifetime": zero_duration,
                    "frame_locked": True,
                    "metadata": [
                        {"key": "kind", "value": "planned_route"},
                        {"key": "route_name", "value": route_name},
                    ],
                    "lines": route_lines,
                },
            ],
        }
        self._writer.add_message(
            channel_id=self._map_scene_channel_id,
            log_time=log_time_ns,
            publish_time=log_time_ns,
            sequence=0,
            data=json.dumps(scene_update, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
        )

    def write_frame(
        self,
        *,
        current_rgb: Any,
        ego_state: EgoStateSample,
    ) -> None:
        log_time_ns = int(round(ego_state.timestamp_s * 1_000_000_000))
        timestamp = _foxglove_time(ego_state.timestamp_s)
        pose = ego_state.pose
        foxglove_position = _carla_point_to_foxglove(x=pose["x"], y=pose["y"], z=pose["z"])
        foxglove_roll_deg, foxglove_pitch_deg, foxglove_yaw_deg = _carla_rpy_to_foxglove(
            roll_deg=pose["roll_deg"],
            pitch_deg=pose["pitch_deg"],
            yaw_deg=pose["yaw_deg"],
        )

        jpeg_buffer = io.BytesIO()
        Image.fromarray(current_rgb).save(jpeg_buffer, format="JPEG", quality=self._jpeg_quality)
        compressed_image = {
            "timestamp": timestamp,
            "frame_id": "ego/front_camera",
            "data": b64encode(jpeg_buffer.getvalue()).decode("ascii"),
            "format": "jpeg",
        }
        self._writer.add_message(
            channel_id=self._front_rgb_channel_id,
            log_time=log_time_ns,
            publish_time=log_time_ns,
            sequence=ego_state.frame_id,
            data=json.dumps(compressed_image, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
        )

        base_link_orientation = _euler_degrees_to_quaternion(
            roll_deg=foxglove_roll_deg,
            pitch_deg=foxglove_pitch_deg,
            yaw_deg=foxglove_yaw_deg,
        )
        frame_transforms = {
            "transforms": [
                {
                    "timestamp": timestamp,
                    "parent_frame_id": "map",
                    "child_frame_id": "ego/base_link",
                    "translation": foxglove_position,
                    "rotation": base_link_orientation,
                },
                {
                    "timestamp": timestamp,
                    "parent_frame_id": "ego/base_link",
                    "child_frame_id": "ego/front_camera",
                    "translation": {
                        "x": 1.5,
                        "y": 0.0,
                        "z": 2.4,
                    },
                    "rotation": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                        "w": 1.0,
                    },
                },
            ]
        }
        self._writer.add_message(
            channel_id=self._frame_transforms_channel_id,
            log_time=log_time_ns,
            publish_time=log_time_ns,
            sequence=ego_state.frame_id,
            data=json.dumps(frame_transforms, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
        )

        ego_state_message = {
            "timestamp": timestamp,
            "episode_id": ego_state.episode_id,
            "frame_id": ego_state.frame_id,
            "elapsed_seconds": ego_state.elapsed_seconds,
            "speed_mps": ego_state.speed_mps,
            "route_completion_ratio": ego_state.route_completion_ratio,
            "distance_to_goal_m": ego_state.distance_to_goal_m,
            "pose": {
                "x": foxglove_position["x"],
                "y": foxglove_position["y"],
                "z": foxglove_position["z"],
                "yaw_deg": foxglove_yaw_deg,
                "pitch_deg": foxglove_pitch_deg,
                "roll_deg": foxglove_roll_deg,
            },
        }
        self._writer.add_message(
            channel_id=self._ego_state_channel_id,
            log_time=log_time_ns,
            publish_time=log_time_ns,
            sequence=ego_state.frame_id,
            data=json.dumps(ego_state_message, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
        )

        ego_control_message = {
            "timestamp": timestamp,
            "episode_id": ego_state.episode_id,
            "frame_id": ego_state.frame_id,
            "elapsed_seconds": ego_state.elapsed_seconds,
            "steer": float(ego_state.control["steer"]),
            "throttle": float(ego_state.control["throttle"]),
            "brake": float(ego_state.control["brake"]),
        }
        self._writer.add_message(
            channel_id=self._ego_control_channel_id,
            log_time=log_time_ns,
            publish_time=log_time_ns,
            sequence=ego_state.frame_id,
            data=json.dumps(ego_control_message, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
        )

        ego_planning_message = {
            "timestamp": timestamp,
            "episode_id": ego_state.episode_id,
            "frame_id": ego_state.frame_id,
            "elapsed_seconds": ego_state.elapsed_seconds,
            "behavior": ego_state.behavior,
            "planner_state": ego_state.planner_state,
            "traffic_light_state": ego_state.traffic_light_state,
            "overtake_state": ego_state.overtake_state,
            "target_lane_id": ego_state.target_lane_id,
            "min_ttc": ego_state.min_ttc,
        }
        self._writer.add_message(
            channel_id=self._ego_planning_channel_id,
            log_time=log_time_ns,
            publish_time=log_time_ns,
            sequence=ego_state.frame_id,
            data=json.dumps(ego_planning_message, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._writer.finish()
        self._stream.close()
