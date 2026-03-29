from __future__ import annotations

import io
import json
from base64 import b64encode
from dataclasses import dataclass
from math import cos, radians, sin, tan
from typing import TYPE_CHECKING, Any

from mcap.writer import CompressionType, Writer
from PIL import Image

from ad_stack.overtake.domain import build_planning_debug_message_schema

if TYPE_CHECKING:
    from pathlib import Path

    from libs.carla_utils.map_raster import TopdownMapAsset


def _carla_point_to_foxglove(*, x: float, y: float, z: float) -> dict[str, float]:
    return {"x": float(x), "y": float(-y), "z": float(z)}


def _carla_rpy_to_foxglove(
    *, roll_deg: float, pitch_deg: float, yaw_deg: float
) -> tuple[float, float, float]:
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

_FOXGLOVE_CAMERA_CALIBRATION_SCHEMA = {
    "type": "object",
    "properties": {
        "timestamp": _foxglove_time_schema(),
        "frame_id": {"type": "string"},
        "width": {"type": "integer"},
        "height": {"type": "integer"},
        "distortion_model": {"type": "string"},
        "D": {"type": "array", "items": {"type": "number"}},
        "K": {"type": "array", "items": {"type": "number"}, "minItems": 9, "maxItems": 9},
        "R": {"type": "array", "items": {"type": "number"}, "minItems": 9, "maxItems": 9},
        "P": {"type": "array", "items": {"type": "number"}, "minItems": 12, "maxItems": 12},
    },
    "required": [
        "timestamp",
        "frame_id",
        "width",
        "height",
        "distortion_model",
        "D",
        "K",
        "R",
        "P",
    ],
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

_EGO_PLANNING_DEBUG_JSON_SCHEMA = build_planning_debug_message_schema(_foxglove_time_schema())

_NPC_VEHICLE_STATES_JSON_SCHEMA = {
    "type": "object",
    "properties": {
        "timestamp": _foxglove_time_schema(),
        "episode_id": {"type": "string"},
        "frame_id": {"type": "integer"},
        "elapsed_seconds": {"type": "number"},
        "vehicles": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "actor_id": {"type": "integer"},
                    "type_id": {"type": ["string", "null"]},
                    "spawn_index": {"type": ["integer", "null"]},
                    "spawn_transform": {
                        "type": ["object", "null"],
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
                    "target_speed_kmh": {"type": ["number", "null"]},
                    "npc_profile_id": {"type": ["string", "null"]},
                    "lane_behavior": {"type": ["string", "null"]},
                    "autopilot_enabled": {"type": ["boolean", "null"]},
                    "speed_mps": {"type": "number"},
                    "lane_id": {"type": ["string", "null"]},
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
                    "size": {
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
                "required": [
                    "actor_id",
                    "type_id",
                    "spawn_index",
                    "target_speed_kmh",
                    "npc_profile_id",
                    "lane_behavior",
                    "autopilot_enabled",
                    "speed_mps",
                    "lane_id",
                    "pose",
                    "size",
                ],
                "additionalProperties": False,
            },
        },
    },
    "required": ["timestamp", "episode_id", "frame_id", "elapsed_seconds", "vehicles"],
    "additionalProperties": False,
}

_TRACKED_VEHICLE_STATES_JSON_SCHEMA = {
    "type": "object",
    "properties": {
        "timestamp": _foxglove_time_schema(),
        "episode_id": {"type": "string"},
        "frame_id": {"type": "integer"},
        "elapsed_seconds": {"type": "number"},
        "vehicles": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "actor_id": {"type": "integer"},
                    "relation": {"type": "string"},
                    "lane_id": {"type": ["string", "null"]},
                    "speed_mps": {"type": "number"},
                    "longitudinal_distance_m": {"type": ["number", "null"]},
                    "lateral_distance_m": {"type": ["number", "null"]},
                    "is_ahead": {"type": "boolean"},
                    "pose": {
                        "type": "object",
                        "properties": {
                            "x": {"type": "number"},
                            "y": {"type": "number"},
                            "yaw_deg": {"type": "number"},
                        },
                        "required": ["x", "y", "yaw_deg"],
                        "additionalProperties": False,
                    },
                },
                "required": [
                    "actor_id",
                    "relation",
                    "lane_id",
                    "speed_mps",
                    "longitudinal_distance_m",
                    "lateral_distance_m",
                    "is_ahead",
                    "pose",
                ],
                "additionalProperties": False,
            },
        },
    },
    "required": ["timestamp", "episode_id", "frame_id", "elapsed_seconds", "vehicles"],
    "additionalProperties": False,
}

_TRAFFIC_LIGHT_OBSERVATIONS_JSON_SCHEMA = {
    "type": "object",
    "properties": {
        "timestamp": _foxglove_time_schema(),
        "episode_id": {"type": "string"},
        "frame_id": {"type": "integer"},
        "elapsed_seconds": {"type": "number"},
        "lights": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "actor_id": {"type": "integer"},
                    "state": {"type": "string"},
                    "affects_ego": {"type": "boolean"},
                    "distance_m": {"type": "number"},
                    "stop_line_distance_m": {"type": ["number", "null"]},
                },
                "required": [
                    "actor_id",
                    "state",
                    "affects_ego",
                    "distance_m",
                    "stop_line_distance_m",
                ],
                "additionalProperties": False,
            },
        },
    },
    "required": ["timestamp", "episode_id", "frame_id", "elapsed_seconds", "lights"],
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
                    "cubes": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
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
                                "size": {
                                    "type": "object",
                                    "properties": {
                                        "x": {"type": "number"},
                                        "y": {"type": "number"},
                                        "z": {"type": "number"},
                                    },
                                    "required": ["x", "y", "z"],
                                    "additionalProperties": False,
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
                            },
                            "required": ["pose", "size", "color"],
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
                            "required": [
                                "type",
                                "pose",
                                "thickness",
                                "scale_invariant",
                                "points",
                                "color",
                            ],
                            "additionalProperties": False,
                        },
                    },
                },
                "required": ["timestamp", "frame_id", "id", "lifetime", "frame_locked", "metadata"],
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
                "required": [
                    "timestamp",
                    "parent_frame_id",
                    "child_frame_id",
                    "translation",
                    "rotation",
                ],
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
    follow_target_distance_m: float | None
    overtake_state: str | None
    target_lane_id: str | None
    min_ttc: float | None
    pose: dict[str, float]
    control: dict[str, float]
    planning_debug: dict[str, Any] | None = None


@dataclass(slots=True)
class NPCVehicleStateSample:
    actor_id: int
    type_id: str | None
    spawn_index: int | None
    spawn_transform: dict[str, float] | None
    target_speed_kmh: float | None
    npc_profile_id: str | None
    lane_behavior: str | None
    autopilot_enabled: bool | None
    speed_mps: float
    lane_id: str | None
    pose: dict[str, float]
    size: dict[str, float]


@dataclass(slots=True)
class TrackedVehicleStateSample:
    actor_id: int
    relation: str
    lane_id: str | None
    speed_mps: float
    longitudinal_distance_m: float | None
    lateral_distance_m: float | None
    is_ahead: bool
    pose: dict[str, float]


@dataclass(slots=True)
class TrafficLightObservationSample:
    actor_id: int
    state: str
    affects_ego: bool
    distance_m: float
    stop_line_distance_m: float | None


@dataclass(slots=True)
class McapSegmentInfo:
    segment_index: int
    path: Path
    start_elapsed_seconds: float
    end_elapsed_seconds: float | None = None
    frame_count: int = 0


def _foxglove_time(timestamp_s: float) -> dict[str, int]:
    sec = int(timestamp_s)
    nsec = int(round((timestamp_s - sec) * 1_000_000_000))
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    return {"sec": sec, "nsec": nsec}


def _euler_degrees_to_quaternion(
    *, roll_deg: float, pitch_deg: float, yaw_deg: float
) -> dict[str, float]:
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
        topdown_map_asset: TopdownMapAsset | None = None,
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
        camera_calibration_schema_id = self._writer.register_schema(
            name="foxglove.CameraCalibration",
            encoding="jsonschema",
            data=json.dumps(_FOXGLOVE_CAMERA_CALIBRATION_SCHEMA, ensure_ascii=False).encode(
                "utf-8"
            ),
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
        ego_planning_debug_schema_id = self._writer.register_schema(
            name="carla_alpamayo.EgoPlanningDebug",
            encoding="jsonschema",
            data=json.dumps(_EGO_PLANNING_DEBUG_JSON_SCHEMA, ensure_ascii=False).encode("utf-8"),
        )
        npc_vehicle_states_schema_id = self._writer.register_schema(
            name="carla_alpamayo.NpcVehicleStates",
            encoding="jsonschema",
            data=json.dumps(_NPC_VEHICLE_STATES_JSON_SCHEMA, ensure_ascii=False).encode("utf-8"),
        )
        tracked_vehicle_states_schema_id = self._writer.register_schema(
            name="carla_alpamayo.TrackedVehicleStates",
            encoding="jsonschema",
            data=json.dumps(_TRACKED_VEHICLE_STATES_JSON_SCHEMA, ensure_ascii=False).encode(
                "utf-8"
            ),
        )
        traffic_light_observations_schema_id = self._writer.register_schema(
            name="carla_alpamayo.TrafficLightObservations",
            encoding="jsonschema",
            data=json.dumps(
                _TRAFFIC_LIGHT_OBSERVATIONS_JSON_SCHEMA, ensure_ascii=False
            ).encode("utf-8"),
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
        self._ego_marker_channel_id = self._writer.register_channel(
            topic="/ego/marker",
            message_encoding="json",
            schema_id=scene_update_schema_id,
            metadata={"frame_id": "ego/base_link"},
        )
        self._npc_marker_channel_id = self._writer.register_channel(
            topic="/world/npc_markers",
            message_encoding="json",
            schema_id=scene_update_schema_id,
            metadata={"frame_id": "map"},
        )
        self._topdown_image_channel_id = self._writer.register_channel(
            topic="/map/topdown/compressed",
            message_encoding="json",
            schema_id=compressed_image_schema_id,
        )
        self._topdown_calibration_channel_id = self._writer.register_channel(
            topic="/map/topdown/camera_info",
            message_encoding="json",
            schema_id=camera_calibration_schema_id,
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
        self._ego_planning_debug_channel_id = self._writer.register_channel(
            topic="/ego/planning_debug",
            message_encoding="json",
            schema_id=ego_planning_debug_schema_id,
        )
        self._npc_vehicle_states_channel_id = self._writer.register_channel(
            topic="/world/npc_vehicles",
            message_encoding="json",
            schema_id=npc_vehicle_states_schema_id,
        )
        self._tracked_vehicle_states_channel_id = self._writer.register_channel(
            topic="/world/tracked_vehicles",
            message_encoding="json",
            schema_id=tracked_vehicle_states_schema_id,
        )
        self._traffic_light_observations_channel_id = self._writer.register_channel(
            topic="/world/traffic_lights",
            message_encoding="json",
            schema_id=traffic_light_observations_schema_id,
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
        self._topdown_map_asset = topdown_map_asset
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

        route_lines = (
            [
                {
                    "type": 0,
                    "pose": identity_pose,
                    "thickness": 0.3,
                    "scale_invariant": False,
                    "points": [_carla_point_to_foxglove(x=x, y=y, z=z) for x, y, z in route_points],
                    "color": {"r": 0.08, "g": 0.72, "b": 1.0, "a": 0.95},
                    "indices": [],
                }
            ]
            if len(route_points) >= 2
            else []
        )

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
            data=json.dumps(scene_update, ensure_ascii=False, separators=(",", ":")).encode(
                "utf-8"
            ),
        )

        if self._topdown_map_asset is not None:
            self._write_topdown_map_static(timestamp_s=timestamp_s)

    def _write_topdown_map_static(self, *, timestamp_s: float) -> None:
        assert self._topdown_map_asset is not None
        asset = self._topdown_map_asset
        log_time_ns = int(round(timestamp_s * 1_000_000_000))
        timestamp = _foxglove_time(timestamp_s)
        png_bytes = asset.image_path.read_bytes()

        image_message = {
            "timestamp": timestamp,
            "frame_id": asset.frame_id,
            "data": b64encode(png_bytes).decode("ascii"),
            "format": "png",
        }
        self._writer.add_message(
            channel_id=self._topdown_image_channel_id,
            log_time=log_time_ns,
            publish_time=log_time_ns,
            sequence=0,
            data=json.dumps(image_message, ensure_ascii=False, separators=(",", ":")).encode(
                "utf-8"
            ),
        )

        if asset.camera_fov_deg > 0.0:
            fx = asset.width / max(1e-6, 2.0 * tan(radians(asset.camera_fov_deg) / 2.0))
            fy = fx
        else:
            fx = asset.camera_height_m * asset.width / max(1e-6, asset.max_x - asset.min_x)
            fy = asset.camera_height_m * asset.height / max(1e-6, asset.max_y - asset.min_y)
        cx = asset.width / 2.0
        cy = asset.height / 2.0
        calibration_message = {
            "timestamp": timestamp,
            "frame_id": asset.frame_id,
            "width": asset.width,
            "height": asset.height,
            "distortion_model": "plumb_bob",
            "D": [0.0, 0.0, 0.0, 0.0, 0.0],
            "K": [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
            "R": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "P": [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        }
        self._writer.add_message(
            channel_id=self._topdown_calibration_channel_id,
            log_time=log_time_ns,
            publish_time=log_time_ns,
            sequence=0,
            data=json.dumps(calibration_message, ensure_ascii=False, separators=(",", ":")).encode(
                "utf-8"
            ),
        )

        topdown_transform = {
            "transforms": [
                {
                    "timestamp": timestamp,
                    "parent_frame_id": "map",
                    "child_frame_id": asset.frame_id,
                    "translation": {
                        "x": asset.center_x,
                        "y": asset.center_y,
                        "z": asset.camera_height_m,
                    },
                    "rotation": {
                        "x": 1.0,
                        "y": 0.0,
                        "z": 0.0,
                        "w": 0.0,
                    },
                }
            ]
        }
        self._writer.add_message(
            channel_id=self._frame_transforms_channel_id,
            log_time=log_time_ns,
            publish_time=log_time_ns,
            sequence=0,
            data=json.dumps(topdown_transform, ensure_ascii=False, separators=(",", ":")).encode(
                "utf-8"
            ),
        )

    def write_frame(
        self,
        *,
        current_rgb: Any,
        ego_state: EgoStateSample,
        npc_vehicle_states: list[NPCVehicleStateSample] | None = None,
        tracked_vehicle_states: list[TrackedVehicleStateSample] | None = None,
        traffic_light_states: list[TrafficLightObservationSample] | None = None,
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
            data=json.dumps(compressed_image, ensure_ascii=False, separators=(",", ":")).encode(
                "utf-8"
            ),
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
                        "z": 1.5,
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
            data=json.dumps(frame_transforms, ensure_ascii=False, separators=(",", ":")).encode(
                "utf-8"
            ),
        )

        ego_marker_message = {
            "deletions": [],
            "entities": [
                {
                    "timestamp": timestamp,
                    "frame_id": "ego/base_link",
                    "id": "ego_vehicle_box",
                    "lifetime": {"sec": 0, "nsec": 0},
                    "frame_locked": True,
                    "metadata": [
                        {"key": "kind", "value": "ego_vehicle_marker"},
                    ],
                    "cubes": [
                        {
                            "pose": {
                                "position": {"x": 0.0, "y": 0.0, "z": 0.8},
                                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                            },
                            "size": {"x": 4.6, "y": 1.9, "z": 1.6},
                            "color": {"r": 0.95, "g": 0.16, "b": 0.16, "a": 0.9},
                        }
                    ],
                }
            ],
        }
        self._writer.add_message(
            channel_id=self._ego_marker_channel_id,
            log_time=log_time_ns,
            publish_time=log_time_ns,
            sequence=ego_state.frame_id,
            data=json.dumps(ego_marker_message, ensure_ascii=False, separators=(",", ":")).encode(
                "utf-8"
            ),
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
            data=json.dumps(ego_state_message, ensure_ascii=False, separators=(",", ":")).encode(
                "utf-8"
            ),
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
            data=json.dumps(ego_control_message, ensure_ascii=False, separators=(",", ":")).encode(
                "utf-8"
            ),
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
            data=json.dumps(ego_planning_message, ensure_ascii=False, separators=(",", ":")).encode(
                "utf-8"
            ),
        )

        if ego_state.planning_debug is not None:
            ego_planning_debug_message = {
                "timestamp": timestamp,
                "episode_id": ego_state.episode_id,
                "frame_id": ego_state.frame_id,
                "elapsed_seconds": ego_state.elapsed_seconds,
                **ego_state.planning_debug,
            }
            self._writer.add_message(
                channel_id=self._ego_planning_debug_channel_id,
                log_time=log_time_ns,
                publish_time=log_time_ns,
                sequence=ego_state.frame_id,
                data=json.dumps(
                    ego_planning_debug_message, ensure_ascii=False, separators=(",", ":")
                ).encode("utf-8"),
            )

        if npc_vehicle_states is not None:
            npc_vehicle_states_message = {
                "timestamp": timestamp,
                "episode_id": ego_state.episode_id,
                "frame_id": ego_state.frame_id,
                "elapsed_seconds": ego_state.elapsed_seconds,
                "vehicles": [
                    {
                        "actor_id": int(vehicle.actor_id),
                        "type_id": vehicle.type_id,
                        "spawn_index": vehicle.spawn_index,
                        "spawn_transform": vehicle.spawn_transform,
                        "target_speed_kmh": vehicle.target_speed_kmh,
                        "npc_profile_id": vehicle.npc_profile_id,
                        "lane_behavior": vehicle.lane_behavior,
                        "autopilot_enabled": vehicle.autopilot_enabled,
                        "speed_mps": float(vehicle.speed_mps),
                        "lane_id": vehicle.lane_id,
                        "pose": {
                            "x": float(vehicle.pose["x"]),
                            "y": float(vehicle.pose["y"]),
                            "z": float(vehicle.pose["z"]),
                            "yaw_deg": float(vehicle.pose["yaw_deg"]),
                            "pitch_deg": float(vehicle.pose["pitch_deg"]),
                            "roll_deg": float(vehicle.pose["roll_deg"]),
                        },
                        "size": {
                            "x": float(vehicle.size["x"]),
                            "y": float(vehicle.size["y"]),
                            "z": float(vehicle.size["z"]),
                        },
                    }
                    for vehicle in npc_vehicle_states
                ],
            }
            self._writer.add_message(
                channel_id=self._npc_vehicle_states_channel_id,
                log_time=log_time_ns,
                publish_time=log_time_ns,
                sequence=ego_state.frame_id,
                data=json.dumps(
                    npc_vehicle_states_message, ensure_ascii=False, separators=(",", ":")
                ).encode("utf-8"),
            )

            npc_marker_message = {
                "deletions": [],
                "entities": [
                    {
                        "timestamp": timestamp,
                        "frame_id": "map",
                        "id": f"npc_vehicle_{vehicle.actor_id}",
                        "lifetime": {"sec": 0, "nsec": 0},
                        "frame_locked": True,
                        "metadata": [
                            {"key": "kind", "value": "npc_vehicle_marker"},
                            {"key": "actor_id", "value": str(vehicle.actor_id)},
                            {"key": "type_id", "value": vehicle.type_id or ""},
                            {"key": "lane_id", "value": vehicle.lane_id or ""},
                            {"key": "npc_profile_id", "value": vehicle.npc_profile_id or ""},
                        ],
                        "cubes": [
                            {
                                "pose": {
                                    "position": _carla_point_to_foxglove(
                                        x=float(vehicle.pose["x"]),
                                        y=float(vehicle.pose["y"]),
                                        z=float(vehicle.pose["z"] + (vehicle.size["z"] * 0.5)),
                                    ),
                                    "orientation": _euler_degrees_to_quaternion(
                                        roll_deg=vehicle.pose["roll_deg"],
                                        pitch_deg=vehicle.pose["pitch_deg"],
                                        yaw_deg=vehicle.pose["yaw_deg"],
                                    ),
                                },
                                "size": {
                                    "x": float(vehicle.size["x"]),
                                    "y": float(vehicle.size["y"]),
                                    "z": float(vehicle.size["z"]),
                                },
                                "color": (
                                    {"r": 0.92, "g": 0.26, "b": 0.18, "a": 0.9}
                                    if (
                                        vehicle.autopilot_enabled is False
                                        or vehicle.speed_mps <= 0.25
                                    )
                                    else {"r": 0.32, "g": 0.62, "b": 0.92, "a": 0.75}
                                ),
                            }
                        ],
                    }
                    for vehicle in npc_vehicle_states
                ],
            }
            self._writer.add_message(
                channel_id=self._npc_marker_channel_id,
                log_time=log_time_ns,
                publish_time=log_time_ns,
                sequence=ego_state.frame_id,
                data=json.dumps(
                    npc_marker_message, ensure_ascii=False, separators=(",", ":")
                ).encode("utf-8"),
            )

        if tracked_vehicle_states is not None:
            tracked_vehicle_states_message = {
                "timestamp": timestamp,
                "episode_id": ego_state.episode_id,
                "frame_id": ego_state.frame_id,
                "elapsed_seconds": ego_state.elapsed_seconds,
                "vehicles": [
                    {
                        "actor_id": int(vehicle.actor_id),
                        "relation": str(vehicle.relation),
                        "lane_id": vehicle.lane_id,
                        "speed_mps": float(vehicle.speed_mps),
                        "longitudinal_distance_m": (
                            float(vehicle.longitudinal_distance_m)
                            if vehicle.longitudinal_distance_m is not None
                            else None
                        ),
                        "lateral_distance_m": (
                            float(vehicle.lateral_distance_m)
                            if vehicle.lateral_distance_m is not None
                            else None
                        ),
                        "is_ahead": bool(vehicle.is_ahead),
                        "pose": {
                            "x": float(vehicle.pose["x"]),
                            "y": float(vehicle.pose["y"]),
                            "yaw_deg": float(vehicle.pose["yaw_deg"]),
                        },
                    }
                    for vehicle in tracked_vehicle_states
                ],
            }
            self._writer.add_message(
                channel_id=self._tracked_vehicle_states_channel_id,
                log_time=log_time_ns,
                publish_time=log_time_ns,
                sequence=ego_state.frame_id,
                data=json.dumps(
                    tracked_vehicle_states_message, ensure_ascii=False, separators=(",", ":")
                ).encode("utf-8"),
            )

        if traffic_light_states is not None:
            traffic_light_observations_message = {
                "timestamp": timestamp,
                "episode_id": ego_state.episode_id,
                "frame_id": ego_state.frame_id,
                "elapsed_seconds": ego_state.elapsed_seconds,
                "lights": [
                    {
                        "actor_id": int(light.actor_id),
                        "state": str(light.state),
                        "affects_ego": bool(light.affects_ego),
                        "distance_m": float(light.distance_m),
                        "stop_line_distance_m": (
                            float(light.stop_line_distance_m)
                            if light.stop_line_distance_m is not None
                            else None
                        ),
                    }
                    for light in traffic_light_states
                ],
            }
            self._writer.add_message(
                channel_id=self._traffic_light_observations_channel_id,
                log_time=log_time_ns,
                publish_time=log_time_ns,
                sequence=ego_state.frame_id,
                data=json.dumps(
                    traffic_light_observations_message,
                    ensure_ascii=False,
                    separators=(",", ":"),
                ).encode("utf-8"),
            )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._writer.finish()
        self._stream.close()


class RotatingRouteLoopMcapWriter:
    def __init__(
        self,
        *,
        root_dir: Path,
        episode_id: str,
        route_name: str,
        town: str,
        weather: str,
        camera_width: int,
        camera_height: int,
        jpeg_quality: int = 85,
        segment_seconds: float = 600.0,
        topdown_map_asset: TopdownMapAsset | None = None,
    ) -> None:
        self.root_dir = root_dir
        self.root_dir.mkdir(parents=True, exist_ok=True)
        self.index_path = self.root_dir / "index.json"
        self._episode_id = episode_id
        self._route_name = route_name
        self._town = town
        self._weather = weather
        self._camera_width = camera_width
        self._camera_height = camera_height
        self._jpeg_quality = jpeg_quality
        self._segment_seconds = max(0.0, float(segment_seconds))
        self._topdown_map_asset = topdown_map_asset
        self._static_scene_timestamp_s: float | None = None
        self._static_scene_route_points: list[tuple[float, float, float]] | None = None
        self._static_scene_lane_centerlines: list[list[tuple[float, float, float]]] | None = None
        self._current_segment_index: int | None = None
        self._current_writer: RouteLoopMcapWriter | None = None
        self._segments: list[McapSegmentInfo] = []
        self._closed = False
        self._write_index()

    @property
    def segment_seconds(self) -> float:
        return self._segment_seconds

    @property
    def segments(self) -> list[McapSegmentInfo]:
        return list(self._segments)

    @property
    def current_segment(self) -> McapSegmentInfo | None:
        if self._current_segment_index is None:
            return None
        return self._segments[self._current_segment_index]

    def _segment_index_for_elapsed_seconds(self, elapsed_seconds: float) -> int:
        if self._segment_seconds <= 0.0:
            return 0
        return max(0, int(float(elapsed_seconds) // self._segment_seconds))

    def _segment_path(self, segment_index: int) -> Path:
        return self.root_dir / f"segment_{segment_index:04d}.mcap"

    def _write_index(self) -> None:
        payload = {
            "episode_id": self._episode_id,
            "route_name": self._route_name,
            "town": self._town,
            "weather": self._weather,
            "segment_seconds": self._segment_seconds,
            "segments": [
                {
                    "segment_index": segment.segment_index,
                    "path": segment.path.name,
                    "start_elapsed_seconds": round(segment.start_elapsed_seconds, 3),
                    "end_elapsed_seconds": (
                        round(segment.end_elapsed_seconds, 3)
                        if segment.end_elapsed_seconds is not None
                        else None
                    ),
                    "frame_count": segment.frame_count,
                }
                for segment in self._segments
            ],
        }
        self.index_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    def _open_segment(
        self, *, segment_index: int, timestamp_s: float, elapsed_seconds: float
    ) -> None:
        path = self._segment_path(segment_index)
        self._current_writer = RouteLoopMcapWriter(
            path=path,
            episode_id=self._episode_id,
            route_name=self._route_name,
            town=self._town,
            weather=self._weather,
            camera_width=self._camera_width,
            camera_height=self._camera_height,
            jpeg_quality=self._jpeg_quality,
            topdown_map_asset=self._topdown_map_asset,
        )
        self._current_segment_index = segment_index
        segment_info = McapSegmentInfo(
            segment_index=segment_index,
            path=path,
            start_elapsed_seconds=float(elapsed_seconds),
        )
        self._segments.append(segment_info)
        if (
            self._static_scene_timestamp_s is not None
            and self._static_scene_route_points is not None
            and self._static_scene_lane_centerlines is not None
        ):
            self._current_writer.write_static_scene(
                timestamp_s=timestamp_s,
                route_name=self._route_name,
                route_points=self._static_scene_route_points,
                lane_centerlines=self._static_scene_lane_centerlines,
            )
        self._write_index()

    def _close_current_segment(self) -> None:
        if self._current_writer is None or self._current_segment_index is None:
            return
        self._current_writer.close()
        self._current_writer = None
        self._current_segment_index = None
        self._write_index()

    def write_static_scene(
        self,
        *,
        timestamp_s: float,
        route_name: str,
        route_points: list[tuple[float, float, float]],
        lane_centerlines: list[list[tuple[float, float, float]]],
    ) -> None:
        self._static_scene_timestamp_s = float(timestamp_s)
        self._static_scene_route_points = route_points
        self._static_scene_lane_centerlines = lane_centerlines
        if self._current_writer is not None:
            self._current_writer.write_static_scene(
                timestamp_s=timestamp_s,
                route_name=route_name,
                route_points=route_points,
                lane_centerlines=lane_centerlines,
            )

    def write_frame(
        self,
        *,
        current_rgb: Any,
        ego_state: EgoStateSample,
        npc_vehicle_states: list[NPCVehicleStateSample] | None = None,
        tracked_vehicle_states: list[TrackedVehicleStateSample] | None = None,
        traffic_light_states: list[TrafficLightObservationSample] | None = None,
    ) -> McapSegmentInfo:
        segment_index = self._segment_index_for_elapsed_seconds(ego_state.elapsed_seconds)
        if self._current_segment_index != segment_index:
            self._close_current_segment()
            self._open_segment(
                segment_index=segment_index,
                timestamp_s=ego_state.timestamp_s,
                elapsed_seconds=ego_state.elapsed_seconds,
            )

        assert self._current_writer is not None
        assert self._current_segment_index is not None
        self._current_writer.write_frame(
            current_rgb=current_rgb,
            ego_state=ego_state,
            npc_vehicle_states=npc_vehicle_states,
            tracked_vehicle_states=tracked_vehicle_states,
            traffic_light_states=traffic_light_states,
        )
        segment = self._segments[self._current_segment_index]
        segment.end_elapsed_seconds = float(ego_state.elapsed_seconds)
        segment.frame_count += 1
        return segment

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._close_current_segment()
        self._write_index()
