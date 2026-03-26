from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(slots=True)
class EgoState:
    x_m: float
    y_m: float
    yaw_deg: float
    speed_mps: float
    lane_id: str | None = None
    speed_limit_mps: float | None = None
    adjacent_lanes_open: dict[str, bool] = field(default_factory=dict)


@dataclass(slots=True)
class RouteState:
    route_id: str
    maneuver: str = "lane_follow"
    progress_ratio: float = 0.0
    target_speed_mps: float | None = None
    target_steer: float = 0.0
    lateral_error_m: float = 0.0
    heading_error_deg: float = 0.0


@dataclass(slots=True)
class SceneState:
    timestamp_s: float
    town_id: str
    ego: EgoState
    route: RouteState
    tracked_objects: tuple[object, ...] = field(default_factory=tuple)
    traffic_lights: tuple[object, ...] = field(default_factory=tuple)
    metadata: dict[str, Any] = field(default_factory=dict)
