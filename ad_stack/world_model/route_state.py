from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class RouteState:
    route_id: str
    maneuver: str = "lane_follow"
    progress_ratio: float = 0.0
    target_speed_mps: float | None = None
    target_steer: float = 0.0
    lateral_error_m: float = 0.0
    heading_error_deg: float = 0.0
