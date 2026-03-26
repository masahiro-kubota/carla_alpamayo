from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(slots=True)
class EgoState:
    x_m: float
    y_m: float
    yaw_deg: float
    speed_mps: float
    lane_id: str | None = None
    speed_limit_mps: float | None = None
    adjacent_lanes_open: dict[str, bool] = field(default_factory=dict)
