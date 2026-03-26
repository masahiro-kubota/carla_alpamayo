from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class ActorKind(str, Enum):
    VEHICLE = "vehicle"
    WALKER = "walker"
    STATIC_OBSTACLE = "static_obstacle"
    UNKNOWN = "unknown"


@dataclass(slots=True)
class TrackedObject:
    actor_id: str
    kind: ActorKind
    x_m: float
    y_m: float
    yaw_deg: float
    speed_mps: float
    lane_id: str | None = None
    longitudinal_distance_m: float | None = None
    lateral_distance_m: float | None = None
    metadata: dict[str, str] = field(default_factory=dict)

    @property
    def is_ahead(self) -> bool:
        return self.longitudinal_distance_m is not None and self.longitudinal_distance_m >= 0.0
