from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class LightColor(str, Enum):
    RED = "red"
    YELLOW = "yellow"
    GREEN = "green"
    UNKNOWN = "unknown"


@dataclass(slots=True)
class TrafficLightState:
    light_id: str
    color: LightColor
    distance_to_stop_line_m: float | None = None
    affecting_ego: bool = False
