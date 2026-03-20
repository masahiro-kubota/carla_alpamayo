from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Any


@dataclass(slots=True)
class EpisodeRecord:
    episode_id: str
    frame_id: int
    town_id: str
    route_id: str
    weather_id: str
    timestamp: float
    front_rgb_path: str
    speed: float
    command: str
    steer: float
    throttle: float
    brake: float
    collision: bool
    lane_invasion: bool
    success: bool

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False)


def append_jsonl(path: Path, record: EpisodeRecord) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(record.to_json())
        handle.write("\n")
