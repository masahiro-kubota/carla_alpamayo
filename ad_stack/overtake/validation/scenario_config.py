from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(slots=True)
class OvertakeScenarioConfig:
    scenario_kind: str
    obstacle_npc_index: int = 0
    blocker_npc_index: int | None = None
    route_aligned_adjacent_lane_available: bool | None = None
    nearest_signal_distance_m: float | None = None
    nearest_junction_distance_m: float | None = None


def parse_overtake_scenario_config(
    raw: dict[str, Any] | None,
) -> OvertakeScenarioConfig | None:
    if raw is None:
        return None
    return OvertakeScenarioConfig(
        scenario_kind=str(raw["scenario_kind"]),
        obstacle_npc_index=int(raw.get("obstacle_npc_index", 0)),
        blocker_npc_index=(
            int(raw["blocker_npc_index"]) if raw.get("blocker_npc_index") is not None else None
        ),
        route_aligned_adjacent_lane_available=(
            bool(raw["route_aligned_adjacent_lane_available"])
            if raw.get("route_aligned_adjacent_lane_available") is not None
            else None
        ),
        nearest_signal_distance_m=(
            float(raw["nearest_signal_distance_m"])
            if raw.get("nearest_signal_distance_m") is not None
            else None
        ),
        nearest_junction_distance_m=(
            float(raw["nearest_junction_distance_m"])
            if raw.get("nearest_junction_distance_m") is not None
            else None
        ),
    )
