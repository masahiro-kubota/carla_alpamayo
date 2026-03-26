from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(slots=True)
class ActorManagerConfig:
    npc_vehicle_count: int = 20
    npc_walker_count: int = 0
    traffic_manager_port: int = 8000
    seed: int = 7


@dataclass(slots=True)
class RuntimeActors:
    ego_vehicle: Any
    sensors: dict[str, Any] = field(default_factory=dict)
    npc_vehicles: tuple[Any, ...] = ()
    walkers: tuple[Any, ...] = ()


@dataclass(slots=True)
class ActorManager:
    config: ActorManagerConfig = field(default_factory=ActorManagerConfig)

    def actor_summary(self, actors: RuntimeActors) -> dict[str, int]:
        return {
            "npc_vehicle_count": len(actors.npc_vehicles),
            "npc_walker_count": len(actors.walkers),
            "sensor_count": len(actors.sensors),
        }
