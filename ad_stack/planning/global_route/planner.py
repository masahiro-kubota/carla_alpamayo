from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from libs.carla_utils import PlannedRoute, build_planned_route, load_route_config
from libs.project import PROJECT_ROOT


@dataclass(slots=True)
class ScenarioRoutePlanner:
    default_route_dir: Path = PROJECT_ROOT / "scenarios" / "routes"

    def resolve_route_config(self, route_id_or_path: str | Path) -> Path:
        candidate = Path(route_id_or_path)
        if candidate.suffix == ".json":
            return candidate if candidate.is_absolute() else PROJECT_ROOT / candidate
        return self.default_route_dir / f"{route_id_or_path}.json"

    def build_for_world(self, world_map: Any, route_id_or_path: str | Path) -> PlannedRoute:
        route_config_path = self.resolve_route_config(route_id_or_path)
        route_config = load_route_config(route_config_path)
        return build_planned_route(world_map, route_config)
