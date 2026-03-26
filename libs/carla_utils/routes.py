from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import json
from math import hypot
from pathlib import Path
from typing import TYPE_CHECKING, Any

from libs.project import PROJECT_ROOT

from .python_api import ensure_carla_agents_on_path

DEFAULT_ROUTE_CONFIG_PATH = PROJECT_ROOT / "data_collection" / "configs" / "routes" / "town01_pilotnet_loop.json"

if TYPE_CHECKING:
    import carla


@dataclass(slots=True)
class RouteConfig:
    name: str
    town: str
    closed_loop: bool
    sampling_resolution_m: float
    anchor_spawn_indices: list[int]
    description: str


@dataclass(slots=True)
class PlannedRoute:
    config: RouteConfig
    trace: list[tuple[Any, Any]]
    anchor_transforms: list[Any]
    segment_pairs: list[tuple[int, int]]
    segment_summaries: list[dict[str, Any]]
    road_option_counts: dict[str, int]
    xy_points: list[tuple[float, float]]
    total_length_m: float


def load_route_config(path: Path) -> RouteConfig:
    with path.open("r", encoding="utf-8") as handle:
        raw = json.load(handle)
    return RouteConfig(
        name=raw["name"],
        town=raw["town"],
        closed_loop=raw.get("closed_loop", False),
        sampling_resolution_m=float(raw["sampling_resolution_m"]),
        anchor_spawn_indices=list(raw["anchor_spawn_indices"]),
        description=raw["description"],
    )


def road_option_name(option: Any) -> str:
    if option is None:
        return "void"
    if hasattr(option, "name"):
        return str(option.name).lower()
    return str(option).split(".")[-1].lower()


def build_planned_route(world_map: "carla.Map", route_config: RouteConfig) -> PlannedRoute:
    ensure_carla_agents_on_path()
    from agents.navigation.global_route_planner import GlobalRoutePlanner

    spawn_points = world_map.get_spawn_points()
    anchor_transforms = [spawn_points[index] for index in route_config.anchor_spawn_indices]
    segment_pairs = list(zip(route_config.anchor_spawn_indices[:-1], route_config.anchor_spawn_indices[1:]))
    if route_config.closed_loop:
        segment_pairs.append((route_config.anchor_spawn_indices[-1], route_config.anchor_spawn_indices[0]))

    planner = GlobalRoutePlanner(world_map, route_config.sampling_resolution_m)
    trace: list[tuple[Any, Any]] = []
    xy_points: list[tuple[float, float]] = []
    segment_summaries: list[dict[str, Any]] = []
    road_option_counts: Counter[str] = Counter()

    for start_index, end_index in segment_pairs:
        start_transform = spawn_points[start_index]
        end_transform = spawn_points[end_index]
        segment_trace = planner.trace_route(start_transform.location, end_transform.location)
        if not segment_trace:
            raise RuntimeError(f"Planner returned an empty route for segment {start_index} -> {end_index}.")

        if trace and _same_waypoint(trace[-1][0], segment_trace[0][0]):
            segment_trace = segment_trace[1:]

        route_points = waypoint_xy(segment_trace)
        if xy_points and route_points and xy_points[-1] == route_points[0]:
            route_points = route_points[1:]

        trace.extend(segment_trace)
        xy_points.extend(route_points)
        road_option_counts.update(road_option_name(option).upper() for _, option in segment_trace)
        segment_summaries.append(
            {
                "start_spawn_index": start_index,
                "end_spawn_index": end_index,
                "waypoint_count": len(segment_trace),
                "length_m": round(route_length(route_points), 2),
            }
        )

    return PlannedRoute(
        config=route_config,
        trace=trace,
        anchor_transforms=anchor_transforms,
        segment_pairs=segment_pairs,
        segment_summaries=segment_summaries,
        road_option_counts=dict(sorted(road_option_counts.items())),
        xy_points=xy_points,
        total_length_m=round(route_length(xy_points), 2),
    )


def waypoint_xy(route_trace: list[tuple[Any, Any]]) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for waypoint, _road_option in route_trace:
        location = waypoint.transform.location
        point = (location.x, location.y)
        if not points or point != points[-1]:
            points.append(point)
    return points


def route_length(points: list[tuple[float, float]]) -> float:
    return sum(
        hypot(x2 - x1, y2 - y1)
        for (x1, y1), (x2, y2) in zip(points[:-1], points[1:])
    )


def _same_waypoint(first: Any, second: Any, tolerance_m: float = 0.05) -> bool:
    first_location = first.transform.location
    second_location = second.transform.location
    return first_location.distance(second_location) <= tolerance_m
