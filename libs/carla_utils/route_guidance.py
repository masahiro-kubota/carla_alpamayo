from __future__ import annotations

from dataclasses import dataclass
from itertools import pairwise
from math import cos, radians, sin
from typing import TYPE_CHECKING

from libs.project import PROJECT_ROOT

from .routes import PlannedRoute, build_planned_route, load_route_config

if TYPE_CHECKING:
    from collections.abc import Iterable
    from pathlib import Path

    import carla


@dataclass(slots=True)
class RouteGeometry:
    route_id: str
    town: str
    closed_loop: bool
    points: list[tuple[float, float]]
    cumulative_lengths_m: list[float]


def route_config_path_for_route_id(route_id: str) -> Path:
    return PROJECT_ROOT / "scenarios" / "routes" / f"{route_id}.json"


def route_geometry_from_planned_route(planned_route: PlannedRoute) -> RouteGeometry:
    cumulative_lengths_m: list[float] = [0.0]
    for (x1, y1), (x2, y2) in pairwise(planned_route.xy_points):
        cumulative_lengths_m.append(
            cumulative_lengths_m[-1] + (((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5)
        )
    return RouteGeometry(
        route_id=planned_route.config.name,
        town=planned_route.config.town,
        closed_loop=planned_route.config.closed_loop,
        points=list(planned_route.xy_points),
        cumulative_lengths_m=cumulative_lengths_m,
    )


def load_route_geometries_for_ids(
    route_ids: Iterable[str],
    *,
    host: str = "127.0.0.1",
    port: int = 2000,
) -> dict[str, RouteGeometry]:
    from .python_api import require_carla

    carla = require_carla()
    client = carla.Client(host, port)
    client.set_timeout(30.0)
    world = client.get_world()

    route_geometries: dict[str, RouteGeometry] = {}
    town_to_map: dict[str, carla.Map] = {}
    for route_id in sorted(set(route_ids)):
        config_path = route_config_path_for_route_id(route_id)
        if not config_path.exists():
            raise FileNotFoundError(
                f"Route config does not exist for route_id={route_id}: {config_path}"
            )
        route_config = load_route_config(config_path)
        world_map = town_to_map.get(route_config.town)
        if world_map is None:
            active_town = world.get_map().name.split("/")[-1]
            if active_town != route_config.town:
                world = client.load_world(route_config.town)
            world_map = world.get_map()
            town_to_map[route_config.town] = world_map
        planned_route = build_planned_route(world_map, route_config)
        route_geometries[route_id] = route_geometry_from_planned_route(planned_route)
    return route_geometries


def compute_local_target_point(
    route_geometry: RouteGeometry,
    *,
    vehicle_x: float,
    vehicle_y: float,
    vehicle_yaw_deg: float,
    lookahead_m: float,
    target_normalization_m: float,
    previous_route_index: int | None = None,
    search_window: int = 160,
    fallback_distance_m: float = 12.0,
) -> tuple[tuple[float, float], int]:
    if not route_geometry.points:
        raise ValueError(f"Route geometry for {route_geometry.route_id} has no points.")

    nearest_index = find_nearest_route_index(
        route_geometry.points,
        vehicle_x=vehicle_x,
        vehicle_y=vehicle_y,
        previous_index=previous_route_index,
        search_window=search_window,
        fallback_distance_m=fallback_distance_m,
    )
    target_index = advance_route_index(
        route_geometry, start_index=nearest_index, lookahead_m=lookahead_m
    )
    target_x, target_y = route_geometry.points[target_index]
    local_x, local_y = world_to_local_2d(
        target_x=target_x,
        target_y=target_y,
        origin_x=vehicle_x,
        origin_y=vehicle_y,
        yaw_deg=vehicle_yaw_deg,
    )
    scale = max(target_normalization_m, 1e-3)
    normalized = (
        max(-1.0, min(1.0, local_x / scale)),
        max(-1.0, min(1.0, local_y / scale)),
    )
    return normalized, nearest_index


def find_nearest_route_index(
    route_points: list[tuple[float, float]],
    *,
    vehicle_x: float,
    vehicle_y: float,
    previous_index: int | None,
    search_window: int,
    fallback_distance_m: float,
) -> int:
    if previous_index is None:
        return global_nearest_route_index(route_points, vehicle_x=vehicle_x, vehicle_y=vehicle_y)

    lower = max(0, previous_index - 20)
    upper = min(len(route_points), previous_index + search_window)
    candidate_indices = range(lower, upper)
    nearest_index, nearest_distance = min(
        (
            (index, squared_distance(route_points[index], vehicle_x=vehicle_x, vehicle_y=vehicle_y))
            for index in candidate_indices
        ),
        key=lambda item: item[1],
    )
    if nearest_distance <= fallback_distance_m**2:
        return nearest_index
    return global_nearest_route_index(route_points, vehicle_x=vehicle_x, vehicle_y=vehicle_y)


def global_nearest_route_index(
    route_points: list[tuple[float, float]],
    *,
    vehicle_x: float,
    vehicle_y: float,
) -> int:
    return min(
        range(len(route_points)),
        key=lambda index: squared_distance(
            route_points[index], vehicle_x=vehicle_x, vehicle_y=vehicle_y
        ),
    )


def advance_route_index(
    route_geometry: RouteGeometry, *, start_index: int, lookahead_m: float
) -> int:
    if lookahead_m <= 0.0:
        return start_index

    start_distance = route_geometry.cumulative_lengths_m[start_index]
    target_distance = start_distance + lookahead_m
    if not route_geometry.closed_loop:
        for index in range(start_index, len(route_geometry.cumulative_lengths_m)):
            if route_geometry.cumulative_lengths_m[index] >= target_distance:
                return index
        return len(route_geometry.points) - 1

    total_length = route_geometry.cumulative_lengths_m[-1]
    wrapped_target = target_distance % total_length if total_length > 0.0 else target_distance
    for index, cumulative_distance in enumerate(route_geometry.cumulative_lengths_m):
        if cumulative_distance >= wrapped_target:
            return index
    return len(route_geometry.points) - 1


def world_to_local_2d(
    *,
    target_x: float,
    target_y: float,
    origin_x: float,
    origin_y: float,
    yaw_deg: float,
) -> tuple[float, float]:
    dx = target_x - origin_x
    dy = target_y - origin_y
    yaw_rad = radians(yaw_deg)
    local_x = (cos(yaw_rad) * dx) + (sin(yaw_rad) * dy)
    local_y = (-sin(yaw_rad) * dx) + (cos(yaw_rad) * dy)
    return local_x, local_y


def squared_distance(
    point: tuple[float, float],
    *,
    vehicle_x: float,
    vehicle_y: float,
) -> float:
    dx = point[0] - vehicle_x
    dy = point[1] - vehicle_y
    return (dx * dx) + (dy * dy)
