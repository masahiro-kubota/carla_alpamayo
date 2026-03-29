from __future__ import annotations

import math
from typing import Any

from ad_stack.overtake.domain.planning_models import RouteBackbone, RouteCommand, RouteTracePoint
from libs.carla_utils import road_option_name


def build_route_backbone(trace: list[tuple[Any, Any]]) -> RouteBackbone:
    if len(trace) < 2:
        raise ValueError("route trace requires at least two waypoints")

    route_trace: list[RouteTracePoint] = []
    xy_points: list[tuple[float, float]] = []
    progress_m: list[float] = []
    route_index_to_trace_index: list[int] = []
    route_index_to_road_option: list[RouteCommand] = []
    route_index_to_lane_id: list[str] = []

    previous_xy: tuple[float, float] | None = None
    cumulative_progress_m = 0.0
    for waypoint, option in trace:
        location = waypoint.transform.location
        point_xy = (float(location.x), float(location.y))
        if previous_xy is not None and math.hypot(point_xy[0] - previous_xy[0], point_xy[1] - previous_xy[1]) <= 0.05:
            continue
        if previous_xy is not None:
            cumulative_progress_m += math.hypot(
                point_xy[0] - previous_xy[0],
                point_xy[1] - previous_xy[1],
            )
        previous_xy = point_xy
        route_command = normalize_route_command(option)
        lane_identifier = f"{waypoint.road_id}:{waypoint.lane_id}"
        route_trace.append(
            RouteTracePoint(
                x=float(location.x),
                y=float(location.y),
                z=float(location.z),
                yaw_deg=float(waypoint.transform.rotation.yaw),
                lane_id=lane_identifier,
                road_option=route_command,
            )
        )
        xy_points.append(point_xy)
        progress_m.append(cumulative_progress_m)
        trace_index = len(route_trace) - 1
        route_index_to_trace_index.append(trace_index)
        route_index_to_road_option.append(route_command)
        route_index_to_lane_id.append(lane_identifier)

    return RouteBackbone(
        trace=tuple(route_trace),
        xy_points=tuple(xy_points),
        progress_m=tuple(progress_m),
        route_index_to_trace_index=tuple(route_index_to_trace_index),
        route_index_to_road_option=tuple(route_index_to_road_option),
        route_index_to_lane_id=tuple(route_index_to_lane_id),
    )


def normalize_route_command(option: Any) -> RouteCommand:
    normalized = road_option_name(option).replace("_", "").lower()
    if normalized in {"straight"}:
        return "straight"
    if normalized in {"left"}:
        return "left"
    if normalized in {"right"}:
        return "right"
    return "lane_follow"
