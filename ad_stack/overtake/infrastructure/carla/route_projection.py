from __future__ import annotations

import math
from typing import Any

from ad_stack.overtake.domain import OvertakeLeadSnapshot

from .route_alignment import lane_id


def route_relative_progress_to_actor(
    *,
    actor: Any,
    reference_route_index: int,
    base_trace: list[tuple[Any, Any]],
    route_point_to_trace_index: list[int],
    route_point_progress_m: list[float],
    search_back_points: int = 24,
    search_forward_points: int = 120,
) -> tuple[float | None, float, str | None]:
    if not route_point_to_trace_index:
        return None, float("inf"), None
    bounded_reference_index = max(0, min(reference_route_index, len(route_point_to_trace_index) - 1))
    start_index = max(0, bounded_reference_index - search_back_points)
    end_index = min(len(route_point_to_trace_index), bounded_reference_index + search_forward_points + 1)
    best_route_index: int | None = None
    best_distance_m = float("inf")
    for route_point_index in range(start_index, end_index):
        trace_index = route_point_to_trace_index[route_point_index]
        route_location = base_trace[trace_index][0].transform.location
        distance_m = math.hypot(route_location.x - actor.x_m, route_location.y - actor.y_m)
        if distance_m < best_distance_m:
            best_distance_m = distance_m
            best_route_index = route_point_index
    if best_route_index is None:
        return None, best_distance_m, None
    trace_index = route_point_to_trace_index[best_route_index]
    route_lane_id = lane_id(base_trace[trace_index][0])
    relative_progress_m = (
        route_point_progress_m[best_route_index] - route_point_progress_m[bounded_reference_index]
    )
    return float(relative_progress_m), best_distance_m, route_lane_id


def build_route_aligned_target_candidates(
    tracked_objects: tuple[Any, ...],
    *,
    route_index: int | None,
    base_trace: list[tuple[Any, Any]],
    route_point_to_trace_index: list[int],
    route_point_progress_m: list[float],
    stopped_speed_threshold_mps: float,
) -> list[OvertakeLeadSnapshot]:
    if route_index is None or not base_trace or not route_point_to_trace_index or not route_point_progress_m:
        return []

    bounded_route_index = max(0, min(route_index, len(route_point_to_trace_index) - 1))
    leads: list[OvertakeLeadSnapshot] = []
    for actor in tracked_objects:
        if actor.actor_id is None or actor.lane_id is None:
            continue
        route_distance_m, route_distance_to_centerline_m, route_lane_id = route_relative_progress_to_actor(
            actor=actor,
            reference_route_index=bounded_route_index,
            base_trace=base_trace,
            route_point_to_trace_index=route_point_to_trace_index,
            route_point_progress_m=route_point_progress_m,
            search_back_points=0,
        )
        if route_distance_m is None or route_distance_to_centerline_m > 4.0:
            continue
        if route_lane_id != actor.lane_id:
            continue
        if route_distance_m <= 0.0:
            continue
        leads.append(
            OvertakeLeadSnapshot(
                actor_id=actor.actor_id,
                lane_id=actor.lane_id,
                distance_m=float(route_distance_m),
                speed_mps=float(actor.speed_mps),
                relative_speed_mps=0.0,
                is_stopped=float(actor.speed_mps) <= stopped_speed_threshold_mps,
            )
        )
    return leads
