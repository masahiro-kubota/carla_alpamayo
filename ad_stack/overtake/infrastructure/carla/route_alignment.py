from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal

from ad_stack.overtake.domain import LaneChangePlanPoint


def lane_id(waypoint: Any | None) -> str | None:
    if waypoint is None:
        return None
    return f"{waypoint.road_id}:{waypoint.lane_id}"


@dataclass(slots=True)
class RouteAlignedWaypoint:
    transform: Any


def adjacent_lane_waypoint(
    carla_module: Any,
    origin_waypoint: Any,
    direction: Literal["left", "right"],
) -> Any | None:
    adjacent_waypoint = (
        origin_waypoint.get_left_lane() if direction == "left" else origin_waypoint.get_right_lane()
    )
    if adjacent_waypoint is None:
        return None
    if adjacent_waypoint.lane_type != carla_module.LaneType.Driving:
        return None
    return adjacent_waypoint


def route_aligned_waypoint(carla_module: Any, waypoint: Any, heading_source_waypoint: Any) -> Any:
    location = waypoint.transform.location
    rotation = heading_source_waypoint.transform.rotation
    transform = carla_module.Transform(
        carla_module.Location(x=location.x, y=location.y, z=location.z),
        carla_module.Rotation(
            pitch=rotation.pitch,
            yaw=rotation.yaw,
            roll=rotation.roll,
        ),
    )
    return RouteAlignedWaypoint(transform=transform)


def interpolate_waypoint(
    carla_module: Any,
    start_waypoint: Any,
    end_waypoint: Any,
    alpha: float,
) -> Any:
    start_location = start_waypoint.transform.location
    end_location = end_waypoint.transform.location
    rotation = start_waypoint.transform.rotation
    transform = carla_module.Transform(
        carla_module.Location(
            x=(start_location.x * (1.0 - alpha)) + (end_location.x * alpha),
            y=(start_location.y * (1.0 - alpha)) + (end_location.y * alpha),
            z=(start_location.z * (1.0 - alpha)) + (end_location.z * alpha),
        ),
        carla_module.Rotation(
            pitch=rotation.pitch,
            yaw=rotation.yaw,
            roll=rotation.roll,
        ),
    )
    return RouteAlignedWaypoint(transform=transform)


def build_route_aligned_lane_samples(
    *,
    carla_module: Any,
    direction: Literal["left", "right"],
    route_index: int | None,
    base_trace: list[tuple[Any, Any]],
    route_point_to_trace_index: list[int],
) -> tuple[list[LaneChangePlanPoint], list[LaneChangePlanPoint], dict[tuple[int, str], Any], str | None]:
    origin_samples: list[LaneChangePlanPoint] = []
    target_samples: list[LaneChangePlanPoint] = []
    waypoint_lookup: dict[tuple[int, str], Any] = {}
    target_lane_id: str | None = None

    start_point_index = max(0, route_index or 0)
    if start_point_index >= len(route_point_to_trace_index):
        return origin_samples, target_samples, waypoint_lookup, target_lane_id

    previous_location: Any | None = None
    progress_m = 0.0
    for point_index in range(start_point_index, len(route_point_to_trace_index)):
        trace_index = route_point_to_trace_index[point_index]
        origin_waypoint = base_trace[trace_index][0]
        origin_location = origin_waypoint.transform.location
        if previous_location is not None:
            progress_m += origin_location.distance(previous_location)
        previous_location = origin_location

        origin_lane = lane_id(origin_waypoint)
        if origin_lane is None:
            continue
        origin_samples.append(
            LaneChangePlanPoint(
                route_index=point_index,
                lane_id=origin_lane,
                progress_m=progress_m,
            )
        )
        waypoint_lookup[(point_index, origin_lane)] = route_aligned_waypoint(
            carla_module, origin_waypoint, origin_waypoint
        )

        adjacent_waypoint = adjacent_lane_waypoint(carla_module, origin_waypoint, direction)
        adjacent_lane = lane_id(adjacent_waypoint)
        if adjacent_waypoint is None or adjacent_lane is None:
            continue
        if target_lane_id is None:
            target_lane_id = adjacent_lane
        target_samples.append(
            LaneChangePlanPoint(
                route_index=point_index,
                lane_id=adjacent_lane,
                progress_m=progress_m,
            )
        )
        waypoint_lookup[(point_index, adjacent_lane)] = route_aligned_waypoint(
            carla_module, adjacent_waypoint, origin_waypoint
        )

    return origin_samples, target_samples, waypoint_lookup, target_lane_id


def materialize_lane_change_waypoints(
    *,
    carla_module: Any,
    start_samples: list[LaneChangePlanPoint],
    destination_samples: list[LaneChangePlanPoint],
    waypoint_lookup: dict[tuple[int, str], Any],
    distance_same_lane_m: float,
    lane_change_distance_m: float,
    distance_other_lane_m: float,
) -> list[Any]:
    if not start_samples or not destination_samples:
        return []

    start_progress_m = start_samples[0].progress_m
    same_lane_end_m = start_progress_m + distance_same_lane_m
    lane_change_end_m = same_lane_end_m + lane_change_distance_m
    destination_lane_end_m = lane_change_end_m + distance_other_lane_m
    destination_by_route_index = {sample.route_index: sample for sample in destination_samples}
    materialized: list[Any] = []
    for point in start_samples:
        if point.progress_m > destination_lane_end_m + 1e-6:
            break
        start_waypoint = waypoint_lookup.get((point.route_index, point.lane_id))
        if start_waypoint is None:
            continue

        destination_point = destination_by_route_index.get(point.route_index)
        destination_waypoint = (
            waypoint_lookup.get((destination_point.route_index, destination_point.lane_id))
            if destination_point is not None
            else None
        )
        if point.progress_m <= same_lane_end_m + 1e-6 or destination_waypoint is None:
            waypoint = start_waypoint
        elif point.progress_m < lane_change_end_m - 1e-6:
            alpha = (point.progress_m - same_lane_end_m) / max(lane_change_distance_m, 1e-6)
            waypoint = interpolate_waypoint(
                carla_module,
                start_waypoint,
                destination_waypoint,
                max(0.0, min(1.0, alpha)),
            )
        else:
            waypoint = destination_waypoint

        if materialized:
            tail_location = materialized[-1].transform.location
            if tail_location.distance(waypoint.transform.location) <= 0.05:
                continue
        materialized.append(waypoint)
    return materialized
