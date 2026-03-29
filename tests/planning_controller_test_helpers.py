from __future__ import annotations

from ad_stack.overtake.domain.planning_models import (
    AdjacentLaneAvailability,
    PlanningScene,
    Pose3D,
    RouteBackbone,
    RouteTracePoint,
    TrackedTarget,
    TrafficLightObservation,
)


def make_straight_route_backbone(
    *,
    num_points: int = 120,
    lane_id: str = "15:-1",
    road_option: str = "straight",
) -> RouteBackbone:
    trace = tuple(
        RouteTracePoint(
            x=float(index),
            y=0.0,
            z=0.0,
            yaw_deg=0.0,
            lane_id=lane_id,
            road_option=road_option,  # type: ignore[arg-type]
        )
        for index in range(num_points)
    )
    return RouteBackbone(
        trace=trace,
        xy_points=tuple((point.x, point.y) for point in trace),
        progress_m=tuple(float(index) for index in range(num_points)),
        route_index_to_trace_index=tuple(range(num_points)),
        route_index_to_road_option=tuple(point.road_option for point in trace),
        route_index_to_lane_id=tuple(point.lane_id for point in trace),
    )


def make_scene(
    *,
    route_index: int = 0,
    route_progress_m: float = 0.0,
    current_lane_id: str = "15:-1",
    ego_speed_mps: float = 20.0 / 3.6,
    tracked_targets: tuple[TrackedTarget, ...] = (),
    traffic_lights: tuple[TrafficLightObservation, ...] = (),
    left_open: bool = True,
    right_open: bool = False,
) -> PlanningScene:
    return PlanningScene(
        ego_pose=Pose3D(x=float(route_index), y=0.0, z=0.0, yaw_deg=0.0),
        ego_speed_mps=ego_speed_mps,
        route_index=route_index,
        route_progress_m=route_progress_m,
        current_lane_id=current_lane_id,
        tracked_targets=tracked_targets,
        traffic_lights=traffic_lights,
        adjacent_lane_availability=AdjacentLaneAvailability(
            left_open=left_open,
            right_open=right_open,
        ),
    )
