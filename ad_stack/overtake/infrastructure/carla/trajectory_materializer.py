from __future__ import annotations

from typing import Any

from ad_stack.overtake.application.trajectory_generation import (
    build_pose_trajectory,
    build_route_backbone_trajectory as build_pure_route_backbone_trajectory,
)
from ad_stack.overtake.domain.planning_models import Pose3D, RouteBackbone, Trajectory


def build_route_backbone_trajectory(
    *,
    route_backbone: RouteBackbone,
    start_route_index: int,
    desired_speed_mps: float,
    trajectory_id: str,
    horizon_m: float = 60.0,
    minimum_horizon_m: float = 30.0,
    resample_interval_m: float = 1.0,
    origin_lane_id: str | None = None,
    target_lane_id: str | None = None,
) -> Trajectory:
    return build_pure_route_backbone_trajectory(
        route_backbone=route_backbone,
        start_route_index=start_route_index,
        desired_speed_mps=desired_speed_mps,
        trajectory_id=trajectory_id,
        horizon_m=horizon_m,
        minimum_horizon_m=minimum_horizon_m,
        resample_interval_m=resample_interval_m,
        origin_lane_id=origin_lane_id,
        target_lane_id=target_lane_id,
    )


def build_waypoint_trajectory(
    *,
    waypoints: list[Any],
    desired_speed_mps: float,
    trajectory_id: str,
    origin_lane_id: str | None = None,
    target_lane_id: str | None = None,
    resample_interval_m: float = 1.0,
) -> Trajectory:
    return build_pose_trajectory(
        pose_samples=waypoints_to_pose_samples(waypoints),
        desired_speed_mps=desired_speed_mps,
        trajectory_id=trajectory_id,
        origin_lane_id=origin_lane_id,
        target_lane_id=target_lane_id,
        resample_interval_m=resample_interval_m,
    )


def waypoints_to_pose_samples(waypoints: list[Any]) -> tuple[Pose3D, ...]:
    if not waypoints:
        raise ValueError("waypoint trajectory requires at least one waypoint")
    return tuple(
        Pose3D(
            x=float(waypoint.transform.location.x),
            y=float(waypoint.transform.location.y),
            z=float(waypoint.transform.location.z),
            yaw_deg=float(waypoint.transform.rotation.yaw),
        )
        for waypoint in waypoints
    )
