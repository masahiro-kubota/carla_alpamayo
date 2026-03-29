from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from ad_stack.overtake.domain.planning_models import RouteBackbone, Trajectory, TrajectoryPoint


@dataclass(frozen=True, slots=True)
class _PoseSample:
    x: float
    y: float
    z: float
    yaw_deg: float


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
    bounded_start_index = max(0, min(start_route_index, len(route_backbone.trace) - 1))
    start_progress_m = route_backbone.progress_m[bounded_start_index]
    target_end_progress_m = min(
        route_backbone.progress_m[-1],
        start_progress_m + max(minimum_horizon_m, horizon_m),
    )
    sampled_progress_m = _sample_progresses(
        start_progress_m=start_progress_m,
        end_progress_m=target_end_progress_m,
        spacing_m=resample_interval_m,
    )
    samples = [
        _interpolate_route_backbone_pose(route_backbone, progress_m)
        for progress_m in sampled_progress_m
    ]
    points = _trajectory_points_from_samples(samples, desired_speed_mps=desired_speed_mps)
    return Trajectory(
        points=tuple(points),
        trajectory_id=trajectory_id,
        origin_lane_id=origin_lane_id,
        target_lane_id=target_lane_id,
        source_route_start_index=bounded_start_index,
        source_route_end_index=min(
            len(route_backbone.trace) - 1,
            _trace_index_at_progress(route_backbone, target_end_progress_m),
        ),
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
    if not waypoints:
        raise ValueError("waypoint trajectory requires at least one waypoint")
    raw_samples = [_pose_from_transform(waypoint.transform) for waypoint in waypoints]
    samples = _resample_pose_samples(raw_samples, spacing_m=resample_interval_m)
    points = _trajectory_points_from_samples(samples, desired_speed_mps=desired_speed_mps)
    return Trajectory(
        points=tuple(points),
        trajectory_id=trajectory_id,
        origin_lane_id=origin_lane_id,
        target_lane_id=target_lane_id,
    )


def _sample_progresses(*, start_progress_m: float, end_progress_m: float, spacing_m: float) -> list[float]:
    if end_progress_m <= start_progress_m:
        return [start_progress_m, start_progress_m + spacing_m]
    progresses = [start_progress_m]
    current_progress_m = start_progress_m
    while current_progress_m + spacing_m < end_progress_m - 1e-6:
        current_progress_m += spacing_m
        progresses.append(current_progress_m)
    if end_progress_m - progresses[-1] < 0.8 and len(progresses) >= 2:
        progresses[-1] = end_progress_m
    else:
        progresses.append(end_progress_m)
    if len(progresses) == 1:
        progresses.append(start_progress_m + spacing_m)
    return progresses


def _interpolate_route_backbone_pose(route_backbone: RouteBackbone, progress_m: float) -> _PoseSample:
    index = _trace_index_at_progress(route_backbone, progress_m)
    if index >= len(route_backbone.trace) - 1:
        trace_point = route_backbone.trace[-1]
        previous = route_backbone.trace[-2]
        yaw_deg = math.degrees(math.atan2(trace_point.y - previous.y, trace_point.x - previous.x))
        return _PoseSample(
            x=trace_point.x,
            y=trace_point.y,
            z=trace_point.z,
            yaw_deg=yaw_deg,
        )

    current = route_backbone.trace[index]
    next_point = route_backbone.trace[index + 1]
    start_progress_m = route_backbone.progress_m[index]
    end_progress_m = route_backbone.progress_m[index + 1]
    span_m = max(end_progress_m - start_progress_m, 1e-6)
    alpha = max(0.0, min(1.0, (progress_m - start_progress_m) / span_m))
    yaw_deg = math.degrees(math.atan2(next_point.y - current.y, next_point.x - current.x))
    return _PoseSample(
        x=(current.x * (1.0 - alpha)) + (next_point.x * alpha),
        y=(current.y * (1.0 - alpha)) + (next_point.y * alpha),
        z=(current.z * (1.0 - alpha)) + (next_point.z * alpha),
        yaw_deg=yaw_deg,
    )


def _trace_index_at_progress(route_backbone: RouteBackbone, progress_m: float) -> int:
    for index in range(len(route_backbone.progress_m) - 1):
        if route_backbone.progress_m[index + 1] >= progress_m:
            return index
    return len(route_backbone.progress_m) - 1


def _pose_from_transform(transform: Any) -> _PoseSample:
    return _PoseSample(
        x=float(transform.location.x),
        y=float(transform.location.y),
        z=float(transform.location.z),
        yaw_deg=float(transform.rotation.yaw),
    )


def _resample_pose_samples(samples: list[_PoseSample], *, spacing_m: float) -> list[_PoseSample]:
    if len(samples) == 1:
        sample = samples[0]
        yaw_rad = math.radians(sample.yaw_deg)
        return [
            sample,
            _PoseSample(
                x=sample.x + (spacing_m * math.cos(yaw_rad)),
                y=sample.y + (spacing_m * math.sin(yaw_rad)),
                z=sample.z,
                yaw_deg=sample.yaw_deg,
            ),
        ]

    cumulative_distances_m = [0.0]
    for previous, current in zip(samples, samples[1:]):
        cumulative_distances_m.append(
            cumulative_distances_m[-1] + math.hypot(current.x - previous.x, current.y - previous.y)
        )
    sample_progresses_m = _sample_progresses(
        start_progress_m=0.0,
        end_progress_m=cumulative_distances_m[-1],
        spacing_m=spacing_m,
    )
    resampled: list[_PoseSample] = []
    segment_index = 0
    for progress_m in sample_progresses_m:
        while (
            segment_index < len(cumulative_distances_m) - 2
            and cumulative_distances_m[segment_index + 1] < progress_m
        ):
            segment_index += 1
        start = samples[segment_index]
        end = samples[min(segment_index + 1, len(samples) - 1)]
        start_progress_m = cumulative_distances_m[segment_index]
        end_progress_m = cumulative_distances_m[min(segment_index + 1, len(samples) - 1)]
        span_m = max(end_progress_m - start_progress_m, 1e-6)
        alpha = max(0.0, min(1.0, (progress_m - start_progress_m) / span_m))
        yaw_deg = math.degrees(math.atan2(end.y - start.y, end.x - start.x))
        resampled.append(
            _PoseSample(
                x=(start.x * (1.0 - alpha)) + (end.x * alpha),
                y=(start.y * (1.0 - alpha)) + (end.y * alpha),
                z=(start.z * (1.0 - alpha)) + (end.z * alpha),
                yaw_deg=yaw_deg,
            )
        )
    return resampled


def _trajectory_points_from_samples(
    samples: list[_PoseSample],
    *,
    desired_speed_mps: float,
) -> list[TrajectoryPoint]:
    points: list[TrajectoryPoint] = []
    for index, sample in enumerate(samples):
        yaw_deg = sample.yaw_deg
        if index == len(samples) - 1 and index > 0:
            previous = samples[index - 1]
            yaw_deg = math.degrees(math.atan2(sample.y - previous.y, sample.x - previous.x))
        points.append(
            TrajectoryPoint(
                x=sample.x,
                y=sample.y,
                z=sample.z,
                yaw_deg=yaw_deg,
                longitudinal_velocity_mps=max(0.0, desired_speed_mps),
            )
        )
    return points
