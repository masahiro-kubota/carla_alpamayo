from __future__ import annotations

import math
from dataclasses import dataclass

from ad_stack.overtake.domain.planning_models import Pose3D, RouteBackbone, RouteTracePoint, Trajectory, TrajectoryPoint


@dataclass(frozen=True, slots=True)
class TrajectoryGenerationConfig:
    horizon_m: float = 60.0
    minimum_horizon_m: float = 30.0
    resample_interval_m: float = 1.0


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
    bounded_start_index = max(0, min(start_route_index, len(route_backbone.route_index_to_trace_index) - 1))
    points, end_route_index = sample_route_trace_points(
        route_backbone,
        start_route_index=bounded_start_index,
        horizon_m=horizon_m,
        minimum_horizon_m=minimum_horizon_m,
        resample_interval_m=resample_interval_m,
    )
    return build_pose_trajectory(
        pose_samples=tuple(
            Pose3D(x=point.x, y=point.y, z=point.z, yaw_deg=point.yaw_deg)
            for point in points
        ),
        desired_speed_mps=desired_speed_mps,
        trajectory_id=trajectory_id,
        origin_lane_id=origin_lane_id,
        target_lane_id=target_lane_id,
        source_route_start_index=bounded_start_index,
        source_route_end_index=end_route_index,
    )


def build_signal_stop_trajectory(
    *,
    route_backbone: RouteBackbone,
    start_route_index: int,
    desired_speed_mps: float,
    stop_line_distance_m: float,
    trajectory_id: str,
    horizon_m: float = 60.0,
    minimum_horizon_m: float = 30.0,
    resample_interval_m: float = 1.0,
    origin_lane_id: str | None = None,
    target_lane_id: str | None = None,
) -> Trajectory:
    bounded_start_index = max(0, min(start_route_index, len(route_backbone.route_index_to_trace_index) - 1))
    points, end_route_index = sample_route_trace_points(
        route_backbone,
        start_route_index=bounded_start_index,
        horizon_m=horizon_m,
        minimum_horizon_m=minimum_horizon_m,
        resample_interval_m=resample_interval_m,
    )
    trajectory_points: list[TrajectoryPoint] = []
    for index, point in enumerate(points):
        distance_from_start_m = index * resample_interval_m
        if distance_from_start_m >= stop_line_distance_m:
            speed_mps = 0.0
        else:
            remaining_ratio = max(
                0.0,
                (stop_line_distance_m - distance_from_start_m) / max(stop_line_distance_m, 1e-3),
            )
            speed_mps = desired_speed_mps * remaining_ratio
        trajectory_points.append(
            TrajectoryPoint(
                x=point.x,
                y=point.y,
                z=point.z,
                yaw_deg=point.yaw_deg,
                longitudinal_velocity_mps=speed_mps,
            )
        )
    return Trajectory(
        points=tuple(trajectory_points),
        trajectory_id=trajectory_id,
        origin_lane_id=origin_lane_id,
        target_lane_id=target_lane_id,
        source_route_start_index=bounded_start_index,
        source_route_end_index=end_route_index,
    )


def build_pose_trajectory(
    *,
    pose_samples: tuple[Pose3D, ...],
    desired_speed_mps: float,
    trajectory_id: str,
    origin_lane_id: str | None = None,
    target_lane_id: str | None = None,
    source_route_start_index: int | None = None,
    source_route_end_index: int | None = None,
    resample_interval_m: float = 1.0,
) -> Trajectory:
    resampled_samples = resample_pose_samples(
        pose_samples,
        spacing_m=resample_interval_m,
    )
    normalized_yaws_deg = _trajectory_yaws_from_pose_samples(resampled_samples)
    return Trajectory(
        points=tuple(
            TrajectoryPoint(
                x=sample.x,
                y=sample.y,
                z=sample.z,
                yaw_deg=normalized_yaws_deg[index],
                longitudinal_velocity_mps=max(0.0, desired_speed_mps),
            )
            for index, sample in enumerate(resampled_samples)
        ),
        trajectory_id=trajectory_id,
        origin_lane_id=origin_lane_id,
        target_lane_id=target_lane_id,
        source_route_start_index=source_route_start_index,
        source_route_end_index=source_route_end_index,
    )


def sample_route_trace_points(
    route_backbone: RouteBackbone,
    *,
    start_route_index: int,
    horizon_m: float,
    minimum_horizon_m: float,
    resample_interval_m: float,
) -> tuple[list[RouteTracePoint], int]:
    start_trace_index = route_backbone.trace_index_for_route_index(start_route_index)
    start_progress_m = route_backbone.progress_m[start_trace_index]
    remaining_horizon_m = route_backbone.progress_m[-1] - start_progress_m
    target_horizon_m = min(horizon_m, remaining_horizon_m)
    if target_horizon_m < minimum_horizon_m and remaining_horizon_m >= minimum_horizon_m:
        target_horizon_m = minimum_horizon_m
    end_progress_m = min(route_backbone.progress_m[-1], start_progress_m + target_horizon_m)
    sampled_progresses_m = sample_progresses(
        start_progress_m=start_progress_m,
        end_progress_m=end_progress_m,
        spacing_m=resample_interval_m,
    )
    sampled_points = [
        interpolate_route_trace(route_backbone, progress_m)
        for progress_m in sampled_progresses_m
    ]
    end_trace_index = find_trace_index_at_progress(route_backbone, sampled_progresses_m[-1])
    end_route_index = start_route_index
    for route_index, trace_index in enumerate(route_backbone.route_index_to_trace_index):
        if trace_index <= end_trace_index:
            end_route_index = route_index
    return sampled_points, end_route_index


def resample_pose_samples(
    pose_samples: tuple[Pose3D, ...],
    *,
    spacing_m: float,
) -> tuple[Pose3D, ...]:
    if len(pose_samples) == 0:
        raise ValueError("pose trajectory requires at least one pose sample")
    if len(pose_samples) == 1:
        sample = pose_samples[0]
        yaw_rad = math.radians(sample.yaw_deg)
        return (
            sample,
            Pose3D(
                x=sample.x + (spacing_m * math.cos(yaw_rad)),
                y=sample.y + (spacing_m * math.sin(yaw_rad)),
                z=sample.z,
                yaw_deg=sample.yaw_deg,
            ),
        )

    cumulative_distances_m = [0.0]
    for previous, current in zip(pose_samples, pose_samples[1:]):
        cumulative_distances_m.append(
            cumulative_distances_m[-1] + math.hypot(current.x - previous.x, current.y - previous.y)
        )
    total_length_m = cumulative_distances_m[-1]
    if total_length_m < spacing_m:
        final_sample = pose_samples[-1]
        previous_sample = pose_samples[-2]
        yaw_deg = math.degrees(
            math.atan2(final_sample.y - previous_sample.y, final_sample.x - previous_sample.x)
        )
        yaw_rad = math.radians(yaw_deg)
        return (
            pose_samples[0],
            Pose3D(
                x=pose_samples[0].x + (spacing_m * math.cos(yaw_rad)),
                y=pose_samples[0].y + (spacing_m * math.sin(yaw_rad)),
                z=pose_samples[0].z,
                yaw_deg=yaw_deg,
            ),
        )

    sample_progresses_m = sample_progresses(
        start_progress_m=0.0,
        end_progress_m=total_length_m,
        spacing_m=spacing_m,
    )
    resampled: list[Pose3D] = []
    segment_index = 0
    for progress_m in sample_progresses_m:
        while (
            segment_index < len(cumulative_distances_m) - 2
            and cumulative_distances_m[segment_index + 1] < progress_m
        ):
            segment_index += 1
        start = pose_samples[segment_index]
        end = pose_samples[min(segment_index + 1, len(pose_samples) - 1)]
        start_progress = cumulative_distances_m[segment_index]
        end_progress = cumulative_distances_m[min(segment_index + 1, len(pose_samples) - 1)]
        span_m = max(end_progress - start_progress, 1e-6)
        alpha = max(0.0, min(1.0, (progress_m - start_progress) / span_m))
        yaw_deg = math.degrees(math.atan2(end.y - start.y, end.x - start.x))
        resampled.append(
            Pose3D(
                x=(start.x * (1.0 - alpha)) + (end.x * alpha),
                y=(start.y * (1.0 - alpha)) + (end.y * alpha),
                z=(start.z * (1.0 - alpha)) + (end.z * alpha),
                yaw_deg=yaw_deg,
            )
        )
    return tuple(resampled)


def sample_progresses(
    *,
    start_progress_m: float,
    end_progress_m: float,
    spacing_m: float,
) -> list[float]:
    if end_progress_m <= start_progress_m:
        return [start_progress_m, start_progress_m + spacing_m]
    progresses = [start_progress_m]
    current_progress_m = start_progress_m
    while current_progress_m + spacing_m <= end_progress_m + 1e-6:
        current_progress_m += spacing_m
        progresses.append(current_progress_m)
    remainder_m = end_progress_m - progresses[-1]
    if remainder_m >= 0.8:
        progresses.append(end_progress_m)
    if len(progresses) == 1:
        progresses.append(start_progress_m + spacing_m)
    return progresses


def interpolate_route_trace(route_backbone: RouteBackbone, progress_m: float) -> RouteTracePoint:
    trace = route_backbone.trace
    progress = route_backbone.progress_m
    if progress_m <= progress[0]:
        return trace[0]
    if progress_m >= progress[-1]:
        return trace[-1]
    for index in range(len(progress) - 1):
        start_progress = progress[index]
        end_progress = progress[index + 1]
        if start_progress <= progress_m <= end_progress:
            span_m = max(end_progress - start_progress, 1e-6)
            ratio = (progress_m - start_progress) / span_m
            start = trace[index]
            end = trace[index + 1]
            return RouteTracePoint(
                x=lerp(start.x, end.x, ratio),
                y=lerp(start.y, end.y, ratio),
                z=lerp(start.z, end.z, ratio),
                yaw_deg=interpolate_yaw(start.yaw_deg, end.yaw_deg, ratio),
                lane_id=start.lane_id,
                road_option=start.road_option,
            )
    return trace[-1]


def find_trace_index_at_progress(route_backbone: RouteBackbone, progress_m: float) -> int:
    for index, value in enumerate(route_backbone.progress_m):
        if value >= progress_m:
            return index
    return len(route_backbone.progress_m) - 1


def lerp(a: float, b: float, ratio: float) -> float:
    return a + (b - a) * ratio


def interpolate_yaw(a_deg: float, b_deg: float, ratio: float) -> float:
    a_rad = math.radians(a_deg)
    b_rad = math.radians(b_deg)
    delta = (b_rad - a_rad + math.pi) % (2.0 * math.pi) - math.pi
    return math.degrees(a_rad + delta * ratio)


def _trajectory_yaws_from_pose_samples(samples: tuple[Pose3D, ...]) -> tuple[float, ...]:
    if len(samples) < 2:
        return tuple(sample.yaw_deg for sample in samples)
    yaws_deg: list[float] = []
    for index, sample in enumerate(samples):
        if index < len(samples) - 1:
            next_sample = samples[index + 1]
            yaw_deg = math.degrees(math.atan2(next_sample.y - sample.y, next_sample.x - sample.x))
        else:
            previous_sample = samples[index - 1]
            yaw_deg = math.degrees(math.atan2(sample.y - previous_sample.y, sample.x - previous_sample.x))
        yaws_deg.append(yaw_deg)
    return tuple(yaws_deg)
