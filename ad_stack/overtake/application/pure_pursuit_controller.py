from __future__ import annotations

import math
from dataclasses import dataclass

from ad_stack.overtake.domain.planning_models import Pose3D, Trajectory


def lookahead_distance_for_speed(ego_speed_mps: float) -> float:
    return max(4.0, min(12.0, 4.0 + 0.3 * ego_speed_mps))


def low_pass_steer(previous_value: float, raw_value: float, alpha: float) -> float:
    return alpha * raw_value + (1.0 - alpha) * previous_value


def rate_limit_steer(previous_value: float, requested_value: float, max_delta: float) -> float:
    if requested_value > previous_value + max_delta:
        return previous_value + max_delta
    if requested_value < previous_value - max_delta:
        return previous_value - max_delta
    return requested_value


def find_nearest_point_index(
    ego_pose: Pose3D,
    trajectory: Trajectory,
    *,
    start_index: int = 0,
) -> int:
    best_index = start_index
    best_distance = float("inf")
    for index in range(start_index, len(trajectory.points)):
        point = trajectory.points[index]
        distance = math.hypot(point.x - ego_pose.x, point.y - ego_pose.y)
        if distance < best_distance:
            best_index = index
            best_distance = distance
    return best_index


def find_lookahead_point_index(
    trajectory: Trajectory,
    *,
    nearest_index: int,
    lookahead_distance_m: float,
) -> int:
    accumulated = 0.0
    for index in range(nearest_index, len(trajectory.points) - 1):
        current = trajectory.points[index]
        next_point = trajectory.points[index + 1]
        accumulated += math.hypot(next_point.x - current.x, next_point.y - current.y)
        if accumulated >= lookahead_distance_m:
            return index + 1
    return len(trajectory.points) - 1


@dataclass(frozen=True, slots=True)
class PurePursuitStep:
    nearest_index: int
    lookahead_index: int
    lookahead_distance_m: float
    steer_raw: float
    steer_filtered: float
    steer_applied: float


@dataclass(slots=True)
class PurePursuitController:
    wheelbase_m: float = 2.8
    steer_alpha: float = 0.35
    max_steer_delta: float = 0.15
    previous_nearest_index: int = 0
    previous_filtered_steer: float = 0.0
    previous_applied_steer: float = 0.0

    def step(
        self,
        *,
        ego_pose: Pose3D,
        ego_speed_mps: float,
        trajectory: Trajectory,
        lookahead_distance_m: float | None = None,
    ) -> PurePursuitStep:
        resolved_lookahead = (
            lookahead_distance_m
            if lookahead_distance_m is not None
            else lookahead_distance_for_speed(ego_speed_mps)
        )
        nearest_index = find_nearest_point_index(
            ego_pose,
            trajectory,
            start_index=self.previous_nearest_index,
        )
        lookahead_index = find_lookahead_point_index(
            trajectory,
            nearest_index=nearest_index,
            lookahead_distance_m=resolved_lookahead,
        )
        target = trajectory.points[lookahead_index]
        steer_raw = _pure_pursuit_steer(
            ego_pose=ego_pose,
            target_x=target.x,
            target_y=target.y,
            lookahead_distance_m=max(resolved_lookahead, 1e-3),
            wheelbase_m=self.wheelbase_m,
        )
        steer_filtered = low_pass_steer(
            self.previous_filtered_steer,
            steer_raw,
            self.steer_alpha,
        )
        steer_applied = rate_limit_steer(
            self.previous_applied_steer,
            steer_filtered,
            self.max_steer_delta,
        )

        self.previous_nearest_index = nearest_index
        self.previous_filtered_steer = steer_filtered
        self.previous_applied_steer = steer_applied

        return PurePursuitStep(
            nearest_index=nearest_index,
            lookahead_index=lookahead_index,
            lookahead_distance_m=resolved_lookahead,
            steer_raw=steer_raw,
            steer_filtered=steer_filtered,
            steer_applied=steer_applied,
        )


def _pure_pursuit_steer(
    *,
    ego_pose: Pose3D,
    target_x: float,
    target_y: float,
    lookahead_distance_m: float,
    wheelbase_m: float,
) -> float:
    yaw_rad = math.radians(ego_pose.yaw_deg)
    heading_to_target = math.atan2(target_y - ego_pose.y, target_x - ego_pose.x)
    alpha = _wrap_angle(heading_to_target - yaw_rad)
    steer_angle_rad = math.atan2(2.0 * wheelbase_m * math.sin(alpha), lookahead_distance_m)
    # Normalize to a simple [-1, 1] steering command proxy.
    return max(-1.0, min(1.0, steer_angle_rad / 0.7))


def _wrap_angle(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi
