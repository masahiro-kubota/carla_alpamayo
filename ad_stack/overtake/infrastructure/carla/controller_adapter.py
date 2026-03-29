from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from ad_stack.overtake.application.pure_pursuit_controller import PurePursuitController
from ad_stack.overtake.domain.planning_models import Pose3D, Trajectory


@dataclass(frozen=True, slots=True)
class TrajectoryTrackingResult:
    control: Any
    desired_speed_mps: float
    lookahead_distance_m: float
    lateral_error_m: float
    heading_error_deg: float
    controller_steer_raw: float
    controller_steer_applied: float
    nearest_index: int
    lookahead_index: int


def run_tracking_control(
    *,
    carla_module: Any,
    controller: PurePursuitController,
    ego_transform: Any,
    ego_speed_mps: float,
    trajectory: Trajectory,
) -> TrajectoryTrackingResult:
    ego_pose = Pose3D(
        x=float(ego_transform.location.x),
        y=float(ego_transform.location.y),
        z=float(ego_transform.location.z),
        yaw_deg=float(ego_transform.rotation.yaw),
    )
    controller_step = controller.step(
        ego_pose=ego_pose,
        ego_speed_mps=ego_speed_mps,
        trajectory=trajectory,
    )
    nearest_point = trajectory.points[controller_step.nearest_index]
    desired_speed_mps = nearest_point.longitudinal_velocity_mps
    lateral_error_m = _lateral_error_m(ego_pose=ego_pose, point=nearest_point)
    heading_error_deg = _heading_error_deg(
        target_yaw_deg=nearest_point.yaw_deg,
        ego_yaw_deg=ego_pose.yaw_deg,
    )
    control = carla_module.VehicleControl(
        throttle=0.0,
        steer=float(controller_step.steer_applied),
        brake=0.0,
        hand_brake=False,
        reverse=False,
        manual_gear_shift=False,
        gear=0,
    )
    return TrajectoryTrackingResult(
        control=control,
        desired_speed_mps=desired_speed_mps,
        lookahead_distance_m=controller_step.lookahead_distance_m,
        lateral_error_m=lateral_error_m,
        heading_error_deg=heading_error_deg,
        controller_steer_raw=controller_step.steer_raw,
        controller_steer_applied=controller_step.steer_applied,
        nearest_index=controller_step.nearest_index,
        lookahead_index=controller_step.lookahead_index,
    )


def _lateral_error_m(*, ego_pose: Pose3D, point: Any) -> float:
    yaw_rad = math.radians(ego_pose.yaw_deg)
    dx = point.x - ego_pose.x
    dy = point.y - ego_pose.y
    return (-math.sin(yaw_rad) * dx) + (math.cos(yaw_rad) * dy)


def _heading_error_deg(*, target_yaw_deg: float, ego_yaw_deg: float) -> float:
    wrapped = (target_yaw_deg - ego_yaw_deg + 180.0) % 360.0 - 180.0
    return float(wrapped)
