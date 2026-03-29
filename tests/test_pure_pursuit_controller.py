from __future__ import annotations

import unittest

from ad_stack.overtake.application.pure_pursuit_controller import (
    PurePursuitController,
    find_lookahead_point_index,
    find_nearest_point_index,
    lookahead_distance_for_speed,
)
from ad_stack.overtake.domain.planning_models import Pose3D, Trajectory, TrajectoryPoint


def _straight_trajectory() -> Trajectory:
    return Trajectory(
        points=tuple(
            TrajectoryPoint(
                x=float(index),
                y=0.0,
                z=0.0,
                yaw_deg=0.0,
                longitudinal_velocity_mps=5.0,
            )
            for index in range(20)
        ),
        trajectory_id="straight",
    )


def _left_curve_trajectory() -> Trajectory:
    return Trajectory(
        points=(
            TrajectoryPoint(x=0.0, y=0.0, z=0.0, yaw_deg=0.0, longitudinal_velocity_mps=5.0),
            TrajectoryPoint(x=1.0, y=0.0, z=0.0, yaw_deg=20.0, longitudinal_velocity_mps=5.0),
            TrajectoryPoint(x=1.9, y=0.4, z=0.0, yaw_deg=35.0, longitudinal_velocity_mps=5.0),
            TrajectoryPoint(x=2.6, y=1.1, z=0.0, yaw_deg=50.0, longitudinal_velocity_mps=5.0),
            TrajectoryPoint(x=3.1, y=2.0, z=0.0, yaw_deg=60.0, longitudinal_velocity_mps=5.0),
        ),
        trajectory_id="curve",
    )


class PurePursuitControllerTests(unittest.TestCase):
    def test_lookahead_distance_uses_documented_rule(self) -> None:
        self.assertEqual(lookahead_distance_for_speed(0.0), 4.0)
        self.assertAlmostEqual(lookahead_distance_for_speed(10.0), 7.0)
        self.assertEqual(lookahead_distance_for_speed(40.0), 12.0)

    def test_nearest_and_lookahead_indices_are_computed_internally(self) -> None:
        trajectory = _straight_trajectory()
        ego_pose = Pose3D(x=0.2, y=0.0, z=0.0, yaw_deg=0.0)

        nearest_index = find_nearest_point_index(ego_pose, trajectory)
        lookahead_index = find_lookahead_point_index(
            trajectory,
            nearest_index=nearest_index,
            lookahead_distance_m=4.0,
        )

        self.assertEqual(nearest_index, 0)
        self.assertEqual(lookahead_index, 4)

    def test_controller_step_applies_smoothing_and_rate_limit(self) -> None:
        controller = PurePursuitController(steer_alpha=0.5, max_steer_delta=0.1)
        result = controller.step(
            ego_pose=Pose3D(x=0.0, y=0.0, z=0.0, yaw_deg=0.0),
            ego_speed_mps=5.0,
            trajectory=_left_curve_trajectory(),
        )

        self.assertGreater(result.steer_raw, 0.0)
        self.assertLessEqual(abs(result.steer_applied), 0.1)
        self.assertLessEqual(abs(result.steer_filtered), abs(result.steer_raw))

    def test_controller_step_does_not_accept_behavior_or_route_backbone_inputs(self) -> None:
        controller = PurePursuitController()
        with self.assertRaises(TypeError):
            controller.step(  # type: ignore[call-arg]
                ego_pose=Pose3D(x=0.0, y=0.0, z=0.0, yaw_deg=0.0),
                ego_speed_mps=1.0,
                trajectory=_straight_trajectory(),
                behavior_state="lane_follow",
            )
