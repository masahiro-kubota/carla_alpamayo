from __future__ import annotations

import unittest

from ad_stack.overtake.application.behavior_path_planner import BehaviorPathPlanner
from ad_stack.overtake.domain.planning_models import Trajectory, TrajectoryPoint
from tests.planning_controller_test_helpers import make_scene, make_straight_route_backbone


class TrajectoryContractTests(unittest.TestCase):
    def test_trajectory_accepts_metadata_and_minimal_points(self) -> None:
        trajectory = Trajectory(
            points=(
                TrajectoryPoint(x=0.0, y=0.0, z=0.0, yaw_deg=0.0, longitudinal_velocity_mps=5.0),
                TrajectoryPoint(x=1.0, y=0.0, z=0.0, yaw_deg=0.0, longitudinal_velocity_mps=5.0),
            ),
            trajectory_id="lane_follow",
            origin_lane_id="15:-1",
            target_lane_id="15:1",
            source_route_start_index=0,
            source_route_end_index=12,
        )

        self.assertEqual(trajectory.trajectory_id, "lane_follow")
        self.assertEqual(trajectory.origin_lane_id, "15:-1")
        self.assertEqual(trajectory.target_lane_id, "15:1")
        self.assertFalse(hasattr(trajectory, "route_command"))
        self.assertFalse(hasattr(trajectory.points[0], "lane_id"))

    def test_trajectory_rejects_bad_spacing(self) -> None:
        with self.assertRaisesRegex(ValueError, "spacing"):
            Trajectory(
                points=(
                    TrajectoryPoint(x=0.0, y=0.0, z=0.0, yaw_deg=0.0, longitudinal_velocity_mps=5.0),
                    TrajectoryPoint(x=3.0, y=0.0, z=0.0, yaw_deg=0.0, longitudinal_velocity_mps=5.0),
                ),
                trajectory_id="bad",
            )

    def test_lane_follow_trajectory_uses_60m_horizon_and_1m_sampling(self) -> None:
        planner = BehaviorPathPlanner()
        backbone = make_straight_route_backbone(num_points=120)
        plan, trajectory = planner.plan(
            route_backbone=backbone,
            scene=make_scene(route_index=0, route_progress_m=0.0),
        )

        self.assertEqual(plan.state, "lane_follow")
        self.assertEqual(len(trajectory.points), 61)
        self.assertAlmostEqual(trajectory.points[-1].x - trajectory.points[0].x, 60.0, places=6)
        self.assertEqual(trajectory.source_route_start_index, 0)
        self.assertGreaterEqual(trajectory.source_route_end_index, 60)
