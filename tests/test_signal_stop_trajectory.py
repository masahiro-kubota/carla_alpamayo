from __future__ import annotations

import unittest

from ad_stack.overtake.application.behavior_path_planner import BehaviorPathPlanner
from ad_stack.overtake.domain.planning_models import TrafficLightObservation
from tests.planning_controller_test_helpers import make_scene, make_straight_route_backbone


class SignalStopTrajectoryTests(unittest.TestCase):
    def test_signal_stop_trajectory_decelerates_monotonically_to_zero(self) -> None:
        planner = BehaviorPathPlanner()
        plan, trajectory = planner.plan(
            route_backbone=make_straight_route_backbone(),
            scene=make_scene(
                traffic_lights=(
                    TrafficLightObservation(
                        actor_id=1,
                        state="red",
                        affects_ego=True,
                        stop_line_distance_m=12.0,
                    ),
                ),
            ),
        )

        self.assertEqual(plan.state, "signal_stop")
        speeds = [point.longitudinal_velocity_mps for point in trajectory.points[:14]]
        self.assertEqual(speeds[-1], 0.0)
        for previous, current in zip(speeds, speeds[1:]):
            self.assertGreaterEqual(previous, current)
            self.assertGreaterEqual(current, 0.0)
