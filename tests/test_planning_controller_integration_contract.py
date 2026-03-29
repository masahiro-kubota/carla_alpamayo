from __future__ import annotations

import unittest

from ad_stack.overtake.application.behavior_path_planner import BehaviorPathPlanner
from ad_stack.overtake.application.pure_pursuit_controller import PurePursuitController
from ad_stack.overtake.domain.planning_models import BehaviorPlan, Trajectory
from tests.planning_controller_test_helpers import make_scene, make_straight_route_backbone


class PlanningControllerIntegrationContractTests(unittest.TestCase):
    def test_planner_public_output_is_behavior_plan_plus_trajectory(self) -> None:
        planner = BehaviorPathPlanner()

        plan, trajectory = planner.plan(
            route_backbone=make_straight_route_backbone(),
            scene=make_scene(),
        )

        self.assertIsInstance(plan, BehaviorPlan)
        self.assertIsInstance(trajectory, Trajectory)

    def test_controller_public_input_is_trajectory_only(self) -> None:
        planner = BehaviorPathPlanner()
        controller = PurePursuitController()
        _, trajectory = planner.plan(
            route_backbone=make_straight_route_backbone(),
            scene=make_scene(),
        )

        with self.assertRaises(TypeError):
            controller.step(  # type: ignore[call-arg]
                ego_pose=make_scene().ego_pose,
                ego_speed_mps=make_scene().ego_speed_mps,
                trajectory=trajectory,
                route_backbone=make_straight_route_backbone(),
            )
