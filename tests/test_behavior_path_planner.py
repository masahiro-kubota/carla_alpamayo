from __future__ import annotations

import unittest

from ad_stack.overtake.application.behavior_path_planner import BehaviorPathPlanner
from ad_stack.overtake.domain.planning_models import BehaviorPlan, TrackedTarget, TrafficLightObservation
from tests.planning_controller_test_helpers import make_scene, make_straight_route_backbone


class BehaviorPathPlannerTests(unittest.TestCase):
    def test_lane_follow_returns_behavior_plan_and_trajectory(self) -> None:
        planner = BehaviorPathPlanner()
        plan, trajectory = planner.plan(
            route_backbone=make_straight_route_backbone(road_option="straight"),
            scene=make_scene(),
        )

        self.assertEqual(plan.state, "lane_follow")
        self.assertEqual(plan.route_command, "straight")
        self.assertEqual(trajectory.trajectory_id, "lane_follow")

    def test_car_follow_rejects_overtake_when_adjacent_lane_closed(self) -> None:
        planner = BehaviorPathPlanner()
        plan, trajectory = planner.plan(
            route_backbone=make_straight_route_backbone(),
            scene=make_scene(
                tracked_targets=(
                    TrackedTarget(
                        actor_id=10,
                        lane_id="15:-1",
                        longitudinal_distance_m=12.0,
                        speed_mps=0.0,
                    ),
                ),
                left_open=False,
                right_open=False,
            ),
        )

        self.assertEqual(plan.state, "car_follow")
        self.assertEqual(plan.active_target_id, 10)
        self.assertEqual(plan.reject_reason, "adjacent_lane_closed")
        self.assertEqual(trajectory.trajectory_id, "car_follow")

    def test_lane_change_out_uses_active_target_kind(self) -> None:
        planner = BehaviorPathPlanner()
        plan, trajectory = planner.plan(
            route_backbone=make_straight_route_backbone(),
            scene=make_scene(
                tracked_targets=(
                    TrackedTarget(
                        actor_id=20,
                        lane_id="15:-1",
                        longitudinal_distance_m=10.0,
                        speed_mps=0.0,
                        target_kind="cluster",
                    ),
                ),
                left_open=True,
            ),
        )

        self.assertEqual(plan.state, "lane_change_out")
        self.assertEqual(plan.active_target_kind, "cluster")
        self.assertEqual(plan.origin_lane_id, "15:-1")
        self.assertIsNotNone(plan.target_lane_id)
        self.assertEqual(trajectory.trajectory_id, "lane_change_out")

    def test_previous_lane_change_out_advances_to_pass_vehicle(self) -> None:
        planner = BehaviorPathPlanner()
        previous = BehaviorPlan(
            state="lane_change_out",
            route_command="straight",
            active_target_id=30,
            active_target_kind="single_actor",
            origin_lane_id="15:-1",
            target_lane_id="15:-1:left",
        )
        plan, trajectory = planner.plan(
            route_backbone=make_straight_route_backbone(),
            scene=make_scene(current_lane_id="15:-1:left"),
            previous_behavior_plan=previous,
        )

        self.assertEqual(plan.state, "pass_vehicle")
        self.assertEqual(plan.active_target_id, 30)
        self.assertEqual(trajectory.trajectory_id, "pass_vehicle")

    def test_previous_pass_vehicle_advances_to_lane_change_back_when_target_gone(self) -> None:
        planner = BehaviorPathPlanner()
        previous = BehaviorPlan(
            state="pass_vehicle",
            route_command="straight",
            active_target_id=40,
            active_target_kind="single_actor",
            origin_lane_id="15:-1",
            target_lane_id="15:-1:left",
        )
        plan, trajectory = planner.plan(
            route_backbone=make_straight_route_backbone(),
            scene=make_scene(current_lane_id="15:-1:left"),
            previous_behavior_plan=previous,
        )

        self.assertEqual(plan.state, "lane_change_back")
        self.assertEqual(plan.target_lane_id, "15:-1")
        self.assertEqual(trajectory.trajectory_id, "lane_change_back")

    def test_signal_stop_generates_stop_trajectory(self) -> None:
        planner = BehaviorPathPlanner()
        plan, trajectory = planner.plan(
            route_backbone=make_straight_route_backbone(),
            scene=make_scene(
                traffic_lights=(
                    TrafficLightObservation(
                        actor_id=1,
                        state="red",
                        affects_ego=True,
                        stop_line_distance_m=10.0,
                    ),
                )
            ),
        )

        self.assertEqual(plan.state, "signal_stop")
        self.assertEqual(trajectory.trajectory_id, "signal_stop")
        self.assertEqual(trajectory.points[0].longitudinal_velocity_mps, 20.0 / 3.6)
        self.assertEqual(trajectory.points[10].longitudinal_velocity_mps, 0.0)

    def test_red_light_during_pass_vehicle_requests_abort_return(self) -> None:
        planner = BehaviorPathPlanner()
        previous = BehaviorPlan(
            state="pass_vehicle",
            route_command="straight",
            active_target_id=50,
            active_target_kind="single_actor",
            origin_lane_id="15:-1",
            target_lane_id="15:-1:left",
        )
        plan, trajectory = planner.plan(
            route_backbone=make_straight_route_backbone(),
            scene=make_scene(
                current_lane_id="15:-1:left",
                traffic_lights=(
                    TrafficLightObservation(
                        actor_id=2,
                        state="red",
                        affects_ego=True,
                        stop_line_distance_m=8.0,
                    ),
                ),
            ),
            previous_behavior_plan=previous,
        )

        self.assertEqual(plan.state, "abort_return")
        self.assertEqual(plan.reject_reason, "signal_stop")
        self.assertEqual(trajectory.trajectory_id, "abort_return")
