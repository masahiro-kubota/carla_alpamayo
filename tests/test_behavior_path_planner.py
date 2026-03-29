from __future__ import annotations

import unittest

from ad_stack.overtake.application.behavior_path_planner import BehaviorPathPlanner
from ad_stack.overtake.domain.planning_models import BehaviorPlan, Pose3D, TrackedTarget, TrafficLightObservation
from tests.planning_controller_test_helpers import make_scene, make_straight_route_backbone


class BehaviorPathPlannerTests(unittest.TestCase):
    def test_lane_follow_returns_behavior_plan_and_trajectory(self) -> None:
        planner = BehaviorPathPlanner()
        plan, trajectory = planner.plan(
            route_backbone=make_straight_route_backbone(road_option="left"),
            scene=make_scene(),
        )

        self.assertEqual(plan.state, "lane_follow")
        self.assertEqual(plan.route_command, "left")
        self.assertIsNone(plan.reject_reason)
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

    def test_stopped_target_outside_trigger_stays_in_lane_follow_even_if_adjacent_lane_closed(self) -> None:
        planner = BehaviorPathPlanner()
        plan, trajectory = planner.plan(
            route_backbone=make_straight_route_backbone(),
            scene=make_scene(
                tracked_targets=(
                    TrackedTarget(
                        actor_id=11,
                        lane_id="15:-1",
                        longitudinal_distance_m=30.0,
                        speed_mps=0.0,
                    ),
                ),
                left_open=False,
                right_open=False,
            ),
        )

        self.assertEqual(plan.state, "lane_follow")
        self.assertEqual(plan.active_target_id, 11)
        self.assertIsNone(plan.reject_reason)
        self.assertEqual(trajectory.trajectory_id, "lane_follow")

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
        self.assertIsNone(plan.reject_reason)
        self.assertIsNotNone(plan.target_lane_id)
        self.assertEqual(trajectory.trajectory_id, "lane_change_out")

    def test_moving_target_keeps_behavior_plan_and_trajectory_shape(self) -> None:
        planner = BehaviorPathPlanner()
        plan, trajectory = planner.plan(
            route_backbone=make_straight_route_backbone(),
            scene=make_scene(
                tracked_targets=(
                    TrackedTarget(
                        actor_id=21,
                        lane_id="15:-1",
                        longitudinal_distance_m=10.0,
                        speed_mps=4.0,
                        target_kind="single_actor",
                    ),
                ),
                left_open=True,
            ),
        )

        self.assertEqual(plan.state, "lane_change_out")
        self.assertEqual(plan.active_target_id, 21)
        self.assertEqual(plan.active_target_kind, "single_actor")
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

    def test_plan_runtime_builds_behavior_plan_and_route_trajectory(self) -> None:
        planner = BehaviorPathPlanner()

        plan, trajectory = planner.plan_runtime(
            route_backbone=make_straight_route_backbone(road_option="right"),
            route_index=3,
            route_command="right",
            planner_state="nominal_cruise",
            desired_speed_mps=5.0,
            active_target_id=None,
            active_target_kind=None,
            origin_lane_id=None,
            target_lane_id=None,
            reject_reason=None,
        )

        self.assertEqual(plan.state, "lane_follow")
        self.assertEqual(plan.route_command, "right")
        self.assertEqual(trajectory.trajectory_id, "lane_follow")
        self.assertIsNone(trajectory.origin_lane_id)

    def test_plan_runtime_uses_local_path_samples_for_overtake_states(self) -> None:
        planner = BehaviorPathPlanner()

        plan, trajectory = planner.plan_runtime(
            route_backbone=make_straight_route_backbone(),
            route_index=2,
            route_command="straight",
            planner_state="lane_change_out",
            desired_speed_mps=4.0,
            active_target_id=91,
            active_target_kind="single_actor",
            origin_lane_id="15:-1",
            target_lane_id="15:-1:left",
            reject_reason=None,
            local_path_samples=(
                Pose3D(x=0.0, y=0.0, z=0.0, yaw_deg=0.0),
                Pose3D(x=1.2, y=0.2, z=0.0, yaw_deg=5.0),
                Pose3D(x=2.6, y=0.9, z=0.0, yaw_deg=15.0),
            ),
        )

        self.assertEqual(plan.state, "lane_change_out")
        self.assertEqual(plan.active_target_id, 91)
        self.assertEqual(trajectory.trajectory_id, "lane_change_out")
        self.assertEqual(trajectory.origin_lane_id, "15:-1")
        self.assertEqual(trajectory.target_lane_id, "15:-1:left")
