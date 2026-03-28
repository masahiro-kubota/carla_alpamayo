from __future__ import annotations

import unittest

from ad_stack.overtake import (
    AdjacentLaneGapSnapshot,
    LaneChangePlanPoint,
    OvertakeLeadSnapshot,
    OvertakeMemory,
    PreflightValidationInput,
    StoppedObstacleContext,
    build_route_aligned_lane_change_plan,
    choose_overtake_action,
    evaluate_pass_progress,
    lane_gap_for_lane_id,
    should_begin_rejoin,
    validate_preflight,
)


def _context(
    *,
    lead_distance_m: float = 20.0,
    lead_speed_mps: float = 0.0,
    lead_is_stopped: bool = True,
    preferred_direction: str = "left_first",
    active_signal_state: str | None = None,
    signal_stop_distance_m: float | None = None,
    left_front_gap_m: float | None = 40.0,
    left_rear_gap_m: float | None = 20.0,
    left_lane_open: bool = True,
    right_front_gap_m: float | None = 40.0,
    right_rear_gap_m: float | None = 20.0,
    right_lane_open: bool = True,
) -> StoppedObstacleContext:
    return StoppedObstacleContext(
        timestamp_s=0.0,
        current_lane_id="15:-1",
        origin_lane_id="15:-1",
        route_target_lane_id="15:-1",
        target_speed_kmh=30.0,
        stopped_speed_threshold_mps=0.3,
        lead=OvertakeLeadSnapshot(
            actor_id=101,
            lane_id="15:-1",
            distance_m=lead_distance_m,
            speed_mps=lead_speed_mps,
            relative_speed_mps=2.0,
            is_stopped=lead_is_stopped,
        ),
        left_lane=AdjacentLaneGapSnapshot(
            lane_id="15:1",
            front_gap_m=left_front_gap_m,
            rear_gap_m=left_rear_gap_m,
            lane_open=left_lane_open,
        ),
        right_lane=AdjacentLaneGapSnapshot(
            lane_id="15:-2",
            front_gap_m=right_front_gap_m,
            rear_gap_m=right_rear_gap_m,
            lane_open=right_lane_open,
        ),
        active_signal_state=active_signal_state,
        signal_stop_distance_m=signal_stop_distance_m,
        allow_overtake=True,
        preferred_direction=preferred_direction,  # type: ignore[arg-type]
    )


class StoppedObstacleLogicTests(unittest.TestCase):
    def test_overtake_rejects_when_lead_out_of_range(self) -> None:
        decision = choose_overtake_action(
            _context(lead_distance_m=50.0),
            overtake_trigger_distance_m=40.0,
            overtake_speed_delta_kmh=8.0,
            overtake_min_front_gap_m=35.0,
            overtake_min_rear_gap_m=15.0,
            signal_suppression_distance_m=35.0,
        )
        self.assertEqual(decision.planner_state, "car_follow")
        self.assertEqual(decision.reject_reason, "lead_out_of_range")

    def test_overtake_rejects_when_adjacent_front_gap_insufficient(self) -> None:
        decision = choose_overtake_action(
            _context(left_front_gap_m=10.0, right_lane_open=False),
            overtake_trigger_distance_m=40.0,
            overtake_speed_delta_kmh=8.0,
            overtake_min_front_gap_m=35.0,
            overtake_min_rear_gap_m=15.0,
            signal_suppression_distance_m=35.0,
        )
        self.assertEqual(decision.planner_state, "car_follow")
        self.assertEqual(decision.reject_reason, "adjacent_front_gap_insufficient")

    def test_overtake_rejects_when_adjacent_rear_gap_insufficient(self) -> None:
        decision = choose_overtake_action(
            _context(left_rear_gap_m=10.0, right_lane_open=False),
            overtake_trigger_distance_m=40.0,
            overtake_speed_delta_kmh=8.0,
            overtake_min_front_gap_m=35.0,
            overtake_min_rear_gap_m=15.0,
            signal_suppression_distance_m=35.0,
        )
        self.assertEqual(decision.planner_state, "car_follow")
        self.assertEqual(decision.reject_reason, "adjacent_rear_gap_insufficient")

    def test_overtake_rejects_when_signal_suppressed(self) -> None:
        decision = choose_overtake_action(
            _context(active_signal_state="red", signal_stop_distance_m=20.0),
            overtake_trigger_distance_m=40.0,
            overtake_speed_delta_kmh=8.0,
            overtake_min_front_gap_m=35.0,
            overtake_min_rear_gap_m=15.0,
            signal_suppression_distance_m=35.0,
        )
        self.assertEqual(decision.planner_state, "car_follow")
        self.assertEqual(decision.reject_reason, "signal_suppressed")

    def test_overtake_accepts_clear_case(self) -> None:
        decision = choose_overtake_action(
            _context(),
            overtake_trigger_distance_m=40.0,
            overtake_speed_delta_kmh=8.0,
            overtake_min_front_gap_m=35.0,
            overtake_min_rear_gap_m=15.0,
            signal_suppression_distance_m=35.0,
        )
        self.assertEqual(decision.planner_state, "lane_change_out")
        self.assertEqual(decision.direction, "left")
        self.assertEqual(decision.target_lane_id, "15:1")

    def test_overtake_prefers_left_when_both_sides_are_open(self) -> None:
        decision = choose_overtake_action(
            _context(preferred_direction="left_first"),
            overtake_trigger_distance_m=40.0,
            overtake_speed_delta_kmh=8.0,
            overtake_min_front_gap_m=35.0,
            overtake_min_rear_gap_m=15.0,
            signal_suppression_distance_m=35.0,
        )
        self.assertEqual(decision.planner_state, "lane_change_out")
        self.assertEqual(decision.direction, "left")
        self.assertEqual(decision.target_lane_id, "15:1")

    def test_overtake_prefers_right_when_both_sides_are_open_and_right_first(self) -> None:
        decision = choose_overtake_action(
            _context(preferred_direction="right_first"),
            overtake_trigger_distance_m=40.0,
            overtake_speed_delta_kmh=8.0,
            overtake_min_front_gap_m=35.0,
            overtake_min_rear_gap_m=15.0,
            signal_suppression_distance_m=35.0,
        )
        self.assertEqual(decision.planner_state, "lane_change_out")
        self.assertEqual(decision.direction, "right")
        self.assertEqual(decision.target_lane_id, "15:-2")

    def test_pass_not_complete_while_target_still_ahead(self) -> None:
        memory = OvertakeMemory(target_actor_id=101)
        updated = evaluate_pass_progress(
            memory,
            timestamp_s=3.0,
            target_actor_visible=True,
            target_longitudinal_distance_m=6.0,
            overtake_resume_front_gap_m=12.0,
        )
        self.assertFalse(updated.target_passed)
        self.assertIsNone(updated.target_pass_distance_m)

    def test_pass_not_complete_until_resume_gap_is_met(self) -> None:
        memory = OvertakeMemory(target_actor_id=101)
        updated = evaluate_pass_progress(
            memory,
            timestamp_s=3.0,
            target_actor_visible=True,
            target_longitudinal_distance_m=-8.0,
            overtake_resume_front_gap_m=12.0,
        )
        self.assertFalse(updated.target_passed)
        self.assertEqual(updated.target_pass_distance_m, 8.0)

    def test_pass_complete_once_target_is_behind_and_resume_gap_met(self) -> None:
        memory = OvertakeMemory(target_actor_id=101)
        updated = evaluate_pass_progress(
            memory,
            timestamp_s=3.0,
            target_actor_visible=True,
            target_longitudinal_distance_m=-13.0,
            overtake_resume_front_gap_m=12.0,
        )
        self.assertTrue(updated.target_passed)
        self.assertEqual(updated.target_pass_distance_m, 13.0)

    def test_pass_does_not_complete_when_target_actor_temporarily_disappears(self) -> None:
        memory = OvertakeMemory(
            target_actor_id=101,
            target_actor_last_seen_s=2.0,
            target_actor_last_seen_longitudinal_m=-3.0,
            target_actor_visibility_timeout_s=1.0,
        )
        updated = evaluate_pass_progress(
            memory,
            timestamp_s=2.5,
            target_actor_visible=False,
            target_longitudinal_distance_m=None,
            overtake_resume_front_gap_m=12.0,
        )
        self.assertFalse(updated.target_passed)
        self.assertEqual(updated.target_actor_last_seen_s, 2.0)

    def test_pass_does_not_complete_after_visibility_timeout_without_actor_reacquisition(self) -> None:
        memory = OvertakeMemory(
            target_actor_id=101,
            target_actor_last_seen_s=2.0,
            target_actor_last_seen_longitudinal_m=-20.0,
            target_actor_visibility_timeout_s=1.0,
            state="pass_vehicle",
        )
        updated = evaluate_pass_progress(
            memory,
            timestamp_s=3.5,
            target_actor_visible=False,
            target_longitudinal_distance_m=None,
            overtake_resume_front_gap_m=12.0,
        )
        self.assertFalse(updated.target_passed)
        self.assertEqual(updated.state, "pass_vehicle")

    def test_rejoin_gap_uses_origin_lane_id_not_relation_name(self) -> None:
        lane_gaps = {
            "15:-1": AdjacentLaneGapSnapshot(
                lane_id="15:-1", front_gap_m=40.0, rear_gap_m=20.0, lane_open=True
            ),
            "15:1": AdjacentLaneGapSnapshot(
                lane_id="15:1", front_gap_m=10.0, rear_gap_m=5.0, lane_open=True
            ),
        }
        gap = lane_gap_for_lane_id(lane_gaps, "15:-1")
        self.assertIsNotNone(gap)
        self.assertEqual(gap.lane_id, "15:-1")
        self.assertEqual(gap.front_gap_m, 40.0)

    def test_rejoin_rejects_when_origin_front_gap_insufficient(self) -> None:
        begin = should_begin_rejoin(
            OvertakeMemory(target_passed=True),
            rejoin_front_gap_m=20.0,
            rejoin_rear_gap_m=20.0,
            overtake_min_front_gap_m=35.0,
            overtake_min_rear_gap_m=15.0,
        )
        self.assertFalse(begin)

    def test_rejoin_rejects_when_origin_rear_gap_insufficient(self) -> None:
        begin = should_begin_rejoin(
            OvertakeMemory(target_passed=True),
            rejoin_front_gap_m=40.0,
            rejoin_rear_gap_m=5.0,
            overtake_min_front_gap_m=35.0,
            overtake_min_rear_gap_m=15.0,
        )
        self.assertFalse(begin)

    def test_rejoin_begins_when_passed_and_origin_gaps_are_safe(self) -> None:
        begin = should_begin_rejoin(
            OvertakeMemory(target_passed=True),
            rejoin_front_gap_m=40.0,
            rejoin_rear_gap_m=20.0,
            overtake_min_front_gap_m=35.0,
            overtake_min_rear_gap_m=15.0,
        )
        self.assertTrue(begin)

    def test_lane_change_plan_does_not_reverse_when_target_lane_native_direction_is_opposite(
        self,
    ) -> None:
        origin_samples = [
            LaneChangePlanPoint(route_index=i, lane_id="15:-1", progress_m=float(i * 2.0))
            for i in range(10)
        ]
        # route progress still increases even if the actual lane yaw would be opposite
        target_samples = [
            LaneChangePlanPoint(route_index=i, lane_id="15:1", progress_m=float(i * 2.0))
            for i in range(10)
        ]
        plan = build_route_aligned_lane_change_plan(
            origin_samples,
            target_samples,
            distance_same_lane_m=4.0,
            lane_change_distance_m=8.0,
            distance_other_lane_m=8.0,
        )
        self.assertTrue(plan.available)
        self.assertEqual(plan.points[0].lane_id, "15:-1")
        self.assertIn("15:1", [point.lane_id for point in plan.points])
        self.assertEqual(
            [point.progress_m for point in plan.points],
            sorted(point.progress_m for point in plan.points),
        )

    def test_lane_change_plan_marks_failure_when_adjacent_lane_sample_is_missing(self) -> None:
        origin_samples = [
            LaneChangePlanPoint(route_index=i, lane_id="15:-1", progress_m=float(i * 2.0))
            for i in range(10)
        ]
        target_samples = [
            LaneChangePlanPoint(route_index=0, lane_id="15:1", progress_m=0.0),
            LaneChangePlanPoint(route_index=1, lane_id="15:1", progress_m=2.0),
        ]
        plan = build_route_aligned_lane_change_plan(
            origin_samples,
            target_samples,
            distance_same_lane_m=4.0,
            lane_change_distance_m=8.0,
            distance_other_lane_m=8.0,
        )
        self.assertFalse(plan.available)
        self.assertEqual(plan.failure_reason, "adjacent_lane_sample_missing")

    def test_preflight_clear_accepts_same_lane_obstacle_and_open_adjacent_lane(self) -> None:
        result = validate_preflight(
            PreflightValidationInput(
                scenario_kind="clear",
                ego_lane_id="15:-1",
                obstacle_lane_id="15:-1",
                ego_to_obstacle_longitudinal_distance_m=20.0,
                left_lane_is_driving=True,
                route_target_lane_id="15:-1",
                route_aligned_adjacent_lane_available=True,
            )
        )
        self.assertTrue(result.is_valid)
        self.assertEqual(result.errors, [])

    def test_preflight_rejects_obstacle_not_in_ego_lane(self) -> None:
        result = validate_preflight(
            PreflightValidationInput(
                scenario_kind="clear",
                ego_lane_id="15:-1",
                obstacle_lane_id="15:1",
                ego_to_obstacle_longitudinal_distance_m=20.0,
                left_lane_is_driving=True,
                route_target_lane_id="15:-1",
                route_aligned_adjacent_lane_available=True,
            )
        )
        self.assertFalse(result.is_valid)
        self.assertIn("obstacle_not_in_ego_lane", result.errors)

    def test_preflight_rejects_blocker_not_in_opposite_lane_for_blocked_static(self) -> None:
        result = validate_preflight(
            PreflightValidationInput(
                scenario_kind="blocked_static",
                ego_lane_id="15:-1",
                obstacle_lane_id="15:-1",
                blocker_lane_id="15:-1",
                ego_to_obstacle_longitudinal_distance_m=20.0,
                ego_to_blocker_longitudinal_distance_m=30.0,
                left_lane_is_driving=True,
                route_target_lane_id="15:-1",
                route_aligned_adjacent_lane_available=True,
            )
        )
        self.assertFalse(result.is_valid)
        self.assertIn("blocker_not_in_opposite_lane", result.errors)

    def test_preflight_warns_when_signal_or_junction_is_too_close(self) -> None:
        result = validate_preflight(
            PreflightValidationInput(
                scenario_kind="clear",
                ego_lane_id="15:-1",
                obstacle_lane_id="15:-1",
                ego_to_obstacle_longitudinal_distance_m=20.0,
                left_lane_is_driving=True,
                route_target_lane_id="15:-1",
                nearest_signal_distance_m=10.0,
                nearest_junction_distance_m=20.0,
                route_aligned_adjacent_lane_available=True,
            )
        )
        self.assertTrue(result.is_valid)
        self.assertIn("signal_nearby", result.warnings)
        self.assertIn("junction_nearby", result.warnings)

    def test_preflight_rejects_adjacent_lane_closed_when_adjacent_driving_lane_exists(self) -> None:
        result = validate_preflight(
            PreflightValidationInput(
                scenario_kind="adjacent_lane_closed",
                ego_lane_id="15:-1",
                obstacle_lane_id="15:-1",
                ego_to_obstacle_longitudinal_distance_m=20.0,
                left_lane_is_driving=True,
                route_target_lane_id="15:-1",
                route_aligned_adjacent_lane_available=False,
            )
        )
        self.assertFalse(result.is_valid)
        self.assertIn("adjacent_lane_not_closed", result.errors)

    def test_preflight_invalidates_near_junction_reject_scenario_when_hazard_is_close(self) -> None:
        result = validate_preflight(
            PreflightValidationInput(
                scenario_kind="near_junction_preflight_reject",
                ego_lane_id="23:-1",
                obstacle_lane_id="23:-1",
                ego_to_obstacle_longitudinal_distance_m=20.0,
                left_lane_is_driving=True,
                route_target_lane_id="23:-1",
                nearest_signal_distance_m=18.0,
                nearest_junction_distance_m=16.0,
                route_aligned_adjacent_lane_available=True,
            )
        )
        self.assertFalse(result.is_valid)
        self.assertIn("signal_nearby", result.errors)
        self.assertIn("junction_nearby", result.errors)


if __name__ == "__main__":
    unittest.main()
