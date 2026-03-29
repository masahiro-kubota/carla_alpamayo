from __future__ import annotations

import unittest

from ad_stack.overtake import resolve_overtake_runtime_transition


class OvertakeRuntimeTransitionTests(unittest.TestCase):
    def test_lane_change_out_advances_to_pass_vehicle_on_target_lane(self) -> None:
        transition = resolve_overtake_runtime_transition(
            state="lane_change_out",
            aborted=False,
            current_lane_id="15:1",
            target_lane_id="15:1",
            origin_lane_id="15:-1",
            route_target_lane_id="15:-1",
            lane_center_offset_m=0.2,
            should_stop_for_light=False,
            target_speed_kmh=30.0,
            follow_target_speed_kmh=12.0,
            lead_speed_kmh=0.0,
            overtake_speed_delta_kmh=8.0,
        )

        self.assertEqual(transition.state, "pass_vehicle")
        self.assertEqual(transition.planner_state, "pass_vehicle")

    def test_stop_for_light_requests_abort_return(self) -> None:
        transition = resolve_overtake_runtime_transition(
            state="pass_vehicle",
            aborted=False,
            current_lane_id="15:1",
            target_lane_id="15:1",
            origin_lane_id="15:-1",
            route_target_lane_id="15:-1",
            lane_center_offset_m=0.2,
            should_stop_for_light=True,
            target_speed_kmh=30.0,
            follow_target_speed_kmh=10.0,
            lead_speed_kmh=0.0,
            overtake_speed_delta_kmh=8.0,
        )

        self.assertEqual(transition.state, "abort_return")
        self.assertEqual(transition.planner_state, "abort_return")
        self.assertTrue(transition.aborted)
        self.assertTrue(transition.should_prepare_abort_return)
        self.assertTrue(transition.event_overtake_abort)

    def test_lane_change_back_completion_reports_success(self) -> None:
        transition = resolve_overtake_runtime_transition(
            state="lane_change_back",
            aborted=False,
            current_lane_id="15:-1",
            target_lane_id="15:-1",
            origin_lane_id="15:-1",
            route_target_lane_id="15:-1",
            lane_center_offset_m=0.5,
            should_stop_for_light=False,
            target_speed_kmh=30.0,
            follow_target_speed_kmh=12.0,
            lead_speed_kmh=0.0,
            overtake_speed_delta_kmh=8.0,
        )

        self.assertEqual(transition.state, "idle")
        self.assertTrue(transition.completed)
        self.assertTrue(transition.event_overtake_success)

    def test_abort_return_completion_does_not_report_success(self) -> None:
        transition = resolve_overtake_runtime_transition(
            state="abort_return",
            aborted=True,
            current_lane_id="15:-1",
            target_lane_id="15:1",
            origin_lane_id="15:-1",
            route_target_lane_id="15:-1",
            lane_center_offset_m=0.3,
            should_stop_for_light=False,
            target_speed_kmh=30.0,
            follow_target_speed_kmh=12.0,
            lead_speed_kmh=0.0,
            overtake_speed_delta_kmh=8.0,
        )

        self.assertEqual(transition.state, "idle")
        self.assertTrue(transition.completed)
        self.assertFalse(transition.event_overtake_success)

