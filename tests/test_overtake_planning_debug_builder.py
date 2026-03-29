from __future__ import annotations

import unittest

from ad_stack.overtake.infrastructure.carla.telemetry_mapper import build_overtake_planning_debug


class OvertakePlanningDebugBuilderTests(unittest.TestCase):
    def test_build_overtake_planning_debug_includes_behavior_and_controller_fields(self) -> None:
        planning_debug = build_overtake_planning_debug(
            behavior_state="lane_follow",
            route_command="straight",
            remaining_waypoints=10,
            route_index=3,
            max_route_index=5,
            current_lane_id="15:-1",
            lane_center_offset_m=0.2,
            route_target_lane_id="15:-1",
            left_lane_open=True,
            right_lane_open=False,
            active_light=None,
            red_light_latched=False,
            traffic_light_stop_buffer_m=3.0,
            traffic_light_stop_target_distance_m=None,
            target_speed_kmh=20.0,
            desired_speed_mps=5.0,
            applied_speed_mps=4.5,
            lookahead_distance_m=4.2,
            lateral_error_m=0.3,
            heading_error_deg=1.2,
            controller_steer_raw=0.15,
            controller_steer_applied=0.1,
            follow_lead=None,
            active_target=None,
            follow_distance_m=None,
            follow_speed_mps=None,
            closing_speed_mps=None,
            left_lane_front_gap_m=float("inf"),
            left_lane_rear_gap_m=float("inf"),
            right_lane_front_gap_m=float("inf"),
            right_lane_rear_gap_m=float("inf"),
            rejoin_front_gap_m=float("inf"),
            rejoin_rear_gap_m=float("inf"),
            overtake_considered=False,
            overtake_reject_reason=None,
            overtake_state="idle",
            overtake_direction=None,
            overtake_origin_lane_id=None,
            overtake_target_actor_id=None,
            overtake_target_kind="single_actor",
            overtake_target_member_actor_ids=(),
            overtake_target_lane_id=None,
            target_passed=False,
            distance_past_target_m=None,
            target_actor_visible=False,
            target_actor_last_seen_s=None,
            lane_change_path_available=True,
            lane_change_path_failed_reason=None,
            target_lane_id="15:-1",
            min_ttc=float("inf"),
            event_flags={},
        )

        self.assertEqual(planning_debug.core.behavior_state, "lane_follow")
        self.assertEqual(planning_debug.core.route_command, "straight")
        self.assertEqual(planning_debug.core.desired_speed_mps, 5.0)
        self.assertEqual(planning_debug.core.applied_speed_mps, 4.5)
        self.assertEqual(planning_debug.core.lookahead_distance_m, 4.2)
        self.assertEqual(planning_debug.core.controller_steer_raw, 0.15)
        self.assertEqual(planning_debug.core.controller_steer_applied, 0.1)
