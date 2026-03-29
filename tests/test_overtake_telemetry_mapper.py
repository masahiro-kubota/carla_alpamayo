from __future__ import annotations

import unittest

from ad_stack.overtake.infrastructure.carla.telemetry_mapper import (
    RouteLoopTelemetryAccumulator,
    build_ego_state_sample,
    build_episode_record,
    build_overtake_planning_debug,
    build_planning_debug_mcap_payload,
)


class RouteLoopTelemetryAccumulatorTests(unittest.TestCase):
    def test_accumulator_tracks_counts_and_rejoin_metrics(self) -> None:
        accumulator = RouteLoopTelemetryAccumulator()

        accumulator.observe(
            planning_debug={
                "event_traffic_light_stop": True,
                "event_traffic_light_resume": True,
                "event_car_follow_start": True,
                "event_overtake_attempt": True,
                "event_overtake_success": False,
                "event_overtake_abort": False,
                "event_unsafe_lane_change_reject": True,
                "traffic_light_violation": True,
                "min_ttc": 4.8,
                "lead_vehicle_distance_m": 19.5,
                "target_passed": True,
            },
            elapsed_seconds=12.0,
        )
        accumulator.observe(
            planning_debug={
                "traffic_light_violation": True,
                "event_overtake_success": True,
                "overtake_state": "lane_change_back",
                "rejoin_front_gap_m": 35.4,
                "rejoin_rear_gap_m": 15.2,
            },
            elapsed_seconds=12.6,
        )
        accumulator.observe(planning_debug={"traffic_light_violation": False}, elapsed_seconds=13.0)

        summary = accumulator.summary_fields()

        self.assertEqual(summary["traffic_light_stop_count"], 1)
        self.assertEqual(summary["traffic_light_resume_count"], 1)
        self.assertEqual(summary["traffic_light_violation_count"], 1)
        self.assertEqual(summary["car_follow_event_count"], 1)
        self.assertEqual(summary["overtake_attempt_count"], 1)
        self.assertEqual(summary["overtake_success_count"], 1)
        self.assertEqual(summary["unsafe_lane_change_reject_count"], 1)
        self.assertEqual(summary["min_ttc"], 4.8)
        self.assertEqual(summary["min_lead_distance_m"], 19.5)
        self.assertEqual(summary["first_target_passed_s"], 12.0)
        self.assertEqual(summary["first_rejoin_started_s"], 12.6)
        self.assertEqual(summary["rejoin_wait_after_target_passed_s"], 0.6)
        self.assertEqual(summary["first_rejoin_front_gap_m"], 35.4)
        self.assertEqual(summary["first_rejoin_rear_gap_m"], 15.2)


class RouteLoopTelemetryMapperTests(unittest.TestCase):
    def test_build_overtake_planning_debug_maps_runtime_fields(self) -> None:
        class _Light:
            actor_id = 31
            state = "red"
            distance_m = 18.0
            stop_line_distance_m = 14.0

        class _Lead:
            actor_id = 101
            lane_id = "15:-1"

        class _Target:
            primary_actor_id = 201
            lane_id = "15:1"

        payload = build_overtake_planning_debug(
            remaining_waypoints=12,
            route_index=7,
            max_route_index=9,
            current_lane_id="15:-1",
            lane_center_offset_m=0.2,
            route_target_lane_id="15:-1",
            left_lane_open=True,
            right_lane_open=False,
            active_light=_Light(),
            red_light_latched=True,
            traffic_light_stop_buffer_m=3.0,
            traffic_light_stop_target_distance_m=12.5,
            target_speed_kmh=24.0,
            lead_vehicle=_Lead(),
            active_target=_Target(),
            lead_distance_m=16.0,
            lead_speed_mps=0.0,
            closing_speed_mps=5.0,
            left_lane_front_gap_m=float("inf"),
            left_lane_rear_gap_m=18.0,
            right_lane_front_gap_m=9.0,
            right_lane_rear_gap_m=float("inf"),
            rejoin_front_gap_m=35.0,
            rejoin_rear_gap_m=15.5,
            overtake_considered=True,
            overtake_reject_reason=None,
            overtake_state="pass_vehicle",
            overtake_direction="left",
            overtake_origin_lane_id="15:-1",
            overtake_target_actor_id=201,
            overtake_target_kind="single_actor",
            overtake_target_member_actor_ids=(201,),
            overtake_target_lane_id="15:1",
            target_passed=False,
            distance_past_target_m=None,
            target_actor_visible=True,
            target_actor_last_seen_s=22.0,
            lane_change_path_available=True,
            lane_change_path_failed_reason=None,
            target_lane_id="15:1",
            min_ttc=float("inf"),
            emergency_stop=False,
            event_flags={"event_overtake_attempt": True},
        )

        self.assertEqual(payload["remaining_waypoints"], 12)
        self.assertEqual(payload["traffic_light_actor_id"], 31)
        self.assertTrue(payload["traffic_light_red_latched"])
        self.assertEqual(payload["lead_vehicle_id"], 101)
        self.assertIsNone(payload["left_lane_front_gap_m"])
        self.assertEqual(payload["right_lane_front_gap_m"], 9.0)
        self.assertEqual(payload["overtake_target_actor_id"], 201)
        self.assertEqual(payload["event_overtake_attempt"], True)
        self.assertIsNone(payload["min_ttc"])

    def test_planning_debug_payload_omits_top_level_planning_fields(self) -> None:
        payload = build_planning_debug_mcap_payload(
            {
                "planner_state": "car_follow",
                "traffic_light_state": "red",
                "overtake_state": "pass_vehicle",
                "target_lane_id": "15:1",
                "min_ttc": 3.2,
                "lead_vehicle_id": 101,
                "target_passed": False,
            }
        )

        self.assertEqual(
            payload,
            {
                "lead_vehicle_id": 101,
                "target_passed": False,
            },
        )

    def test_build_ego_state_sample_filters_planning_debug(self) -> None:
        ego_state = build_ego_state_sample(
            episode_id="ep",
            frame_id=7,
            timestamp_s=123.0,
            elapsed_seconds=5.0,
            speed_mps=4.0,
            behavior="car_follow",
            route_completion_ratio=0.5,
            distance_to_goal_m=11.0,
            planner_state="car_follow",
            traffic_light_state="red",
            lead_vehicle_distance_m=9.0,
            overtake_state="idle",
            target_lane_id="15:-1",
            min_ttc=4.2,
            pose={"x": 1.0, "y": 2.0, "z": 0.0, "yaw_deg": 0.0, "pitch_deg": 0.0, "roll_deg": 0.0},
            control={"steer": 0.1, "throttle": 0.2, "brake": 0.0},
            planning_debug={
                "planner_state": "car_follow",
                "traffic_light_state": "red",
                "lead_vehicle_id": 12,
            },
        )

        self.assertEqual(ego_state.planning_debug, {"lead_vehicle_id": 12})

    def test_build_episode_record_maps_overtake_fields(self) -> None:
        record = build_episode_record(
            episode_id="ep",
            frame_id=3,
            town_id="Town01",
            route_id="route",
            weather_id="ClearNoon",
            timestamp=10.0,
            speed=3.5,
            command="car_follow",
            steer=0.0,
            throttle=0.3,
            brake=0.0,
            collision=False,
            lane_invasion=False,
            success=True,
            vehicle_x=1.0,
            vehicle_y=2.0,
            vehicle_z=0.1,
            vehicle_yaw_deg=90.0,
            route_completion_ratio=0.25,
            distance_to_goal_m=20.0,
            expert_steer=0.1,
            route_target_x=5.0,
            route_target_y=6.0,
            planner_state="pass_vehicle",
            planning_debug={
                "overtake_state": "pass_vehicle",
                "overtake_reject_reason": None,
                "overtake_target_actor_id": 44,
                "overtake_target_kind": "cluster",
                "overtake_target_member_actor_ids": [44, 45],
                "target_passed": False,
                "current_lane_id": "15:1",
                "route_target_lane_id": "15:-1",
                "target_lane_id": "15:1",
            },
            mcap_segment_index=0,
            mcap_segment_path="outputs/evaluate/x/telemetry/segment_0000.mcap",
        )

        self.assertEqual(record.overtake_state, "pass_vehicle")
        self.assertEqual(record.overtake_target_actor_id, 44)
        self.assertEqual(record.overtake_target_kind, "cluster")
        self.assertEqual(record.overtake_target_member_actor_ids, [44, 45])
        self.assertEqual(record.current_lane_id, "15:1")
        self.assertEqual(record.route_target_lane_id, "15:-1")
        self.assertEqual(record.target_lane_id, "15:1")


if __name__ == "__main__":
    unittest.main()
