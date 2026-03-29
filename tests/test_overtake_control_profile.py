from __future__ import annotations

import unittest

from ad_stack.overtake import (
    is_traffic_light_violation,
    should_stop_for_light,
    speed_control,
    traffic_light_stop_control,
    traffic_light_stop_target_distance_m,
    traffic_light_stop_target_speed_kmh,
)


class OvertakeControlProfileTests(unittest.TestCase):
    def test_speed_control_accelerates_when_below_target(self) -> None:
        throttle, brake, next_error = speed_control(
            current_speed_mps=0.0,
            target_speed_kmh=30.0,
            previous_error_kmh=0.0,
        )

        self.assertGreater(throttle, 0.0)
        self.assertEqual(brake, 0.0)
        self.assertEqual(next_error, 30.0)

    def test_should_stop_for_red_light(self) -> None:
        self.assertTrue(
            should_stop_for_light(
                ignore_traffic_lights=False,
                light_state="red",
                raw_stop_distance_m=20.0,
                current_speed_mps=6.0,
                stop_buffer_m=3.0,
                reaction_margin_m=2.0,
                preferred_deceleration_mps2=4.0,
                yellow_stop_margin_seconds=1.0,
            )
        )

    def test_traffic_light_stop_target_speed_creeps_when_nearly_stopped(self) -> None:
        target_speed_kmh = traffic_light_stop_target_speed_kmh(
            stop_target_distance_m=2.0,
            current_speed_mps=0.1,
            target_speed_kmh=30.0,
            brake_start_distance_m=10.0,
            creep_resume_distance_m=1.0,
            creep_speed_kmh=4.0,
        )

        self.assertEqual(target_speed_kmh, 6.0)

    def test_traffic_light_stop_control_hard_brakes_at_stop_line(self) -> None:
        throttle, brake, next_error = traffic_light_stop_control(
            current_speed_mps=3.0,
            light_state="red",
            stop_target_distance_m=0.2,
            target_speed_kmh=0.0,
            previous_error_kmh=5.0,
            preferred_deceleration_mps2=4.0,
        )

        self.assertEqual(throttle, 0.0)
        self.assertEqual(brake, 1.0)
        self.assertEqual(next_error, 0.0)

    def test_traffic_light_violation_requires_red_and_small_stop_distance(self) -> None:
        self.assertTrue(
            is_traffic_light_violation(
                light_state="red",
                stop_line_distance_m=0.4,
                current_speed_mps=2.5,
            )
        )
        self.assertFalse(
            is_traffic_light_violation(
                light_state="green",
                stop_line_distance_m=0.4,
                current_speed_mps=2.5,
            )
        )

    def test_stop_target_distance_applies_buffer(self) -> None:
        self.assertEqual(
            traffic_light_stop_target_distance_m(distance_m=8.0, stop_buffer_m=3.0),
            5.0,
        )

