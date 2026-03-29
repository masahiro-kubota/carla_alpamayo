from __future__ import annotations

import unittest

from ad_stack.overtake.domain.planning_models import BehaviorTrajectoryTelemetry


class BehaviorTrajectoryTelemetryTests(unittest.TestCase):
    def test_telemetry_uses_new_public_fields(self) -> None:
        telemetry = BehaviorTrajectoryTelemetry(
            behavior_state="lane_follow",
            route_command="straight",
            active_target_id=None,
            active_target_kind=None,
            reject_reason=None,
            desired_speed_mps=5.0,
            applied_speed_mps=4.5,
            lateral_error_m=0.1,
            heading_error_deg=2.0,
            lookahead_distance_m=4.0,
            controller_steer_raw=0.2,
            controller_steer_applied=0.1,
        )

        self.assertEqual(telemetry.behavior_state, "lane_follow")
        self.assertEqual(telemetry.route_command, "straight")
        self.assertEqual(telemetry.desired_speed_mps, 5.0)
        self.assertFalse(hasattr(telemetry, "emergency_stop"))
        self.assertFalse(hasattr(telemetry, "remaining_waypoints"))
        self.assertFalse(hasattr(telemetry, "lane_change_path_available"))
