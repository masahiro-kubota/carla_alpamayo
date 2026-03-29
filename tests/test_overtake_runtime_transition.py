from __future__ import annotations

import unittest

from ad_stack.overtake.application.runtime_transition import (
    resolve_overtake_runtime_transition,
)


class OvertakeRuntimeTransitionTest(unittest.TestCase):
    def test_lane_change_out_completes_when_partial_bypass_reaches_lateral_offset(self) -> None:
        transition = resolve_overtake_runtime_transition(
            state="lane_change_out",
            aborted=False,
            current_lane_id="15:-1",
            target_lane_id="15:-1:left",
            origin_lane_id="15:-1",
            route_target_lane_id="15:-1",
            lane_center_offset_m=1.2,
            should_stop_for_light=False,
            target_speed_kmh=20.0,
            follow_target_speed_kmh=0.0,
            lead_speed_kmh=0.0,
            overtake_target_speed_kmh=20.0,
        )

        self.assertEqual(transition.state, "pass_vehicle")
        self.assertEqual(transition.planner_state, "pass_vehicle")
        self.assertEqual(transition.phase_target_speed_kmh, 20.0)


if __name__ == "__main__":
    unittest.main()
