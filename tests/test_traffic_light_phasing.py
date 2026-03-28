from __future__ import annotations

import unittest

from libs.carla_utils.traffic_light_phasing import TrafficLightApproach, build_opposing_phase_groups


class TrafficLightPhasingTests(unittest.TestCase):
    def test_cross_intersection_groups_opposing_approaches(self) -> None:
        phases = build_opposing_phase_groups(
            [
                TrafficLightApproach(actor_id=10, heading_deg=0.0),
                TrafficLightApproach(actor_id=11, heading_deg=180.0),
                TrafficLightApproach(actor_id=20, heading_deg=90.0),
                TrafficLightApproach(actor_id=21, heading_deg=270.0),
            ]
        )

        self.assertEqual(phases, [[10, 11], [20, 21]])

    def test_t_junction_groups_mainline_opposing_approaches_together(self) -> None:
        phases = build_opposing_phase_groups(
            [
                TrafficLightApproach(actor_id=30, heading_deg=90.0),
                TrafficLightApproach(actor_id=31, heading_deg=270.0),
                TrafficLightApproach(actor_id=40, heading_deg=0.0),
            ]
        )

        self.assertEqual(phases, [[40], [30, 31]])

    def test_small_heading_noise_still_merges_same_axis(self) -> None:
        phases = build_opposing_phase_groups(
            [
                TrafficLightApproach(actor_id=50, heading_deg=2.0),
                TrafficLightApproach(actor_id=51, heading_deg=178.5),
                TrafficLightApproach(actor_id=60, heading_deg=92.0),
                TrafficLightApproach(actor_id=61, heading_deg=269.0),
            ],
            axis_merge_tolerance_deg=15.0,
        )

        self.assertEqual(phases, [[50, 51], [60, 61]])

    def test_invalid_negative_tolerance_raises(self) -> None:
        with self.assertRaises(ValueError):
            build_opposing_phase_groups(
                [TrafficLightApproach(actor_id=1, heading_deg=0.0)],
                axis_merge_tolerance_deg=-1.0,
            )


if __name__ == "__main__":
    unittest.main()
