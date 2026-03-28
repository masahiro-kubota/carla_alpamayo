from __future__ import annotations

import unittest

from libs.carla_utils.traffic_light_phasing import (
    TrafficLightApproach,
    TrafficLightPhaseCycle,
    build_opposing_phase_groups,
    compute_phase_states,
)


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

    def test_compute_phase_states_for_two_phase_cycle(self) -> None:
        phases = [[10, 11], [20]]
        cycle = TrafficLightPhaseCycle(green_seconds=10.0, yellow_seconds=2.0, red_seconds=6.0)

        self.assertEqual(
            compute_phase_states(phases, elapsed_seconds=0.0, cycle=cycle),
            {10: "green", 11: "green", 20: "red"},
        )
        self.assertEqual(
            compute_phase_states(phases, elapsed_seconds=10.5, cycle=cycle),
            {10: "yellow", 11: "yellow", 20: "red"},
        )
        self.assertEqual(
            compute_phase_states(phases, elapsed_seconds=12.5, cycle=cycle),
            {10: "red", 11: "red", 20: "green"},
        )
        self.assertEqual(
            compute_phase_states(phases, elapsed_seconds=24.5, cycle=cycle),
            {10: "red", 11: "red", 20: "red"},
        )

    def test_compute_phase_states_respects_initial_offset(self) -> None:
        phases = [[10, 11], [20]]
        cycle = TrafficLightPhaseCycle(
            green_seconds=10.0,
            yellow_seconds=2.0,
            red_seconds=6.0,
            initial_offset_seconds=12.0,
        )

        self.assertEqual(
            compute_phase_states(phases, elapsed_seconds=0.0, cycle=cycle),
            {10: "red", 11: "red", 20: "green"},
        )

    def test_compute_phase_states_rejects_invalid_cycle(self) -> None:
        with self.assertRaises(ValueError):
            compute_phase_states(
                [[10]],
                elapsed_seconds=0.0,
                cycle=TrafficLightPhaseCycle(green_seconds=-1.0, yellow_seconds=0.0, red_seconds=0.0),
            )


if __name__ == "__main__":
    unittest.main()
