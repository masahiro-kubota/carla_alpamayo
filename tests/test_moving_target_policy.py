from __future__ import annotations

import unittest

from ad_stack.overtake import (
    OvertakeLeadSnapshot,
    build_moving_overtake_targets,
    next_moving_overtake_target,
)


class MovingTargetPolicyTests(unittest.TestCase):
    def test_builds_single_target_for_slow_moving_lead(self) -> None:
        targets = build_moving_overtake_targets(
            [
                OvertakeLeadSnapshot(
                    actor_id=101,
                    lane_id="15:-1",
                    distance_m=24.0,
                    speed_mps=3.0,
                    relative_speed_mps=2.5,
                    is_stopped=False,
                )
            ],
            max_target_speed_mps=6.0,
            min_relative_speed_mps=1.0,
            max_distance_m=40.0,
            cluster_merge_gap_m=12.0,
            cluster_max_member_speed_delta_mps=1.0,
        )

        self.assertEqual(len(targets), 1)
        self.assertEqual(targets[0].kind, "single_actor")
        self.assertEqual(targets[0].primary_actor_id, 101)
        self.assertFalse(targets[0].is_stopped)

    def test_rejects_stopped_and_too_fast_targets(self) -> None:
        targets = build_moving_overtake_targets(
            [
                OvertakeLeadSnapshot(
                    actor_id=101,
                    lane_id="15:-1",
                    distance_m=20.0,
                    speed_mps=0.0,
                    relative_speed_mps=2.0,
                    is_stopped=True,
                ),
                OvertakeLeadSnapshot(
                    actor_id=102,
                    lane_id="15:-1",
                    distance_m=22.0,
                    speed_mps=9.0,
                    relative_speed_mps=2.0,
                    is_stopped=False,
                ),
                OvertakeLeadSnapshot(
                    actor_id=103,
                    lane_id="15:-1",
                    distance_m=24.0,
                    speed_mps=4.0,
                    relative_speed_mps=0.2,
                    is_stopped=False,
                ),
            ],
            max_target_speed_mps=6.0,
            min_relative_speed_mps=1.0,
            max_distance_m=40.0,
            cluster_merge_gap_m=12.0,
            cluster_max_member_speed_delta_mps=1.0,
        )

        self.assertEqual(targets, [])

    def test_clusters_close_moving_targets_in_same_lane(self) -> None:
        targets = build_moving_overtake_targets(
            [
                OvertakeLeadSnapshot(
                    actor_id=201,
                    lane_id="15:-1",
                    distance_m=18.0,
                    speed_mps=4.0,
                    relative_speed_mps=2.0,
                    is_stopped=False,
                ),
                OvertakeLeadSnapshot(
                    actor_id=202,
                    lane_id="15:-1",
                    distance_m=24.0,
                    speed_mps=4.4,
                    relative_speed_mps=2.1,
                    is_stopped=False,
                ),
            ],
            max_target_speed_mps=6.0,
            min_relative_speed_mps=1.0,
            max_distance_m=40.0,
            cluster_merge_gap_m=8.0,
            cluster_max_member_speed_delta_mps=1.0,
        )

        self.assertEqual(len(targets), 1)
        self.assertEqual(targets[0].kind, "cluster")
        self.assertEqual(targets[0].member_actor_ids, (201, 202))
        self.assertEqual(targets[0].entry_distance_m, 18.0)
        self.assertEqual(targets[0].exit_distance_m, 24.0)

    def test_next_target_advances_after_current_primary_actor(self) -> None:
        targets = build_moving_overtake_targets(
            [
                OvertakeLeadSnapshot(
                    actor_id=301,
                    lane_id="15:-1",
                    distance_m=18.0,
                    speed_mps=4.0,
                    relative_speed_mps=2.0,
                    is_stopped=False,
                ),
                OvertakeLeadSnapshot(
                    actor_id=302,
                    lane_id="15:-1",
                    distance_m=40.0,
                    speed_mps=3.5,
                    relative_speed_mps=2.5,
                    is_stopped=False,
                ),
            ],
            max_target_speed_mps=6.0,
            min_relative_speed_mps=1.0,
            max_distance_m=60.0,
            cluster_merge_gap_m=5.0,
            cluster_max_member_speed_delta_mps=0.5,
        )

        next_target = next_moving_overtake_target(
            targets,
            current_primary_actor_id=301,
        )

        assert next_target is not None
        self.assertEqual(next_target.primary_actor_id, 302)


if __name__ == "__main__":
    unittest.main()
