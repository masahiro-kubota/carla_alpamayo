from __future__ import annotations

import unittest

from ad_stack.overtake.application import OvertakeRuntimeState
from ad_stack.overtake.domain import OvertakeTargetSnapshot


class OvertakeRuntimeStateTests(unittest.TestCase):
    def test_begin_lane_change_out_uses_active_target_when_present(self) -> None:
        runtime = OvertakeRuntimeState()
        active_target = OvertakeTargetSnapshot(
            kind="cluster",
            primary_actor_id=101,
            member_actor_ids=(101, 102),
            lane_id="15:-1",
            entry_distance_m=12.0,
            exit_distance_m=26.0,
            speed_mps=0.0,
            is_stopped=True,
        )

        runtime.begin_lane_change_out(
            direction="left",
            origin_lane_id="15:-1",
            active_target=active_target,
            lead_actor_id=99,
            lead_lane_id="15:-1",
            lead_distance_m=10.0,
            target_lane_id="15:1",
        )

        self.assertEqual(runtime.state, "lane_change_out")
        self.assertEqual(runtime.direction, "left")
        self.assertEqual(runtime.origin_lane_id, "15:-1")
        self.assertEqual(runtime.target_actor_id, 101)
        self.assertEqual(runtime.target_member_actor_ids, (101, 102))
        self.assertEqual(runtime.memory.target_kind, "cluster")
        self.assertEqual(runtime.memory.target_lane_id, "15:1")

    def test_begin_lane_change_out_falls_back_to_single_lead(self) -> None:
        runtime = OvertakeRuntimeState()

        runtime.begin_lane_change_out(
            direction="right",
            origin_lane_id="7:1",
            active_target=None,
            lead_actor_id=42,
            lead_lane_id="7:1",
            lead_distance_m=18.0,
            target_lane_id="7:-1",
        )

        self.assertEqual(runtime.target_actor_id, 42)
        self.assertEqual(runtime.target_member_actor_ids, (42,))
        self.assertEqual(runtime.memory.target_kind, "single_actor")
        self.assertEqual(runtime.memory.target_exit_distance_m, 18.0)

    def test_reset_clears_runtime_fields(self) -> None:
        runtime = OvertakeRuntimeState()
        runtime.begin_lane_change_out(
            direction="left",
            origin_lane_id="15:-1",
            active_target=None,
            lead_actor_id=1,
            lead_lane_id="15:-1",
            lead_distance_m=12.0,
            target_lane_id="15:1",
        )

        runtime.reset()

        self.assertEqual(runtime.state, "idle")
        self.assertIsNone(runtime.direction)
        self.assertIsNone(runtime.origin_lane_id)
        self.assertIsNone(runtime.target_actor_id)
        self.assertEqual(runtime.target_member_actor_ids, ())
        self.assertFalse(runtime.aborted)
        self.assertEqual(runtime.memory.state, "idle")
