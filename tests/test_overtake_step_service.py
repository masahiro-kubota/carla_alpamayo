from __future__ import annotations

import unittest

from ad_stack.overtake import (
    AdjacentLaneGapSnapshot,
    OvertakeContext,
    OvertakeRuntimeState,
    OvertakeStepRequest,
    OvertakeTargetSnapshot,
    resolve_overtake_step,
)
from ad_stack.overtake.policies import accept_stopped_overtake_target


def _context(**overrides) -> OvertakeContext:
    base = dict(
        timestamp_s=1.0,
        current_lane_id="15:-1",
        origin_lane_id=None,
        route_target_lane_id="15:-1",
        target_speed_kmh=30.0,
        lead=None,
        left_lane=AdjacentLaneGapSnapshot(
            lane_id="15:1",
            front_gap_m=40.0,
            rear_gap_m=20.0,
            lane_open=True,
        ),
        right_lane=None,
        active_signal_state=None,
        signal_stop_distance_m=None,
        allow_overtake=True,
        preferred_direction="left_first",
        active_target=None,
    )
    base.update(overrides)
    return OvertakeContext(**base)


def _request(**overrides) -> OvertakeStepRequest:
    base = dict(
        runtime_state=OvertakeRuntimeState(),
        decision_context=_context(),
        target_acceptance_policy=accept_stopped_overtake_target,
        stop_for_light=False,
        ignore_traffic_lights=False,
        ignore_vehicles=False,
        lead_vehicle_present=False,
        active_target_present=False,
        lead_distance_m=None,
        lead_speed_mps=0.0,
        follow_target_speed_kmh=10.0,
        current_speed_mps=6.0,
        current_lane_id="15:-1",
        route_target_lane_id="15:-1",
        lane_center_offset_m=0.0,
        target_speed_kmh=30.0,
        active_light_state=None,
        signal_stop_distance_m=None,
        traffic_light_brake_start_distance_m=10.0,
        traffic_light_creep_resume_distance_m=1.0,
        traffic_light_creep_speed_kmh=4.0,
        overtake_speed_delta_kmh=8.0,
        overtake_trigger_distance_m=18.0,
        overtake_min_front_gap_m=20.0,
        overtake_min_rear_gap_m=15.0,
        overtake_signal_suppression_distance_m=35.0,
        rejoin_front_gap_m=None,
        rejoin_rear_gap_m=None,
    )
    base.update(overrides)
    return OvertakeStepRequest(**base)


class OvertakeStepServiceTests(unittest.TestCase):
    def test_requests_lane_change_out_for_stopped_target(self) -> None:
        active_target = OvertakeTargetSnapshot(
            kind="single_actor",
            primary_actor_id=101,
            member_actor_ids=(101,),
            lane_id="15:-1",
            entry_distance_m=12.0,
            exit_distance_m=12.0,
            speed_mps=0.0,
            motion_profile="stopped",
        )
        request = _request(
            decision_context=_context(active_target=active_target),
            active_target_present=True,
            lead_distance_m=12.0,
        )

        decision = resolve_overtake_step(request)

        self.assertEqual(decision.planner_state, "lane_change_out")
        self.assertEqual(decision.request_overtake_direction, "left")
        self.assertTrue(decision.overtake_considered)

    def test_requests_rejoin_only_after_target_passed_and_gaps_open(self) -> None:
        runtime_state = OvertakeRuntimeState()
        runtime_state.state = "pass_vehicle"
        runtime_state.memory.target_passed = True

        request = _request(
            runtime_state=runtime_state,
            rejoin_front_gap_m=35.0,
            rejoin_rear_gap_m=15.5,
        )

        decision = resolve_overtake_step(request)

        self.assertEqual(decision.planner_state, "pass_vehicle")
        self.assertTrue(decision.request_rejoin)

    def test_keeps_cruise_speed_during_lane_change_out_for_stopped_target(self) -> None:
        runtime_state = OvertakeRuntimeState()
        runtime_state.state = "lane_change_out"
        runtime_state.memory.target_lane_id = "15:1"
        active_target = OvertakeTargetSnapshot(
            kind="single_actor",
            primary_actor_id=101,
            member_actor_ids=(101,),
            lane_id="15:-1",
            entry_distance_m=12.0,
            exit_distance_m=12.0,
            speed_mps=0.0,
            motion_profile="stopped",
        )

        decision = resolve_overtake_step(
            _request(
                runtime_state=runtime_state,
                decision_context=_context(active_target=active_target),
                current_lane_id="15:-1",
                route_target_lane_id="15:-1",
                target_speed_kmh=30.0,
                follow_target_speed_kmh=0.0,
                lead_speed_mps=0.0,
                overtake_speed_delta_kmh=8.0,
            )
        )

        self.assertEqual(decision.planner_state, "lane_change_out")
        self.assertEqual(decision.target_speed_kmh, 30.0)

    def test_marks_signal_suppressed_while_waiting_at_red(self) -> None:
        active_target = OvertakeTargetSnapshot(
            kind="single_actor",
            primary_actor_id=201,
            member_actor_ids=(201,),
            lane_id="15:-1",
            entry_distance_m=8.0,
            exit_distance_m=8.0,
            speed_mps=0.0,
            motion_profile="stopped",
        )
        request = _request(
            decision_context=_context(
                active_signal_state="red",
                signal_stop_distance_m=10.0,
                active_target=active_target,
            ),
            stop_for_light=True,
            lead_vehicle_present=True,
            active_target_present=True,
            follow_target_speed_kmh=0.0,
            active_light_state="red",
            signal_stop_distance_m=10.0,
        )

        decision = resolve_overtake_step(request)

        self.assertEqual(decision.planner_state, "traffic_light_stop")
        self.assertEqual(decision.overtake_reject_reason, "signal_suppressed")
        self.assertTrue(decision.event_flags.unsafe_lane_change_reject)


if __name__ == "__main__":
    unittest.main()
