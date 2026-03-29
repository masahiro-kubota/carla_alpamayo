from __future__ import annotations

import unittest
from dataclasses import dataclass
from pathlib import Path

from ad_stack.overtake import (
    AdjacentLaneGapSnapshot,
    OvertakeLeadSnapshot,
    OvertakeContext,
    OvertakeMemory,
    choose_overtake_action,
    should_begin_rejoin,
)
from ad_stack.overtake.policies import accept_stopped_overtake_target
from libs.project import PROJECT_ROOT
from simulation.environment_config import load_environment_config
from simulation.pipelines.route_loop_run_config import load_route_loop_run_config, resolve_user_path
from tests.integration.ad_stack._shared.overtake_scenario_contract import (
    PreflightValidationInput,
    parse_overtake_scenario_config,
    validate_preflight,
)


@dataclass(frozen=True, slots=True)
class ScenarioContractCase:
    name: str
    run_config_path: str
    route_path: str
    environment_path: str
    scenario_kind: str
    expected_npc_count: int
    expected_blocker_present: bool
    expected_blocker_target_speed_kmh: float | None
    expected_route_aligned_adjacent_lane_available: bool
    expected_nearest_signal_distance_m: float | None
    expected_nearest_junction_distance_m: float | None
    preflight_input: PreflightValidationInput
    expected_valid: bool
    expected_errors: tuple[str, ...]
    expected_warnings: tuple[str, ...]


def _context(
    *,
    lead_distance_m: float = 20.0,
    left_lane_open: bool = True,
    right_lane_open: bool = False,
    left_front_gap_m: float | None = 40.0,
    left_rear_gap_m: float | None = 20.0,
    active_signal_state: str | None = None,
    signal_stop_distance_m: float | None = None,
) -> OvertakeContext:
    return OvertakeContext(
        timestamp_s=0.0,
        current_lane_id="15:-1",
        origin_lane_id="15:-1",
        route_target_lane_id="15:-1",
        target_speed_kmh=30.0,
        lead=OvertakeLeadSnapshot(
            actor_id=101,
            lane_id="15:-1",
            distance_m=lead_distance_m,
            speed_mps=0.0,
            relative_speed_mps=2.0,
            motion_profile="stopped",
        ),
        left_lane=AdjacentLaneGapSnapshot(
            lane_id="15:1",
            front_gap_m=left_front_gap_m,
            rear_gap_m=left_rear_gap_m,
            lane_open=left_lane_open,
        ),
        right_lane=AdjacentLaneGapSnapshot(
            lane_id="15:-2",
            front_gap_m=40.0,
            rear_gap_m=20.0,
            lane_open=right_lane_open,
        ),
        active_signal_state=active_signal_state,
        signal_stop_distance_m=signal_stop_distance_m,
        allow_overtake=True,
        preferred_direction="left_first",
        active_target=None,
    )


_CASES: tuple[ScenarioContractCase, ...] = (
    ScenarioContractCase(
        name="signal_suppressed",
        run_config_path="tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_signal_suppressed_long_expert.json",
        route_path="scenarios/routes/town01_signal_conflict_short.json",
        environment_path="scenarios/environments/town01_stopped_obstacle_signal_suppressed_45s.json",
        scenario_kind="signal_suppressed",
        expected_npc_count=1,
        expected_blocker_present=False,
        expected_blocker_target_speed_kmh=None,
        expected_route_aligned_adjacent_lane_available=True,
        expected_nearest_signal_distance_m=20.0,
        expected_nearest_junction_distance_m=None,
        preflight_input=PreflightValidationInput(
            scenario_kind="signal_suppressed",
            ego_lane_id="10:1",
            obstacle_lane_id="10:1",
            obstacle_route_lane_id="10:1",
            ego_to_obstacle_longitudinal_distance_m=18.0,
            left_lane_is_driving=True,
            right_lane_is_driving=False,
            route_target_lane_id="10:1",
            nearest_signal_distance_m=20.0,
            route_aligned_adjacent_lane_available=True,
        ),
        expected_valid=True,
        expected_errors=(),
        expected_warnings=("signal_nearby",),
    ),
    ScenarioContractCase(
        name="near_junction_preflight_reject",
        run_config_path="tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_near_junction_preflight_reject_long_expert.json",
        route_path="scenarios/routes/town01_signal_conflict_short.json",
        environment_path="scenarios/environments/town01_stopped_obstacle_near_junction_preflight_reject_30s.json",
        scenario_kind="near_junction_preflight_reject",
        expected_npc_count=1,
        expected_blocker_present=False,
        expected_blocker_target_speed_kmh=None,
        expected_route_aligned_adjacent_lane_available=True,
        expected_nearest_signal_distance_m=None,
        expected_nearest_junction_distance_m=16.0,
        preflight_input=PreflightValidationInput(
            scenario_kind="near_junction_preflight_reject",
            ego_lane_id="10:1",
            obstacle_lane_id="10:1",
            obstacle_route_lane_id="10:1",
            ego_to_obstacle_longitudinal_distance_m=18.0,
            left_lane_is_driving=True,
            right_lane_is_driving=False,
            route_target_lane_id="10:1",
            nearest_junction_distance_m=16.0,
            route_aligned_adjacent_lane_available=True,
        ),
        expected_valid=False,
        expected_errors=("junction_nearby",),
        expected_warnings=("junction_nearby",),
    ),
    ScenarioContractCase(
        name="rejoin_blocked_then_release",
        run_config_path="tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_rejoin_blocked_then_release_long_expert.json",
        route_path="scenarios/routes/town01_northbound_overtake_long.json",
        environment_path="scenarios/environments/town01_stopped_obstacle_rejoin_blocked_then_release_long_60s.json",
        scenario_kind="rejoin_blocked_then_release",
        expected_npc_count=2,
        expected_blocker_present=True,
        expected_blocker_target_speed_kmh=40.0,
        expected_route_aligned_adjacent_lane_available=True,
        expected_nearest_signal_distance_m=None,
        expected_nearest_junction_distance_m=None,
        preflight_input=PreflightValidationInput(
            scenario_kind="rejoin_blocked_then_release",
            ego_lane_id="15:-1",
            obstacle_lane_id="15:-1",
            obstacle_route_lane_id="15:-1",
            blocker_lane_id="15:-1",
            ego_to_obstacle_longitudinal_distance_m=80.0,
            ego_to_blocker_longitudinal_distance_m=120.0,
            left_lane_is_driving=True,
            right_lane_is_driving=False,
            route_target_lane_id="15:-1",
            route_aligned_adjacent_lane_available=True,
        ),
        expected_valid=True,
        expected_errors=(),
        expected_warnings=(),
    ),
    ScenarioContractCase(
        name="curve_clear",
        run_config_path="tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_curve_clear_long_expert.json",
        route_path="scenarios/routes/town01_curve_overtake_clear.json",
        environment_path="scenarios/environments/town01_stopped_obstacle_curve_clear_45s.json",
        scenario_kind="curve_clear",
        expected_npc_count=1,
        expected_blocker_present=False,
        expected_blocker_target_speed_kmh=None,
        expected_route_aligned_adjacent_lane_available=True,
        expected_nearest_signal_distance_m=None,
        expected_nearest_junction_distance_m=999.0,
        preflight_input=PreflightValidationInput(
            scenario_kind="curve_clear",
            ego_lane_id="15:1",
            obstacle_lane_id="13:-1",
            obstacle_route_lane_id="13:-1",
            ego_to_obstacle_longitudinal_distance_m=14.0,
            left_lane_is_driving=True,
            right_lane_is_driving=False,
            route_target_lane_id="15:1",
            nearest_junction_distance_m=999.0,
            route_aligned_adjacent_lane_available=True,
        ),
        expected_valid=True,
        expected_errors=(),
        expected_warnings=(),
    ),
    ScenarioContractCase(
        name="adjacent_lane_closed",
        run_config_path="tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_adjacent_lane_closed_long_expert.json",
        route_path="scenarios/routes/town01_adjacent_lane_closed_corridor.json",
        environment_path="scenarios/environments/town01_stopped_obstacle_adjacent_lane_closed_45s.json",
        scenario_kind="adjacent_lane_closed",
        expected_npc_count=1,
        expected_blocker_present=False,
        expected_blocker_target_speed_kmh=None,
        expected_route_aligned_adjacent_lane_available=False,
        expected_nearest_signal_distance_m=None,
        expected_nearest_junction_distance_m=None,
        preflight_input=PreflightValidationInput(
            scenario_kind="adjacent_lane_closed",
            ego_lane_id="24:1",
            obstacle_lane_id="272:1",
            obstacle_route_lane_id="272:1",
            ego_to_obstacle_longitudinal_distance_m=40.0,
            left_lane_is_driving=False,
            right_lane_is_driving=False,
            route_target_lane_id="24:1",
            route_aligned_adjacent_lane_available=False,
        ),
        expected_valid=True,
        expected_errors=(),
        expected_warnings=(),
    ),
)


class StoppedObstacleScenarioContractTests(unittest.TestCase):
    def test_assets_match_contract(self) -> None:
        for case in _CASES:
            with self.subTest(case=case.name):
                run_config = load_route_loop_run_config(PROJECT_ROOT / case.run_config_path)
                environment = load_environment_config(PROJECT_ROOT / case.environment_path)

                self.assertEqual(
                    resolve_user_path(run_config.request.scenario.route_config_path),
                    (PROJECT_ROOT / case.route_path).resolve(),
                )
                self.assertEqual(
                    resolve_user_path(run_config.request.scenario.environment_config_path),
                    (PROJECT_ROOT / case.environment_path).resolve(),
                )
                self.assertEqual(environment.name, Path(case.environment_path).stem)
                self.assertEqual(len(environment.npc_vehicles), case.expected_npc_count)
                self.assertIsNotNone(environment.overtake_scenario)
                scenario_config = parse_overtake_scenario_config(environment.overtake_scenario)
                assert scenario_config is not None
                self.assertEqual(scenario_config.scenario_kind, case.scenario_kind)
                blocker_present = scenario_config.blocker_npc_index is not None
                self.assertEqual(blocker_present, case.expected_blocker_present)
                blocker_speed = None
                if blocker_present:
                    assert scenario_config.blocker_npc_index is not None
                    blocker_spec = environment.npc_vehicles[
                        scenario_config.blocker_npc_index
                    ]
                    blocker_speed = blocker_spec.target_speed_kmh
                self.assertEqual(blocker_speed, case.expected_blocker_target_speed_kmh)
                self.assertEqual(
                    scenario_config.route_aligned_adjacent_lane_available,
                    case.expected_route_aligned_adjacent_lane_available,
                )
                self.assertEqual(
                    scenario_config.nearest_signal_distance_m,
                    case.expected_nearest_signal_distance_m,
                )
                self.assertEqual(
                    scenario_config.nearest_junction_distance_m,
                    case.expected_nearest_junction_distance_m,
                )

    def test_preflight_contracts_match_expected(self) -> None:
        for case in _CASES:
            with self.subTest(case=case.name):
                result = validate_preflight(case.preflight_input)
                self.assertEqual(result.is_valid, case.expected_valid)
                self.assertEqual(tuple(result.errors), case.expected_errors)
                self.assertEqual(tuple(result.warnings), case.expected_warnings)

    def test_signal_suppressed_rejects_overtake_in_pure_logic(self) -> None:
        decision = choose_overtake_action(
            _context(active_signal_state="red", signal_stop_distance_m=20.0),
            overtake_trigger_distance_m=40.0,
            overtake_target_speed_kmh=30.0,
            overtake_min_front_gap_m=35.0,
            overtake_min_rear_gap_m=15.0,
            signal_suppression_distance_m=35.0,
            target_acceptance_policy=accept_stopped_overtake_target,
        )
        self.assertEqual(decision.planner_state, "car_follow")
        self.assertEqual(decision.reject_reason, "signal_suppressed")

    def test_rejoin_blocked_then_release_waits_then_rejoins(self) -> None:
        self.assertFalse(
            should_begin_rejoin(
                memory=OvertakeMemory(target_passed=True),
                rejoin_front_gap_m=8.0,
                rejoin_rear_gap_m=5.0,
                overtake_min_front_gap_m=35.0,
                overtake_min_rear_gap_m=15.0,
            )
        )
        self.assertTrue(
            should_begin_rejoin(
                memory=OvertakeMemory(target_passed=True),
                rejoin_front_gap_m=40.0,
                rejoin_rear_gap_m=20.0,
                overtake_min_front_gap_m=35.0,
                overtake_min_rear_gap_m=15.0,
            )
        )

    def test_adjacent_lane_closed_rejects_without_open_lane(self) -> None:
        decision = choose_overtake_action(
            _context(left_lane_open=False, right_lane_open=False),
            overtake_trigger_distance_m=40.0,
            overtake_target_speed_kmh=30.0,
            overtake_min_front_gap_m=35.0,
            overtake_min_rear_gap_m=15.0,
            signal_suppression_distance_m=35.0,
            target_acceptance_policy=accept_stopped_overtake_target,
        )
        self.assertEqual(decision.planner_state, "car_follow")
        self.assertEqual(decision.reject_reason, "adjacent_lane_closed")

    def test_blocked_static_near_preflight_contract_is_valid(self) -> None:
        result = validate_preflight(
            PreflightValidationInput(
                scenario_kind="blocked_static",
                ego_lane_id="15:-1",
                obstacle_lane_id="15:-1",
                obstacle_route_lane_id="15:-1",
                blocker_lane_id="15:1",
                ego_to_obstacle_longitudinal_distance_m=34.0,
                ego_to_blocker_longitudinal_distance_m=12.0,
                left_lane_is_driving=True,
                right_lane_is_driving=False,
                route_target_lane_id="15:-1",
                route_aligned_adjacent_lane_available=True,
            )
        )
        self.assertTrue(result.is_valid)
        self.assertEqual(tuple(result.errors), ())
        self.assertEqual(tuple(result.warnings), ())

    def test_blocked_static_near_rejects_due_to_gap_not_target_range(self) -> None:
        decision = choose_overtake_action(
            _context(
                lead_distance_m=34.0,
                left_front_gap_m=10.0,
                left_rear_gap_m=20.0,
                right_lane_open=False,
            ),
            overtake_trigger_distance_m=40.0,
            overtake_target_speed_kmh=30.0,
            overtake_min_front_gap_m=35.0,
            overtake_min_rear_gap_m=15.0,
            signal_suppression_distance_m=35.0,
            target_acceptance_policy=accept_stopped_overtake_target,
        )
        self.assertEqual(decision.planner_state, "car_follow")
        self.assertEqual(decision.reject_reason, "adjacent_front_gap_insufficient")


if __name__ == "__main__":
    unittest.main()
