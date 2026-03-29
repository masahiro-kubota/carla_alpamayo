from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from tests.integration.ad_stack._shared import (
    InspectOnlySuiteCase,
    ManifestExpectation,
    RouteLoopSuiteCase,
    ScenarioSummaryExpectation,
)


SUITE_DIR = Path(__file__).resolve().parent


def _run_config(name: str) -> Path:
    return SUITE_DIR / "run_configs" / name


@dataclass(frozen=True, slots=True)
class StoppedObstacleRouteLoopScenario:
    case: RouteLoopSuiteCase
    summary_expectation: ScenarioSummaryExpectation
    manifest_expectations: tuple[ManifestExpectation, ...] = ()


ROUTE_LOOP_SCENARIOS: tuple[StoppedObstacleRouteLoopScenario, ...] = (
    StoppedObstacleRouteLoopScenario(
        case=RouteLoopSuiteCase(
            name="clear",
            run_config_path=_run_config("town01_stopped_obstacle_clear_long_expert.json"),
        ),
        summary_expectation=ScenarioSummaryExpectation(
            name="clear",
            success=True,
            collision_count=0,
            min_overtake_attempt_count=1,
            min_overtake_success_count=1,
        ),
    ),
    StoppedObstacleRouteLoopScenario(
        case=RouteLoopSuiteCase(
            name="clear_with_far_opposite_static",
            run_config_path=_run_config(
                "town01_stopped_obstacle_clear_with_far_opposite_static_long_expert.json"
            ),
        ),
        summary_expectation=ScenarioSummaryExpectation(
            name="clear_with_far_opposite_static",
            success=True,
            collision_count=0,
            min_average_speed_kmh=13.0,
            min_overtake_attempt_count=1,
            min_overtake_success_count=1,
        ),
        manifest_expectations=(
            ManifestExpectation(
                field="overtake_reject_reason",
                kind="none_equals",
                expected="adjacent_front_gap_insufficient",
                message="clear_with_far_opposite_static incorrectly rejected due to far opposite static",
            ),
            ManifestExpectation(
                field="target_speed_kmh",
                kind="min_numeric_where_equals",
                filter_field="overtake_state",
                filter_equals="lane_change_out",
                min_value=20.0,
                message="clear_with_far_opposite_static slowed lane_change_out target speed too much",
            ),
        ),
    ),
    StoppedObstacleRouteLoopScenario(
        case=RouteLoopSuiteCase(
            name="blocked_static",
            run_config_path=_run_config("town01_stopped_obstacle_blocked_long_expert.json"),
        ),
        summary_expectation=ScenarioSummaryExpectation(
            name="blocked_static",
            success=False,
            failure_reason="stalled",
            collision_count=0,
            exact_overtake_attempt_count=0,
            min_unsafe_lane_change_reject_count=1,
        ),
    ),
    StoppedObstacleRouteLoopScenario(
        case=RouteLoopSuiteCase(
            name="blocked_oncoming",
            run_config_path=_run_config("town01_stopped_obstacle_blocked_oncoming_long_expert.json"),
        ),
        summary_expectation=ScenarioSummaryExpectation(
            name="blocked_oncoming",
            success=True,
            collision_count=0,
            min_overtake_attempt_count=1,
            min_overtake_success_count=1,
            min_unsafe_lane_change_reject_count=1,
        ),
    ),
    StoppedObstacleRouteLoopScenario(
        case=RouteLoopSuiteCase(
            name="double_stopped_separated",
            run_config_path=_run_config(
                "town01_stopped_obstacle_double_stopped_separated_long_expert.json"
            ),
        ),
        summary_expectation=ScenarioSummaryExpectation(
            name="double_stopped_separated",
            success=True,
            collision_count=0,
            min_overtake_attempt_count=2,
            min_overtake_success_count=2,
        ),
        manifest_expectations=(
            ManifestExpectation(
                field="overtake_target_actor_id",
                kind="min_unique_non_null",
                min_unique=2,
                message="double_stopped_separated never switched target actor",
            ),
        ),
    ),
    StoppedObstacleRouteLoopScenario(
        case=RouteLoopSuiteCase(
            name="double_stopped_clustered",
            run_config_path=_run_config(
                "town01_stopped_obstacle_double_stopped_clustered_long_expert.json"
            ),
        ),
        summary_expectation=ScenarioSummaryExpectation(
            name="double_stopped_clustered",
            success=True,
            collision_count=0,
            min_overtake_attempt_count=1,
            min_overtake_success_count=1,
        ),
        manifest_expectations=(
            ManifestExpectation(
                field="overtake_target_kind",
                kind="any_equals",
                expected="cluster",
                message="double_stopped_clustered never reported cluster target kind",
            ),
            ManifestExpectation(
                field="overtake_target_member_actor_ids",
                kind="any_sequence_len_at_least",
                min_len=2,
                message="double_stopped_clustered never kept multi-actor cluster members",
            ),
        ),
    ),
    StoppedObstacleRouteLoopScenario(
        case=RouteLoopSuiteCase(
            name="double_stopped_clustered_with_oncoming_block",
            run_config_path=_run_config(
                "town01_stopped_obstacle_double_stopped_clustered_with_oncoming_block_long_expert.json"
            ),
        ),
        summary_expectation=ScenarioSummaryExpectation(
            name="double_stopped_clustered_with_oncoming_block",
            success=True,
            collision_count=0,
            min_overtake_attempt_count=1,
            min_overtake_success_count=1,
            min_unsafe_lane_change_reject_count=1,
        ),
        manifest_expectations=(
            ManifestExpectation(
                field="overtake_target_kind",
                kind="any_equals",
                expected="cluster",
                message="double_stopped_clustered_with_oncoming_block never reported cluster target kind",
            ),
            ManifestExpectation(
                field="overtake_target_member_actor_ids",
                kind="any_sequence_len_at_least",
                min_len=2,
                message="double_stopped_clustered_with_oncoming_block never kept multi-actor cluster members",
            ),
        ),
    ),
    StoppedObstacleRouteLoopScenario(
        case=RouteLoopSuiteCase(
            name="double_stopped_separated_with_far_opposite_static",
            run_config_path=_run_config(
                "town01_stopped_obstacle_double_stopped_separated_with_far_opposite_static_long_expert.json"
            ),
        ),
        summary_expectation=ScenarioSummaryExpectation(
            name="double_stopped_separated_with_far_opposite_static",
            success=True,
            collision_count=0,
            min_overtake_attempt_count=2,
            min_overtake_success_count=2,
        ),
        manifest_expectations=(
            ManifestExpectation(
                field="overtake_target_actor_id",
                kind="min_unique_non_null",
                min_unique=2,
                message="double_stopped_separated_with_far_opposite_static never switched target actor",
            ),
            ManifestExpectation(
                field="overtake_reject_reason",
                kind="none_equals",
                expected="adjacent_front_gap_insufficient",
                message="double_stopped_separated_with_far_opposite_static incorrectly rejected due to far opposite static",
            ),
        ),
    ),
    StoppedObstacleRouteLoopScenario(
        case=RouteLoopSuiteCase(
            name="signal_suppressed",
            run_config_path=_run_config("town01_stopped_obstacle_signal_suppressed_long_expert.json"),
        ),
        summary_expectation=ScenarioSummaryExpectation(
            name="signal_suppressed",
            success=False,
            failure_reason="stalled",
            collision_count=0,
            exact_overtake_attempt_count=0,
        ),
    ),
    StoppedObstacleRouteLoopScenario(
        case=RouteLoopSuiteCase(
            name="adjacent_lane_closed",
            run_config_path=_run_config(
                "town01_stopped_obstacle_adjacent_lane_closed_long_expert.json"
            ),
        ),
        summary_expectation=ScenarioSummaryExpectation(
            name="adjacent_lane_closed",
            success=False,
            failure_reason="stalled",
            collision_count=0,
            exact_overtake_attempt_count=0,
            min_unsafe_lane_change_reject_count=1,
        ),
    ),
    StoppedObstacleRouteLoopScenario(
        case=RouteLoopSuiteCase(
            name="curve_clear",
            run_config_path=_run_config("town01_stopped_obstacle_curve_clear_long_expert.json"),
        ),
        summary_expectation=ScenarioSummaryExpectation(
            name="curve_clear",
            success=True,
            collision_count=0,
            min_overtake_attempt_count=1,
            min_overtake_success_count=1,
        ),
    ),
    StoppedObstacleRouteLoopScenario(
        case=RouteLoopSuiteCase(
            name="curve_clear_with_opposite_static_blocked",
            run_config_path=_run_config(
                "town01_stopped_obstacle_curve_clear_with_opposite_static_blocked_long_expert.json"
            ),
        ),
        summary_expectation=ScenarioSummaryExpectation(
            name="curve_clear_with_opposite_static_blocked",
            success=False,
            failure_reason="stalled",
            collision_count=0,
            exact_overtake_attempt_count=0,
            min_unsafe_lane_change_reject_count=1,
        ),
        manifest_expectations=(
            ManifestExpectation(
                field="overtake_reject_reason",
                kind="any_equals",
                expected="adjacent_front_gap_insufficient",
                message="curve_clear_with_opposite_static_blocked never reported front-gap rejection",
            ),
        ),
    ),
    StoppedObstacleRouteLoopScenario(
        case=RouteLoopSuiteCase(
            name="rejoin_blocked_then_release",
            run_config_path=_run_config(
                "town01_stopped_obstacle_rejoin_blocked_then_release_long_expert.json"
            ),
        ),
        summary_expectation=ScenarioSummaryExpectation(
            name="rejoin_blocked_then_release",
            success=True,
            collision_count=0,
            min_overtake_attempt_count=1,
            min_overtake_success_count=1,
            require_positive_rejoin_wait=True,
        ),
    ),
)


INSPECT_ONLY_SCENARIOS: tuple[InspectOnlySuiteCase, ...] = (
    InspectOnlySuiteCase(
        name="near_junction_preflight_reject",
        command=(
            "uv",
            "run",
            "python",
            str(SUITE_DIR / "inspect_scenarios.py"),
            "--allow-invalid",
            str(_run_config("town01_stopped_obstacle_near_junction_preflight_reject_long_expert.json")),
        ),
    ),
)


SCENARIO_ORDER = tuple(scenario.case.name for scenario in ROUTE_LOOP_SCENARIOS)
