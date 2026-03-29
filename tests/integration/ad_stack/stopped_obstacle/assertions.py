from __future__ import annotations

from pathlib import Path
from typing import Any

from tests.integration.ad_stack._shared import (
    ScenarioSummaryExpectation,
    assert_summary_expectations,
    load_manifest,
    load_ordered_summaries,
    require,
)

SCENARIO_ORDER = (
    "clear",
    "blocked_static",
    "blocked_oncoming",
    "double_stopped_separated",
    "double_stopped_clustered",
    "signal_suppressed",
    "near_junction_preflight_reject",
    "adjacent_lane_closed",
    "curve_clear",
    "rejoin_blocked_then_release",
)

SUMMARY_EXPECTATIONS = (
    ScenarioSummaryExpectation(
        name="clear",
        success=True,
        collision_count=0,
        min_overtake_attempt_count=1,
        min_overtake_success_count=1,
    ),
    ScenarioSummaryExpectation(
        name="blocked_static",
        success=False,
        failure_reason="stalled",
        collision_count=0,
        exact_overtake_attempt_count=0,
        min_unsafe_lane_change_reject_count=1,
    ),
    ScenarioSummaryExpectation(
        name="blocked_oncoming",
        success=True,
        collision_count=0,
        min_overtake_attempt_count=1,
        min_overtake_success_count=1,
        min_unsafe_lane_change_reject_count=1,
    ),
    ScenarioSummaryExpectation(
        name="double_stopped_separated",
        success=True,
        collision_count=0,
        min_overtake_attempt_count=2,
        min_overtake_success_count=2,
    ),
    ScenarioSummaryExpectation(
        name="double_stopped_clustered",
        success=True,
        collision_count=0,
        min_overtake_attempt_count=1,
        min_overtake_success_count=1,
    ),
    ScenarioSummaryExpectation(
        name="signal_suppressed",
        success=False,
        failure_reason="stalled",
        collision_count=0,
        exact_overtake_attempt_count=0,
        scenario_validation_valid=True,
    ),
    ScenarioSummaryExpectation(
        name="near_junction_preflight_reject",
        success=False,
        failure_reason="scenario_preflight_invalid",
        scenario_validation_valid=False,
        required_validation_errors=("junction_nearby",),
    ),
    ScenarioSummaryExpectation(
        name="adjacent_lane_closed",
        success=False,
        failure_reason="stalled",
        collision_count=0,
        exact_overtake_attempt_count=0,
        min_unsafe_lane_change_reject_count=1,
    ),
    ScenarioSummaryExpectation(
        name="curve_clear",
        success=True,
        collision_count=0,
        min_overtake_attempt_count=1,
        min_overtake_success_count=1,
    ),
    ScenarioSummaryExpectation(
        name="rejoin_blocked_then_release",
        success=True,
        collision_count=0,
        min_overtake_attempt_count=1,
        min_overtake_success_count=1,
        require_positive_rejoin_wait=True,
    ),
)


def load_stopped_obstacle_summaries(
    summary_paths: list[str] | tuple[str, ...] | list[Path] | tuple[Path, ...],
) -> dict[str, dict[str, Any]]:
    return load_ordered_summaries(
        summary_paths,
        scenario_order=SCENARIO_ORDER,
    )


def assert_stopped_obstacle_suite(summaries: dict[str, dict[str, Any]]) -> None:
    assert_summary_expectations(summaries, SUMMARY_EXPECTATIONS)

    double_stopped_separated_summary = summaries["double_stopped_separated"]
    separated_manifest = load_manifest(double_stopped_separated_summary)
    separated_targets = [
        row["overtake_target_actor_id"]
        for row in separated_manifest
        if row.get("overtake_target_actor_id") is not None
    ]
    require(
        len(set(separated_targets)) >= 2,
        "double_stopped_separated never switched target actor",
    )

    double_stopped_clustered_summary = summaries["double_stopped_clustered"]
    clustered_manifest = load_manifest(double_stopped_clustered_summary)
    require(
        any(row.get("overtake_target_kind") == "cluster" for row in clustered_manifest),
        "double_stopped_clustered never reported cluster target kind",
    )
    require(
        any(len(row.get("overtake_target_member_actor_ids") or []) >= 2 for row in clustered_manifest),
        "double_stopped_clustered never kept multi-actor cluster members",
    )
