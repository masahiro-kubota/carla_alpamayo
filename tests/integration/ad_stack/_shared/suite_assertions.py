from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

from .summary_tools import load_summary, require


@dataclass(frozen=True, slots=True)
class ScenarioSummaryExpectation:
    name: str
    success: bool | None = None
    failure_reason: str | None = None
    collision_count: int | None = None
    min_overtake_attempt_count: int | None = None
    exact_overtake_attempt_count: int | None = None
    min_overtake_success_count: int | None = None
    exact_overtake_success_count: int | None = None
    min_unsafe_lane_change_reject_count: int | None = None
    scenario_validation_valid: bool | None = None
    required_validation_errors: tuple[str, ...] = ()
    require_positive_rejoin_wait: bool = False


def load_ordered_summaries(
    summary_paths: list[str] | tuple[str, ...] | list[Path] | tuple[Path, ...],
    *,
    scenario_order: Iterable[str],
) -> dict[str, dict[str, Any]]:
    scenario_names = tuple(scenario_order)
    require(
        len(summary_paths) == len(scenario_names),
        f"expected {len(scenario_names)} summary paths, got {len(summary_paths)}",
    )
    return {
        scenario_name: load_summary(path)
        for scenario_name, path in zip(scenario_names, summary_paths, strict=True)
    }


def assert_summary_expectations(
    summaries: dict[str, dict[str, Any]],
    expectations: Iterable[ScenarioSummaryExpectation],
) -> None:
    for expectation in expectations:
        summary = summaries[expectation.name]
        if expectation.success is not None:
            require(
                bool(summary["success"]) is expectation.success,
                f"{expectation.name} success expected {expectation.success} got {summary['success']}",
            )
        if expectation.failure_reason is not None:
            require(
                summary["failure_reason"] == expectation.failure_reason,
                f"{expectation.name} failure_reason unexpected: {summary['failure_reason']}",
            )
        if expectation.collision_count is not None:
            require(
                summary["collision_count"] == expectation.collision_count,
                f"{expectation.name} collision_count unexpected: {summary['collision_count']}",
            )
        if expectation.min_overtake_attempt_count is not None:
            require(
                summary["overtake_attempt_count"] >= expectation.min_overtake_attempt_count,
                f"{expectation.name} overtake_attempt_count too small: {summary['overtake_attempt_count']}",
            )
        if expectation.exact_overtake_attempt_count is not None:
            require(
                summary["overtake_attempt_count"] == expectation.exact_overtake_attempt_count,
                f"{expectation.name} overtake_attempt_count unexpected: {summary['overtake_attempt_count']}",
            )
        if expectation.min_overtake_success_count is not None:
            require(
                summary["overtake_success_count"] >= expectation.min_overtake_success_count,
                f"{expectation.name} overtake_success_count too small: {summary['overtake_success_count']}",
            )
        if expectation.exact_overtake_success_count is not None:
            require(
                summary["overtake_success_count"] == expectation.exact_overtake_success_count,
                f"{expectation.name} overtake_success_count unexpected: {summary['overtake_success_count']}",
            )
        if expectation.min_unsafe_lane_change_reject_count is not None:
            require(
                summary["unsafe_lane_change_reject_count"]
                >= expectation.min_unsafe_lane_change_reject_count,
                f"{expectation.name} unsafe_lane_change_reject_count too small: "
                f"{summary['unsafe_lane_change_reject_count']}",
            )
        if expectation.scenario_validation_valid is not None:
            require(
                bool(summary["scenario_validation"]["valid"]) is expectation.scenario_validation_valid,
                f"{expectation.name} scenario_validation.valid unexpected: "
                f"{summary['scenario_validation']['valid']}",
            )
        for error in expectation.required_validation_errors:
            require(
                error in summary["scenario_validation"]["errors"],
                f"{expectation.name} missing scenario_validation error: {error}",
            )
        if expectation.require_positive_rejoin_wait:
            wait_after_pass_s = summary.get("rejoin_wait_after_target_passed_s")
            require(
                wait_after_pass_s is not None and wait_after_pass_s > 0.0,
                f"{expectation.name} never waited after passing target",
            )
