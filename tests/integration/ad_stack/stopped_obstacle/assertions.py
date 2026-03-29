from __future__ import annotations

from pathlib import Path
from typing import Any

from tests.integration.ad_stack._shared import (
    assert_manifest_expectations,
    assert_summary_expectations,
    load_manifest,
    load_ordered_summaries,
)
from .scenario_matrix import ROUTE_LOOP_SCENARIOS, SCENARIO_ORDER


def load_stopped_obstacle_summaries(
    summary_paths: list[str] | tuple[str, ...] | list[Path] | tuple[Path, ...],
) -> dict[str, dict[str, Any]]:
    return load_ordered_summaries(
        summary_paths,
        scenario_order=SCENARIO_ORDER,
    )


def assert_stopped_obstacle_suite(summaries: dict[str, dict[str, Any]]) -> None:
    assert_summary_expectations(
        summaries,
        tuple(scenario.summary_expectation for scenario in ROUTE_LOOP_SCENARIOS),
    )

    for scenario in ROUTE_LOOP_SCENARIOS:
        if not scenario.manifest_expectations:
            continue
        assert_manifest_expectations(
            load_manifest(summaries[scenario.case.name]),
            scenario.manifest_expectations,
        )


def assert_near_junction_preflight_contract(validation_payload: dict[str, Any]) -> None:
    assert not bool(validation_payload.get("valid", True)), (
        "near_junction_preflight_reject should fail preflight"
    )
    assert "junction_nearby" in tuple(validation_payload.get("errors", ())), (
        "near_junction_preflight_reject missing junction_nearby validation error"
    )
