from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from tests.integration.ad_stack.stopped_obstacle.assertions import (
    SCENARIO_ORDER,
    assert_stopped_obstacle_suite,
    load_stopped_obstacle_summaries,
)
from tests.integration.ad_stack.stopped_obstacle.scenario_matrix import ROUTE_LOOP_SCENARIOS


class StoppedObstacleSuiteAssertionsTest(unittest.TestCase):
    def test_accepts_passing_suite_fixture(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            base_dir = Path(tmp_dir)
            manifest_dir = base_dir / "manifests"
            manifest_dir.mkdir(parents=True, exist_ok=True)

            summary_paths: list[Path] = []
            scenario_by_name = {scenario.case.name: scenario for scenario in ROUTE_LOOP_SCENARIOS}
            for scenario_name in SCENARIO_ORDER:
                scenario = scenario_by_name[scenario_name]
                summary_path = base_dir / f"{scenario_name}.json"
                expectation = scenario.summary_expectation
                payload = {
                    "success": expectation.success if expectation.success is not None else True,
                    "failure_reason": expectation.failure_reason,
                    "collision_count": (
                        expectation.collision_count if expectation.collision_count is not None else 0
                    ),
                    "average_speed_kmh": (
                        expectation.min_average_speed_kmh
                        if expectation.min_average_speed_kmh is not None
                        else 30.0
                    ),
                    "overtake_attempt_count": (
                        expectation.exact_overtake_attempt_count
                        if expectation.exact_overtake_attempt_count is not None
                        else (
                            expectation.min_overtake_attempt_count
                            if expectation.min_overtake_attempt_count is not None
                            else 1
                        )
                    ),
                    "overtake_success_count": (
                        expectation.exact_overtake_success_count
                        if expectation.exact_overtake_success_count is not None
                        else (
                            expectation.min_overtake_success_count
                            if expectation.min_overtake_success_count is not None
                            else 1
                        )
                    ),
                    "unsafe_lane_change_reject_count": (
                        expectation.min_unsafe_lane_change_reject_count
                        if expectation.min_unsafe_lane_change_reject_count is not None
                        else 0
                    ),
                    "rejoin_wait_after_target_passed_s": (
                        0.5 if expectation.require_positive_rejoin_wait else None
                    ),
                }
                manifest_rows: list[dict[str, object]] = []
                for manifest_expectation in scenario.manifest_expectations:
                    if manifest_expectation.kind == "min_unique_non_null":
                        min_unique = manifest_expectation.min_unique or 0
                        manifest_rows.extend(
                            {
                                manifest_expectation.field: 100 + index,
                            }
                            for index in range(min_unique)
                        )
                    elif manifest_expectation.kind == "any_equals":
                        manifest_rows.append(
                            {
                                manifest_expectation.field: manifest_expectation.expected,
                            }
                        )
                    elif manifest_expectation.kind == "any_sequence_len_at_least":
                        min_len = manifest_expectation.min_len or 0
                        manifest_rows.append(
                            {
                                manifest_expectation.field: list(range(min_len)),
                            }
                        )
                    elif manifest_expectation.kind == "none_equals":
                        manifest_rows.append(
                            {
                                manifest_expectation.field: None,
                            }
                        )
                    elif manifest_expectation.kind == "min_numeric_where_equals":
                        manifest_rows.append(
                            {
                                manifest_expectation.filter_field: manifest_expectation.filter_equals,
                                manifest_expectation.field: manifest_expectation.min_value,
                            }
                        )
                manifest_path = manifest_dir / f"{scenario_name}.jsonl"
                manifest_path.write_text(
                    "\n".join(json.dumps(row) for row in manifest_rows),
                    encoding="utf-8",
                )
                payload["manifest_path"] = str(manifest_path)
                summary_path.write_text(json.dumps(payload), encoding="utf-8")
                summary_paths.append(summary_path)

            summaries = load_stopped_obstacle_summaries(summary_paths)
            assert_stopped_obstacle_suite(summaries)
