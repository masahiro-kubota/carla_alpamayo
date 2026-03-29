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


class StoppedObstacleSuiteAssertionsTest(unittest.TestCase):
    def test_accepts_passing_suite_fixture(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            base_dir = Path(tmp_dir)
            manifest_dir = base_dir / "manifests"
            manifest_dir.mkdir(parents=True, exist_ok=True)

            empty_manifest = manifest_dir / "empty.jsonl"
            empty_manifest.write_text("", encoding="utf-8")

            separated_manifest = manifest_dir / "separated.jsonl"
            separated_manifest.write_text(
                "\n".join(
                    [
                        json.dumps({"overtake_target_actor_id": 101}),
                        json.dumps({"overtake_target_actor_id": 202}),
                    ]
                ),
                encoding="utf-8",
            )

            clustered_manifest = manifest_dir / "clustered.jsonl"
            clustered_manifest.write_text(
                "\n".join(
                    [
                        json.dumps(
                            {
                                "overtake_target_kind": "cluster",
                                "overtake_target_member_actor_ids": [301, 302],
                            }
                        )
                    ]
                ),
                encoding="utf-8",
            )

            summary_paths: list[Path] = []
            for scenario_name in SCENARIO_ORDER:
                summary_path = base_dir / f"{scenario_name}.json"
                payload = {
                    "success": True,
                    "failure_reason": None,
                    "collision_count": 0,
                    "overtake_attempt_count": 1,
                    "overtake_success_count": 1,
                    "unsafe_lane_change_reject_count": 1,
                    "scenario_validation": {"valid": True, "errors": []},
                    "manifest_path": str(empty_manifest),
                    "rejoin_wait_after_target_passed_s": 0.5,
                }
                if scenario_name in {"blocked_static", "signal_suppressed", "adjacent_lane_closed"}:
                    payload["success"] = False
                    payload["failure_reason"] = "stalled"
                    payload["overtake_attempt_count"] = 0
                    payload["overtake_success_count"] = 0
                if scenario_name == "blocked_oncoming":
                    payload["unsafe_lane_change_reject_count"] = 2
                if scenario_name == "double_stopped_separated":
                    payload["overtake_attempt_count"] = 2
                    payload["overtake_success_count"] = 2
                    payload["manifest_path"] = str(separated_manifest)
                if scenario_name == "double_stopped_clustered":
                    payload["manifest_path"] = str(clustered_manifest)
                if scenario_name == "near_junction_preflight_reject":
                    payload["success"] = False
                    payload["failure_reason"] = "scenario_preflight_invalid"
                    payload["scenario_validation"] = {"valid": False, "errors": ["junction_nearby"]}
                summary_path.write_text(json.dumps(payload), encoding="utf-8")
                summary_paths.append(summary_path)

            summaries = load_stopped_obstacle_summaries(summary_paths)
            assert_stopped_obstacle_suite(summaries)

