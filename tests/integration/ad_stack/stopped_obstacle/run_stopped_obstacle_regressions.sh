#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
cd "${ROOT_DIR}"

RUN_CONFIGS=(
  "tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_clear_long_expert.json"
  "tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_blocked_long_expert.json"
  "tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_blocked_oncoming_long_expert.json"
  "tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_double_stopped_separated_long_expert.json"
  "tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_double_stopped_clustered_long_expert.json"
)

SUMMARY_PATHS=()

for run_config in "${RUN_CONFIGS[@]}"; do
  echo "==> running ${run_config}"
  output="$(PYTHONPATH="" uv run python -m simulation.pipelines.run_route_loop "${run_config}")"
  echo "${output}"
  summary_path="$(RUN_OUTPUT="${output}" python - <<'PY'
import json
import os
from pathlib import Path

payload = json.loads(os.environ["RUN_OUTPUT"])
print(Path("outputs/evaluate") / payload["episode_id"] / "summary.json")
PY
)"
  SUMMARY_PATHS+=("${summary_path}")
  echo "    summary: ${summary_path}"
done

python - "${SUMMARY_PATHS[@]}" <<'PY'
import json
import sys
from pathlib import Path

summary_paths = [Path(item) for item in sys.argv[1:]]
if len(summary_paths) != 5:
    raise SystemExit(f"expected 5 summary paths, got {len(summary_paths)}")

(
    clear_summary,
    blocked_static_summary,
    blocked_oncoming_summary,
    double_stopped_separated_summary,
    double_stopped_clustered_summary,
) = [
    json.loads(path.read_text()) for path in summary_paths
]

def require(condition: bool, message: str) -> None:
    if not condition:
        raise SystemExit(message)

def load_manifest(summary: dict) -> list[dict]:
    return [
        json.loads(line)
        for line in Path(summary["manifest_path"]).read_text().splitlines()
        if line.strip()
    ]

require(bool(clear_summary["success"]), "clear scenario did not succeed")
require(clear_summary["collision_count"] == 0, "clear scenario collided")
require(clear_summary["overtake_attempt_count"] >= 1, "clear scenario never attempted overtake")
require(clear_summary["overtake_success_count"] >= 1, "clear scenario never completed overtake")

require(not blocked_static_summary["success"], "blocked_static unexpectedly succeeded")
require(
    blocked_static_summary["failure_reason"] == "stalled",
    f"blocked_static failure_reason unexpected: {blocked_static_summary['failure_reason']}",
)
require(blocked_static_summary["collision_count"] == 0, "blocked_static collided")
require(
    blocked_static_summary["overtake_attempt_count"] == 0,
    "blocked_static unexpectedly attempted overtake",
)
require(
    blocked_static_summary["unsafe_lane_change_reject_count"] >= 1,
    "blocked_static never recorded unsafe lane change reject",
)

require(bool(blocked_oncoming_summary["success"]), "blocked_oncoming did not succeed")
require(blocked_oncoming_summary["collision_count"] == 0, "blocked_oncoming collided")
require(
    blocked_oncoming_summary["unsafe_lane_change_reject_count"] >= 1,
    "blocked_oncoming never rejected initial overtake",
)
require(
    blocked_oncoming_summary["overtake_attempt_count"] >= 1,
    "blocked_oncoming never attempted overtake after waiting",
)
require(
    blocked_oncoming_summary["overtake_success_count"] >= 1,
    "blocked_oncoming never completed overtake",
)

require(bool(double_stopped_separated_summary["success"]), "double_stopped_separated did not succeed")
require(double_stopped_separated_summary["collision_count"] == 0, "double_stopped_separated collided")
require(
    double_stopped_separated_summary["overtake_attempt_count"] >= 2,
    "double_stopped_separated never attempted two overtakes",
)
require(
    double_stopped_separated_summary["overtake_success_count"] >= 2,
    "double_stopped_separated never completed two overtakes",
)
separated_manifest = load_manifest(double_stopped_separated_summary)
separated_targets = [
    row["overtake_target_actor_id"]
    for row in separated_manifest
    if row.get("overtake_target_actor_id") is not None
]
require(len(set(separated_targets)) >= 2, "double_stopped_separated never switched target actor")

require(bool(double_stopped_clustered_summary["success"]), "double_stopped_clustered did not succeed")
require(double_stopped_clustered_summary["collision_count"] == 0, "double_stopped_clustered collided")
require(
    double_stopped_clustered_summary["overtake_attempt_count"] >= 1,
    "double_stopped_clustered never attempted overtake",
)
require(
    double_stopped_clustered_summary["overtake_success_count"] >= 1,
    "double_stopped_clustered never completed overtake",
)
clustered_manifest = load_manifest(double_stopped_clustered_summary)
require(
    any(row.get("overtake_target_kind") == "cluster" for row in clustered_manifest),
    "double_stopped_clustered never reported cluster target kind",
)
require(
    any(len(row.get("overtake_target_member_actor_ids") or []) >= 2 for row in clustered_manifest),
    "double_stopped_clustered never kept multi-actor cluster members",
)

print("stopped-obstacle regressions passed")
for path in summary_paths:
    print(path)
PY
