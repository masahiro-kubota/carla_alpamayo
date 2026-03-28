#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "${ROOT_DIR}"

RUN_CONFIGS=(
  "tests/integration/stopped_obstacle/run_configs/town01_stopped_obstacle_clear_long_expert.json"
  "tests/integration/stopped_obstacle/run_configs/town01_stopped_obstacle_blocked_long_expert.json"
  "tests/integration/stopped_obstacle/run_configs/town01_stopped_obstacle_blocked_oncoming_long_expert.json"
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
if len(summary_paths) != 3:
    raise SystemExit(f"expected 3 summary paths, got {len(summary_paths)}")

clear_summary, blocked_static_summary, blocked_oncoming_summary = [
    json.loads(path.read_text()) for path in summary_paths
]

def require(condition: bool, message: str) -> None:
    if not condition:
        raise SystemExit(message)

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

print("stopped-obstacle regressions passed")
for path in summary_paths:
    print(path)
PY
