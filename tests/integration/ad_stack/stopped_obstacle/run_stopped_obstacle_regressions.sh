#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
cd "${ROOT_DIR}"

CARLA_ROOT="${CARLA_ROOT:-/media/masa/ssd_data/sim/carla-0.9.16}"
CARLA_LAUNCH_ARGS=(${CARLA_LAUNCH_ARGS:-"-quality-level=Low -RenderOffScreen -nosound"})
CARLA_PID=""

RUN_CONFIGS=(
  "tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_clear_long_expert.json"
  "tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_blocked_long_expert.json"
  "tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_blocked_oncoming_long_expert.json"
  "tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_double_stopped_separated_long_expert.json"
  "tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_double_stopped_clustered_long_expert.json"
  "tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_signal_suppressed_long_expert.json"
  "tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_near_junction_preflight_reject_long_expert.json"
  "tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_adjacent_lane_closed_long_expert.json"
  "tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_curve_clear_long_expert.json"
  "tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_rejoin_blocked_then_release_long_expert.json"
)

SUMMARY_PATHS=()

wait_for_carla() {
  local timeout_s=60
  local elapsed=0
  while (( elapsed < timeout_s )); do
    if ss -ltnp | rg -q ':2000|:2001|:2002'; then
      return 0
    fi
    sleep 1
    ((elapsed+=1))
  done
  return 1
}

wait_for_carla_exit() {
  local timeout_s=30
  local elapsed=0
  while (( elapsed < timeout_s )); do
    if ! ss -ltnp | rg -q ':2000|:2001|:2002'; then
      return 0
    fi
    sleep 1
    ((elapsed+=1))
  done
  return 1
}

stop_carla() {
  if [[ -n "${CARLA_PID}" ]] && kill -0 "${CARLA_PID}" 2>/dev/null; then
    kill "${CARLA_PID}" 2>/dev/null || true
    wait "${CARLA_PID}" 2>/dev/null || true
  fi
  pkill -f 'CarlaUE4-Linux-Shipping|CarlaUE4.sh' 2>/dev/null || true
  wait_for_carla_exit || true
  CARLA_PID=""
}

start_carla() {
  if ss -ltnp | rg -q ':2000|:2001|:2002'; then
    echo "CARLA ports 2000/2001/2002 are already in use" >&2
    return 1
  fi
  "${CARLA_ROOT}/CarlaUE4.sh" ${CARLA_LAUNCH_ARGS[@]} >/tmp/carla_stopped_obstacle_regression.log 2>&1 &
  CARLA_PID=$!
  wait_for_carla
}

trap stop_carla EXIT

for run_config in "${RUN_CONFIGS[@]}"; do
  echo "==> running ${run_config}"
  start_carla
  output="$(PYTHONPATH="" uv run python -m simulation.pipelines.run_route_loop "${run_config}")"
  echo "${output}"
  stop_carla
  summary_path="$(RUN_OUTPUT="${output}" python - <<'PY'
import os
from tests.integration.ad_stack._shared import summary_path_from_run_output

print(summary_path_from_run_output(os.environ["RUN_OUTPUT"]))
PY
)"
  SUMMARY_PATHS+=("${summary_path}")
  echo "    summary: ${summary_path}"
done

python - "${SUMMARY_PATHS[@]}" <<'PY'
import sys
from tests.integration.ad_stack._shared import load_manifest, load_summary, require

summary_paths = sys.argv[1:]
if len(summary_paths) != 10:
    raise SystemExit(f"expected 10 summary paths, got {len(summary_paths)}")

(
    clear_summary,
    blocked_static_summary,
    blocked_oncoming_summary,
    double_stopped_separated_summary,
    double_stopped_clustered_summary,
    signal_suppressed_summary,
    near_junction_summary,
    adjacent_lane_closed_summary,
    curve_clear_summary,
    rejoin_blocked_then_release_summary,
) = [
    load_summary(path) for path in summary_paths
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

require(
    not signal_suppressed_summary["success"],
    "signal_suppressed unexpectedly succeeded",
)
require(
    signal_suppressed_summary["failure_reason"] == "stalled",
    f"signal_suppressed failure_reason unexpected: {signal_suppressed_summary['failure_reason']}",
)
require(
    signal_suppressed_summary["collision_count"] == 0,
    "signal_suppressed collided",
)
require(
    signal_suppressed_summary["overtake_attempt_count"] == 0,
    "signal_suppressed unexpectedly attempted overtake",
)
require(
    signal_suppressed_summary["scenario_validation"]["valid"] is True,
    "signal_suppressed scenario validation was not valid",
)

require(
    not near_junction_summary["success"],
    "near_junction_preflight_reject unexpectedly succeeded",
)
require(
    near_junction_summary["failure_reason"] == "scenario_preflight_invalid",
    f"near_junction_preflight_reject failure_reason unexpected: {near_junction_summary['failure_reason']}",
)
require(
    near_junction_summary["scenario_validation"]["valid"] is False,
    "near_junction_preflight_reject scenario validation unexpectedly valid",
)
require(
    "junction_nearby" in near_junction_summary["scenario_validation"]["errors"],
    "near_junction_preflight_reject did not report junction_nearby",
)

require(
    not adjacent_lane_closed_summary["success"],
    "adjacent_lane_closed unexpectedly succeeded",
)
require(
    adjacent_lane_closed_summary["failure_reason"] == "stalled",
    f"adjacent_lane_closed failure_reason unexpected: {adjacent_lane_closed_summary['failure_reason']}",
)
require(
    adjacent_lane_closed_summary["collision_count"] == 0,
    "adjacent_lane_closed collided",
)
require(
    adjacent_lane_closed_summary["overtake_attempt_count"] == 0,
    "adjacent_lane_closed unexpectedly attempted overtake",
)
require(
    adjacent_lane_closed_summary["unsafe_lane_change_reject_count"] >= 1,
    "adjacent_lane_closed never rejected an unsafe lane change",
)
adjacent_manifest = load_manifest(adjacent_lane_closed_summary)
require(
    any(row.get("overtake_reject_reason") == "adjacent_lane_closed" for row in adjacent_manifest),
    "adjacent_lane_closed never reported adjacent_lane_closed reject reason",
)

require(bool(curve_clear_summary["success"]), "curve_clear did not succeed")
require(curve_clear_summary["collision_count"] == 0, "curve_clear collided")
require(
    curve_clear_summary["overtake_attempt_count"] >= 1,
    "curve_clear never attempted overtake",
)
require(
    curve_clear_summary["overtake_success_count"] >= 1,
    "curve_clear never completed overtake",
)

require(
    bool(rejoin_blocked_then_release_summary["success"]),
    "rejoin_blocked_then_release did not succeed",
)
require(
    rejoin_blocked_then_release_summary["collision_count"] == 0,
    "rejoin_blocked_then_release collided",
)
require(
    rejoin_blocked_then_release_summary["overtake_attempt_count"] >= 1,
    "rejoin_blocked_then_release never attempted overtake",
)
require(
    rejoin_blocked_then_release_summary["overtake_success_count"] >= 1,
    "rejoin_blocked_then_release never completed overtake",
)
require(
    rejoin_blocked_then_release_summary["first_target_passed_s"] is not None,
    "rejoin_blocked_then_release never reported target_passed",
)
require(
    rejoin_blocked_then_release_summary["first_rejoin_started_s"] is not None,
    "rejoin_blocked_then_release never reported rejoin start",
)
require(
    rejoin_blocked_then_release_summary["rejoin_wait_after_target_passed_s"] is not None
    and rejoin_blocked_then_release_summary["rejoin_wait_after_target_passed_s"] > 0.0,
    "rejoin_blocked_then_release did not wait after target_passed",
)
require(
    rejoin_blocked_then_release_summary["first_rejoin_rear_gap_m"] is not None
    and rejoin_blocked_then_release_summary["first_rejoin_rear_gap_m"] >= 15.0,
    "rejoin_blocked_then_release rejoined before rear gap opened",
)

print("stopped-obstacle regressions passed")
for path in summary_paths:
    print(path)
PY
