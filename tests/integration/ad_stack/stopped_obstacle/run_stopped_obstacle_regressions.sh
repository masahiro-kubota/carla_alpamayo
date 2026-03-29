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
from tests.integration.ad_stack.stopped_obstacle.assertions import (
    assert_stopped_obstacle_suite,
    load_stopped_obstacle_summaries,
)

summary_paths = sys.argv[1:]
assert_stopped_obstacle_suite(load_stopped_obstacle_summaries(summary_paths))

print("stopped-obstacle regressions passed")
for path in summary_paths:
    print(path)
PY
