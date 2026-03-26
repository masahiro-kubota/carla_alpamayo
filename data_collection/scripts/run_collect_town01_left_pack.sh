#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

cd "${REPO_ROOT}"

declare -a ROUTE_RUNS=(
  "data_collection/configs/routes/town01_left_focus_south.json:4"
  "data_collection/configs/routes/town01_left_focus_sw.json:4"
  "data_collection/configs/routes/town01_left_focus_ne.json:4"
  "data_collection/configs/routes/town01_left_focus_west.json:3"
)

for spec in "${ROUTE_RUNS[@]}"; do
  route_config="${spec%%:*}"
  episode_count="${spec##*:}"
  route_name="$(basename "${route_config}" .json)"
  echo "== ${route_name} x ${episode_count} =="
  for ((episode_index = 1; episode_index <= episode_count; episode_index++)); do
    echo "-- episode ${episode_index}/${episode_count}"
    "${SCRIPT_DIR}/run_collect_town01_route.sh" "${route_config}" --no-record-video
  done
done
