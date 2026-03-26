#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"

declare -a ROUTE_SPECS=(
  "data_collection/configs/routes/town01_right_focus_se.json:4"
  "data_collection/configs/routes/town01_right_focus_ne.json:4"
  "data_collection/configs/routes/town01_right_focus_sw.json:3"
  "data_collection/configs/routes/town01_straight_focus_south.json:5"
  "data_collection/configs/routes/town01_straight_focus_east.json:5"
  "data_collection/configs/routes/town01_straight_focus_north.json:5"
  "data_collection/configs/routes/town01_straight_focus_west.json:5"
)

record_video_flag="--no-record-video"
if [[ "${CARLA_RECORD_VIDEO:-0}" == "1" ]]; then
  record_video_flag="--record-video"
fi

seed="${CARLA_BASE_SEED:-100}"

for spec in "${ROUTE_SPECS[@]}"; do
  route_config="${spec%%:*}"
  repeats="${spec##*:}"
  route_name="$(basename "${route_config}" .json)"
  for ((run_idx = 1; run_idx <= repeats; run_idx++)); do
    echo "==> collecting ${route_name} (${run_idx}/${repeats}) with seed ${seed}"
    "${SCRIPT_DIR}/run_collect_town01_route.sh" \
      "${route_config}" \
      "${record_video_flag}" \
      --seed "${seed}"
    seed=$((seed + 1))
  done
done
