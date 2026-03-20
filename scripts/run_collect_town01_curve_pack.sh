#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_ROOT}"

REPETITIONS="${CARLA_CURVE_PACK_REPETITIONS:-5}"
SEED_BASE="${CARLA_CURVE_PACK_SEED_BASE:-400}"
ROUTES=(
  "configs/routes/town01_curve_focus_early_left_exit.json"
  "configs/routes/town01_curve_focus_late_right_exit.json"
  "configs/routes/town01_curve_focus_bottom_right.json"
  "configs/routes/town01_curve_focus_top_right.json"
  "configs/routes/town01_curve_focus_top_left_entry.json"
  "configs/routes/town01_curve_focus_top_left_exit.json"
  "configs/routes/town01_curve_focus_lower_left.json"
)

run_index=0
for route_config in "${ROUTES[@]}"; do
  for ((repeat_index = 0; repeat_index < REPETITIONS; repeat_index += 1)); do
    seed=$((SEED_BASE + run_index))
    echo "Collecting ${route_config} repetition $((repeat_index + 1))/${REPETITIONS} with seed ${seed}"
    ./scripts/run_collect_town01_route.sh "${route_config}" --seed "${seed}" --no-record-video
    run_index=$((run_index + 1))
  done
done
