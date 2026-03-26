#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

cd "${REPO_ROOT}"

route_dir="${TOWN01_MOVEMENT_TRAIN_DIR:-data_collection/configs/routes/town01_movement_train}"
route_glob="${TOWN01_MOVEMENT_ROUTE_GLOB:-*.json}"
repeats="${TOWN01_MOVEMENT_REPEATS:-1}"
seed_base="${TOWN01_MOVEMENT_SEED_BASE:-7}"
max_routes="${TOWN01_MOVEMENT_MAX_ROUTES:-0}"
record_video_flag="--no-record-video"
extra_args=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --route-dir)
      route_dir="$2"
      shift 2
      ;;
    --route-glob)
      route_glob="$2"
      shift 2
      ;;
    --repeats)
      repeats="$2"
      shift 2
      ;;
    --seed-base)
      seed_base="$2"
      shift 2
      ;;
    --max-routes)
      max_routes="$2"
      shift 2
      ;;
    --record-video)
      record_video_flag="--record-video"
      shift
      ;;
    --no-record-video)
      record_video_flag="--no-record-video"
      shift
      ;;
    --)
      shift
      extra_args+=("$@")
      break
      ;;
    *)
      extra_args+=("$1")
      shift
      ;;
  esac
done

mapfile -t route_configs < <(find "${route_dir}" -maxdepth 1 -type f -name "${route_glob}" | sort)
if [[ "${#route_configs[@]}" -eq 0 ]]; then
  echo "No route configs matched ${route_dir}/${route_glob}" >&2
  exit 1
fi

if (( max_routes > 0 && ${#route_configs[@]} > max_routes )); then
  route_configs=("${route_configs[@]:0:max_routes}")
fi

total_runs=$(( ${#route_configs[@]} * repeats ))
run_index=0

for route_config in "${route_configs[@]}"; do
  for (( repeat_index = 0; repeat_index < repeats; repeat_index++ )); do
    run_index=$((run_index + 1))
    seed=$((seed_base + run_index - 1))
    echo "[${run_index}/${total_runs}] collecting $(basename "${route_config}") repeat=$((repeat_index + 1)) seed=${seed}"
    "${SCRIPT_DIR}/run_collect_town01_route.sh" \
      "${route_config}" \
      "${record_video_flag}" \
      --seed "${seed}" \
      "${extra_args[@]}"
  done
done
