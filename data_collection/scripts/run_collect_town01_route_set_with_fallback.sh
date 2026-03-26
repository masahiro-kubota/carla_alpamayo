#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

cd "${REPO_ROOT}"

route_dir="${TOWN01_ROUTE_SET_DIR:-data_collection/configs/routes/town01_movement_eval}"
route_glob="${TOWN01_ROUTE_SET_GLOB:-*.json}"
speed_list="${TOWN01_ROUTE_SET_SPEEDS:-30,25,20}"
repeats="${TOWN01_ROUTE_SET_REPEATS:-1}"
seed_base="${TOWN01_ROUTE_SET_SEED_BASE:-7}"
max_routes="${TOWN01_ROUTE_SET_MAX_ROUTES:-0}"
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
    --speed-list)
      speed_list="$2"
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

IFS=',' read -r -a speeds <<< "${speed_list}"
if [[ "${#speeds[@]}" -eq 0 ]]; then
  echo "No fallback speeds were configured." >&2
  exit 1
fi

mapfile -t route_configs < <(find "${route_dir}" -maxdepth 1 -type f -name "${route_glob}" | sort)
if [[ "${#route_configs[@]}" -eq 0 ]]; then
  echo "No route configs matched ${route_dir}/${route_glob}" >&2
  exit 1
fi

if (( max_routes > 0 && ${#route_configs[@]} > max_routes )); then
  route_configs=("${route_configs[@]:0:max_routes}")
fi

declare -a failed_runs=()
declare -a successful_manifests=()
declare -a successful_summaries=()

total_runs=$(( ${#route_configs[@]} * repeats ))
run_index=0

for route_config in "${route_configs[@]}"; do
  route_name="$(basename "${route_config}" .json)"
  for (( repeat_index = 0; repeat_index < repeats; repeat_index++ )); do
    run_index=$((run_index + 1))
    seed=$((seed_base + run_index - 1))
    run_succeeded=0

    for speed in "${speeds[@]}"; do
      echo "[${run_index}/${total_runs}] collecting ${route_name} repeat=$((repeat_index + 1)) seed=${seed} speed=${speed}km/h"
      log_file="$(mktemp)"

      if "${SCRIPT_DIR}/run_collect_town01_route.sh" \
        "${route_config}" \
        "${record_video_flag}" \
        --seed "${seed}" \
        --target-speed-kmh "${speed}" \
        "${extra_args[@]}" | tee "${log_file}"; then
        mapfile -t parsed < <(python3 - <<'PY' "${log_file}"
import json
import sys

with open(sys.argv[1], "r", encoding="utf-8") as handle:
    summary = json.load(handle)

print("true" if summary["success"] else "false")
print(f"outputs/collect/{summary['episode_id']}/summary.json")
print(summary["manifest_path"])
print(summary["target_speed_kmh"])
PY
)

        success="${parsed[0]}"
        summary_path="${parsed[1]}"
        manifest_path="${parsed[2]}"
        target_speed="${parsed[3]}"

        if [[ "${success}" == "true" ]]; then
          run_succeeded=1
          successful_summaries+=("${summary_path}")
          successful_manifests+=("${manifest_path}")
          echo "  -> success at ${target_speed}km/h"
          break
        fi

        echo "  -> collected failure run at ${target_speed}km/h, retrying if fallback speeds remain"
      else
        echo "  -> collection command failed for ${route_name} at ${speed}km/h" >&2
      fi
    done

    if (( run_succeeded == 0 )); then
      failed_runs+=("${route_name}:repeat$((repeat_index + 1))")
    fi
  done
done

python3 - <<'PY' "${#route_configs[@]}" "${repeats}" "${#successful_manifests[@]}" "${#failed_runs[@]}" "${successful_manifests[@]}" -- "${failed_runs[@]}"
import json
import sys

route_count = int(sys.argv[1])
repeats = int(sys.argv[2])
success_count = int(sys.argv[3])
failure_count = int(sys.argv[4])
args = sys.argv[5:]
split_index = args.index("--")
successful_manifests = args[:split_index]
failed_runs = args[split_index + 1 :]

print(
    json.dumps(
        {
            "route_count": route_count,
            "repeats": repeats,
            "attempt_count": route_count * repeats,
            "success_count": success_count,
            "failure_count": failure_count,
            "successful_manifests": successful_manifests,
            "failed_runs": failed_runs,
        },
        indent=2,
    )
)
PY

if (( ${#failed_runs[@]} > 0 )); then
  exit 1
fi
