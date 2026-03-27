#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

cd "${REPO_ROOT}"

if ! command -v uv >/dev/null 2>&1; then
  echo "uv is required but was not found in PATH." >&2
  exit 1
fi

usage() {
  cat <<'EOF'
Usage:
  ./simulation/scripts/run_expert_town01_collection_triplet.sh

This wrapper launches 3 fixed expert route-loop workers in parallel.
It accepts no positional CLI overrides.

Fixed worker allocation:
  worker 1 -> port 2000 -> scenarios/routes/town01_perimeter_cw.json
  worker 2 -> port 2004 -> scenarios/routes/town01_intersection_weave_cw.json
  worker 3 -> port 2008 -> scenarios/routes/town01_intersection_weave_ccw.json

Configure it via environment variables instead:
  CARLA_HOST
  CARLA_ENVIRONMENT_CONFIG
  CARLA_EXPERT_CONFIG
  CARLA_CAMERA_WIDTH
  CARLA_CAMERA_HEIGHT
  CARLA_CAMERA_FOV
  CARLA_MCAP_SEGMENT_SECONDS
EOF
}

if [[ $# -gt 0 ]]; then
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unexpected argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
fi

HOST="${CARLA_HOST:-127.0.0.1}"
ENVIRONMENT_CONFIG="${CARLA_ENVIRONMENT_CONFIG:-scenarios/environments/town01_all_intersections_clear.json}"
EXPERT_CONFIG="${CARLA_EXPERT_CONFIG:-ad_stack/configs/expert/no_overtake.json}"
CAMERA_WIDTH="${CARLA_CAMERA_WIDTH:-1280}"
CAMERA_HEIGHT="${CARLA_CAMERA_HEIGHT:-720}"
CAMERA_FOV="${CARLA_CAMERA_FOV:-60}"
MCAP_SEGMENT_SECONDS="${CARLA_MCAP_SEGMENT_SECONDS:-600}"

ROUTES=(
  "scenarios/routes/town01_perimeter_cw.json"
  "scenarios/routes/town01_intersection_weave_cw.json"
  "scenarios/routes/town01_intersection_weave_ccw.json"
)
PORTS=(2000 2004 2008)

RUN_ID="$(date +%Y%m%d_%H%M%S)_town01_collection_triplet"
LOG_DIR="${REPO_ROOT}/outputs/launcher_runs/${RUN_ID}"
mkdir -p "${LOG_DIR}"

declare -a PIDS=()
declare -a EXIT_CODES=()

cleanup_children() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill "${pid}" >/dev/null 2>&1 || true
    fi
  done
}

trap cleanup_children INT TERM

for index in "${!ROUTES[@]}"; do
  worker_id="$((index + 1))"
  route_config="${ROUTES[$index]}"
  port="${PORTS[$index]}"
  log_path="${LOG_DIR}/worker_${worker_id}.log"

  echo "Launching worker ${worker_id}: port=${port} route=${route_config} log=${log_path}"

  (
    PYTHONPATH="" uv run python -m simulation.pipelines.run_route_loop \
      --policy-kind expert \
      --host "${HOST}" \
      --port "${port}" \
      --route-config "${route_config}" \
      --environment-config "${ENVIRONMENT_CONFIG}" \
      --expert-config "${EXPERT_CONFIG}" \
      --camera-width "${CAMERA_WIDTH}" \
      --camera-height "${CAMERA_HEIGHT}" \
      --camera-fov "${CAMERA_FOV}" \
      --mcap-segment-seconds "${MCAP_SEGMENT_SECONDS}" \
      --ignore-traffic-lights \
      --no-record-video
  ) >"${log_path}" 2>&1 &

  PIDS+=("$!")
done

for index in "${!PIDS[@]}"; do
  pid="${PIDS[$index]}"
  if wait "${pid}"; then
    EXIT_CODES+=(0)
  else
    EXIT_CODES+=($?)
  fi
done

failed=0
for index in "${!EXIT_CODES[@]}"; do
  worker_id="$((index + 1))"
  exit_code="${EXIT_CODES[$index]}"
  log_path="${LOG_DIR}/worker_${worker_id}.log"
  if [[ "${exit_code}" -ne 0 ]]; then
    failed=1
    echo "Worker ${worker_id} failed with exit code ${exit_code}. See ${log_path}" >&2
  else
    echo "Worker ${worker_id} finished successfully. See ${log_path}"
  fi
done

if [[ "${failed}" -ne 0 ]]; then
  exit 1
fi

echo "All workers finished successfully. Logs: ${LOG_DIR}"
