#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

cd "${REPO_ROOT}"

if ! command -v uv >/dev/null 2>&1; then
  echo "uv is required but was not found in PATH." >&2
  exit 1
fi

resolve_carla_root() {
  if [[ -n "${CARLA_ROOT:-}" ]]; then
    echo "${CARLA_ROOT}"
    return 0
  fi
  local candidates=(
    "/media/masa/ssd_data/sim/carla-0.9.16"
    "${HOME}/sim/carla-0.9.16"
  )
  local candidate
  for candidate in "${candidates[@]}"; do
    if [[ -x "${candidate}/CarlaUE4.sh" ]]; then
      echo "${candidate}"
      return 0
    fi
  done
  echo "CARLA_ROOT is not set and CarlaUE4.sh was not found in the default locations." >&2
  return 1
}

is_port_listening() {
  local port="$1"
  ss -ltn "( sport = :${port} )" | grep -q LISTEN
}

wait_for_carla_server() {
  local host="$1"
  local port="$2"
  local timeout_seconds="$3"
  PYTHONPATH="" uv run python - "$host" "$port" "$timeout_seconds" <<'PY'
import sys
import time

import carla

host = sys.argv[1]
port = int(sys.argv[2])
timeout_seconds = float(sys.argv[3])
deadline = time.time() + timeout_seconds
last_error = None

while time.time() < deadline:
    try:
        client = carla.Client(host, port)
        client.set_timeout(2.0)
        client.get_world()
        sys.exit(0)
    except Exception as exc:
        last_error = exc
        time.sleep(1.0)

raise SystemExit(f"Timed out waiting for CARLA server on {host}:{port}: {last_error}")
PY
}

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
  CARLA_ROOT
  CARLA_HOST
  CARLA_PORT_1
  CARLA_PORT_2
  CARLA_PORT_3
  CARLA_ENVIRONMENT_CONFIG
  CARLA_EXPERT_CONFIG
  CARLA_CAMERA_WIDTH
  CARLA_CAMERA_HEIGHT
  CARLA_CAMERA_FOV
  CARLA_MCAP_SEGMENT_SECONDS
  CARLA_MANAGE_SERVERS
  CARLA_REUSE_RUNNING_SERVERS
  CARLA_START_TIMEOUT_SECONDS
  CARLA_DISPLAY
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
PORT_1="${CARLA_PORT_1:-2000}"
PORT_2="${CARLA_PORT_2:-2004}"
PORT_3="${CARLA_PORT_3:-2008}"
ENVIRONMENT_CONFIG="${CARLA_ENVIRONMENT_CONFIG:-scenarios/environments/town01_all_intersections_clear.json}"
EXPERT_CONFIG="${CARLA_EXPERT_CONFIG:-ad_stack/configs/expert/no_overtake.json}"
CAMERA_WIDTH="${CARLA_CAMERA_WIDTH:-1280}"
CAMERA_HEIGHT="${CARLA_CAMERA_HEIGHT:-720}"
CAMERA_FOV="${CARLA_CAMERA_FOV:-60}"
MCAP_SEGMENT_SECONDS="${CARLA_MCAP_SEGMENT_SECONDS:-600}"
MANAGE_SERVERS="${CARLA_MANAGE_SERVERS:-1}"
REUSE_RUNNING_SERVERS="${CARLA_REUSE_RUNNING_SERVERS:-0}"
CARLA_START_TIMEOUT_SECONDS="${CARLA_START_TIMEOUT_SECONDS:-90}"
CARLA_DISPLAY_VALUE="${CARLA_DISPLAY:-${DISPLAY:-:1}}"
CARLA_QUALITY_LEVEL="${CARLA_QUALITY_LEVEL:-Low}"

ROUTES=(
  "scenarios/routes/town01_perimeter_cw.json"
  "scenarios/routes/town01_intersection_weave_cw.json"
  "scenarios/routes/town01_intersection_weave_ccw.json"
)
PORTS=("${PORT_1}" "${PORT_2}" "${PORT_3}")
CARLA_ROOT_PATH="$(resolve_carla_root)"

RUN_ID="$(date +%Y%m%d_%H%M%S)_town01_collection_triplet"
LOG_DIR="${REPO_ROOT}/outputs/launcher_runs/${RUN_ID}"
mkdir -p "${LOG_DIR}"

declare -a WORKER_PIDS=()
declare -a EXIT_CODES=()
declare -a SERVER_WRAPPER_PIDS=()

stop_process_tree() {
  local pid="$1"
  if ! kill -0 "${pid}" >/dev/null 2>&1; then
    return 0
  fi
  pkill -TERM -P "${pid}" >/dev/null 2>&1 || true
  kill -TERM "${pid}" >/dev/null 2>&1 || true
  for _ in {1..20}; do
    if ! kill -0 "${pid}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.5
  done
  pkill -KILL -P "${pid}" >/dev/null 2>&1 || true
  kill -KILL "${pid}" >/dev/null 2>&1 || true
}

cleanup_workers() {
  for pid in "${WORKER_PIDS[@]:-}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill "${pid}" >/dev/null 2>&1 || true
    fi
  done
}

cleanup_servers() {
  for pid in "${SERVER_WRAPPER_PIDS[@]:-}"; do
    stop_process_tree "${pid}"
  done
}

cleanup_all() {
  cleanup_workers
  cleanup_servers
}

trap cleanup_all INT TERM EXIT

start_server() {
  local worker_id="$1"
  local port="$2"
  local server_log_path="${LOG_DIR}/server_${worker_id}.log"
  local -a extra_args=()
  if [[ "${port}" != "2000" ]]; then
    extra_args+=("-carla-rpc-port=${port}" "-carla-streaming-port=0")
  fi

  echo "Starting CARLA server for worker ${worker_id}: port=${port} log=${server_log_path}"
  (
    cd "${CARLA_ROOT_PATH}"
    export DISPLAY="${CARLA_DISPLAY_VALUE}"
    exec ./CarlaUE4.sh \
      -quality-level="${CARLA_QUALITY_LEVEL}" \
      -RenderOffScreen \
      -nosound \
      "${extra_args[@]}"
  ) >"${server_log_path}" 2>&1 &

  local wrapper_pid="$!"
  SERVER_WRAPPER_PIDS+=("${wrapper_pid}")
  wait_for_carla_server "${HOST}" "${port}" "${CARLA_START_TIMEOUT_SECONDS}"
}

for index in "${!PORTS[@]}"; do
  worker_id="$((index + 1))"
  port="${PORTS[$index]}"
  if [[ "${MANAGE_SERVERS}" == "1" ]]; then
    if is_port_listening "${port}"; then
      if [[ "${REUSE_RUNNING_SERVERS}" == "1" ]]; then
        echo "Reusing existing CARLA server for worker ${worker_id} on port ${port}"
      else
        echo "Port ${port} is already in use. Refusing to reuse unmanaged CARLA server." >&2
        echo "Stop the existing server or rerun with CARLA_REUSE_RUNNING_SERVERS=1." >&2
        exit 2
      fi
    else
      start_server "${worker_id}" "${port}"
    fi
  else
    if ! is_port_listening "${port}"; then
      echo "Expected CARLA server on port ${port}, but nothing is listening there." >&2
      exit 2
    fi
  fi
done

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

  WORKER_PIDS+=("$!")
done

for index in "${!WORKER_PIDS[@]}"; do
  pid="${WORKER_PIDS[$index]}"
  if wait "${pid}"; then
    EXIT_CODES+=(0)
  else
    EXIT_CODES+=($?)
  fi
done

cleanup_servers

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
