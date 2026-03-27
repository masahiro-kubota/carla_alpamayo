#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

cd "${REPO_ROOT}"

if ! command -v uv >/dev/null 2>&1; then
  echo "uv is required but was not found in PATH." >&2
  exit 1
fi

traffic_setup_args=()
if [[ -n "${CARLA_TRAFFIC_SETUP:-}" ]]; then
  traffic_setup_args=(--traffic-setup "${CARLA_TRAFFIC_SETUP}")
fi

PYTHONPATH="" uv run python -m simulation.pipelines.run_route_loop \
  --policy-kind expert \
  --host "${CARLA_HOST:-127.0.0.1}" \
  --port "${CARLA_PORT:-2000}" \
  --route-config "${CARLA_ROUTE_CONFIG:-scenarios/routes/town01_pilotnet_loop.json}" \
  --expert-config "${CARLA_EXPERT_CONFIG:-ad_stack/configs/expert/default.json}" \
  "${traffic_setup_args[@]}" \
  --camera-width "${CARLA_CAMERA_WIDTH:-1280}" \
  --camera-height "${CARLA_CAMERA_HEIGHT:-720}" \
  --target-speed-kmh "${CARLA_TARGET_SPEED_KMH:-30}" \
  --weather "${CARLA_WEATHER:-ClearNoon}" \
  --max-seconds "${CARLA_MAX_SECONDS:-600}" \
  "$@"
