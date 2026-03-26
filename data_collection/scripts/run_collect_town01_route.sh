#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"

ROUTE_CONFIG="${CARLA_ROUTE_CONFIG:-data_collection/configs/routes/town01_pilotnet_loop.json}"
if [[ $# -gt 0 && "$1" != -* ]]; then
  ROUTE_CONFIG="$1"
  shift
fi

PYTHONPATH="" uv run python -m data_collection.pipelines.collect.collect_route_loop \
  --route-config "${ROUTE_CONFIG}" \
  --image-width "${CARLA_IMAGE_WIDTH:-320}" \
  --image-height "${CARLA_IMAGE_HEIGHT:-180}" \
  --target-speed-kmh "${CARLA_TARGET_SPEED_KMH:-30}" \
  --weather "${CARLA_WEATHER:-ClearNoon}" \
  --max-seconds "${CARLA_MAX_SECONDS:-600}" \
  "$@"
