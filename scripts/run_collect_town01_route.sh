#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_ROOT}"

ROUTE_CONFIG="${1:-${CARLA_ROUTE_CONFIG:-configs/routes/town01_pilotnet_loop.json}}"
if [[ $# -gt 0 ]]; then
  shift
fi

PYTHONPATH="" uv run python -m pipelines.collect.collect_route_loop \
  --route-config "${ROUTE_CONFIG}" \
  --image-width "${CARLA_IMAGE_WIDTH:-320}" \
  --image-height "${CARLA_IMAGE_HEIGHT:-180}" \
  --target-speed-kmh "${CARLA_TARGET_SPEED_KMH:-30}" \
  --weather "${CARLA_WEATHER:-ClearNoon}" \
  --max-seconds "${CARLA_MAX_SECONDS:-600}" \
  "$@"
