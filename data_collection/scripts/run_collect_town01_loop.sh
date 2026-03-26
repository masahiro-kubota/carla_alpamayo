#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

cd "${REPO_ROOT}"

if ! command -v uv >/dev/null 2>&1; then
  echo "uv is required but was not found in PATH." >&2
  exit 1
fi

PYTHONPATH="" uv run python -m data_collection.pipelines.collect.collect_route_loop \
  --host "${CARLA_HOST:-127.0.0.1}" \
  --port "${CARLA_PORT:-2000}" \
  --route-config "${CARLA_ROUTE_CONFIG:-data_collection/configs/routes/town01_pilotnet_loop.json}" \
  --target-speed-kmh "${CARLA_TARGET_SPEED_KMH:-30}" \
  --image-width "${CARLA_IMAGE_WIDTH:-320}" \
  --image-height "${CARLA_IMAGE_HEIGHT:-180}" \
  --max-seconds "${CARLA_MAX_SECONDS:-600}" \
  "$@"
