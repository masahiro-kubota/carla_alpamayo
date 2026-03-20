#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PYTHON_BIN="${REPO_ROOT}/.venv/bin/python"

cd "${REPO_ROOT}"

if [[ ! -x "${PYTHON_BIN}" ]]; then
  PYTHON_BIN="$(command -v python3)"
fi

"${PYTHON_BIN}" -m pipelines.collect.minimal_collect \
  --town "${CARLA_TOWN:-Town01}" \
  --host "${CARLA_HOST:-127.0.0.1}" \
  --port "${CARLA_PORT:-2000}" \
  --traffic-manager-port "${CARLA_TM_PORT:-8000}" \
  --frames "${CARLA_FRAMES:-400}" \
  --image-width "${CARLA_IMAGE_WIDTH:-640}" \
  --image-height "${CARLA_IMAGE_HEIGHT:-360}" \
  "$@"
