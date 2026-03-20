#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

cd "${REPO_ROOT}"

python3 -m pipelines.collect.minimal_collect \
  --town "${CARLA_TOWN:-Town01}" \
  --host "${CARLA_HOST:-127.0.0.1}" \
  --port "${CARLA_PORT:-2000}" \
  --traffic-manager-port "${CARLA_TM_PORT:-8000}" \
  --frames "${CARLA_FRAMES:-300}" \
  "$@"
