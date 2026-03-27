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
  ./simulation/scripts/run_expert_town01.sh

This wrapper accepts no positional CLI overrides.
Configure it via environment variables instead:
  CARLA_HOST
  CARLA_PORT
  CARLA_ROUTE_CONFIG
  CARLA_ENVIRONMENT_CONFIG  (required)
  CARLA_EXPERT_CONFIG
  CARLA_CAMERA_WIDTH
  CARLA_CAMERA_HEIGHT
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

if [[ -z "${CARLA_ENVIRONMENT_CONFIG:-}" ]]; then
  echo "CARLA_ENVIRONMENT_CONFIG is required." >&2
  usage >&2
  exit 2
fi

PYTHONPATH="" uv run python -m simulation.pipelines.run_route_loop \
  --policy-kind expert \
  --host "${CARLA_HOST:-127.0.0.1}" \
  --port "${CARLA_PORT:-2000}" \
  --route-config "${CARLA_ROUTE_CONFIG:-scenarios/routes/town01_pilotnet_loop.json}" \
  --environment-config "${CARLA_ENVIRONMENT_CONFIG}" \
  --expert-config "${CARLA_EXPERT_CONFIG:-ad_stack/configs/expert/default.json}" \
  --camera-width "${CARLA_CAMERA_WIDTH:-1280}" \
  --camera-height "${CARLA_CAMERA_HEIGHT:-720}"
