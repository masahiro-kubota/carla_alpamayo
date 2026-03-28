#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

cd "${REPO_ROOT}"

usage() {
  cat <<'EOF'
Usage:
  ./simulation/scripts/run_expert_town01_collection_triplet.sh <run-config-1.json> <run-config-2.json> <run-config-3.json>

This wrapper launches exactly 3 route-loop configs in parallel.
CARLA servers must already be running on the ports specified in each JSON file.
EOF
}

if ! command -v uv >/dev/null 2>&1; then
  echo "uv is required but was not found in PATH." >&2
  exit 1
fi

if [[ $# -ne 3 ]]; then
  if [[ $# -eq 1 && ( "$1" == "-h" || "$1" == "--help" ) ]]; then
    usage
    exit 0
  fi
  usage >&2
  exit 2
fi

PYTHONPATH="" uv run python -m simulation.pipelines.run_route_loop "$1" "$2" "$3"
