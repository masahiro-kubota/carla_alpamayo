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
  ./simulation/scripts/run_expert_town01.sh <run-config.json>

Every simulation setting must live in the JSON file.
This wrapper only forwards the config to simulation.pipelines.run_route_loop.
EOF
}

if [[ $# -ne 1 ]]; then
  if [[ $# -eq 1 && ( "$1" == "-h" || "$1" == "--help" ) ]]; then
    usage
    exit 0
  fi
  usage >&2
  exit 2
fi

PYTHONPATH="" uv run python -m simulation.pipelines.run_route_loop "$1"
