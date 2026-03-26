#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

cd "${REPO_ROOT}"

if ! command -v uv >/dev/null 2>&1; then
  echo "uv is required but was not found in PATH." >&2
  exit 1
fi

if [[ $# -lt 1 ]]; then
  echo "usage: $0 <checkpoint> [extra args...]" >&2
  exit 1
fi

CHECKPOINT="$1"
shift

PYTHONPATH="" uv run python -m evaluation.pipelines.evaluate_pilotnet_loop \
  --checkpoint "${CHECKPOINT}" \
  "$@"
