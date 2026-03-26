#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

cd "${REPO_ROOT}"

CHECKPOINT="outputs/train/pilotnet_best/best.pt"
if [[ $# -gt 0 && "$1" != -* ]]; then
  CHECKPOINT="$1"
  shift
fi

PYTHONPATH="" uv run python -m evaluation.pipelines.evaluate_pilotnet_loop \
  --checkpoint "${CHECKPOINT}" \
  --route-config "data_collection/configs/routes/town01_pilotnet_loop.json" \
  "$@"
