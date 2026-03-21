#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

cd "${REPO_ROOT}"

CHECKPOINT="${1:-outputs/train/pilotnet_branch_fs3_20260321_231852/best.pt}"
if [[ $# -gt 0 ]]; then
  shift
fi

PYTHONPATH="" uv run python -m pipelines.evaluate.evaluate_pilotnet_loop \
  --checkpoint "${CHECKPOINT}" \
  --route-config "configs/routes/town01_pilotnet_loop.json" \
  "$@"
