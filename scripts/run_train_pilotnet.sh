#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

cd "${REPO_ROOT}"

if ! command -v uv >/dev/null 2>&1; then
  echo "uv is required but was not found in PATH." >&2
  exit 1
fi

PYTHONPATH="" uv run python -m pipelines.train.train_pilotnet \
  --manifest-glob "${PILOTNET_MANIFEST_GLOB:-data/manifests/episodes/town01_pilotnet_loop_*.jsonl}" \
  --epochs "${PILOTNET_EPOCHS:-8}" \
  --batch-size "${PILOTNET_BATCH_SIZE:-64}" \
  "$@"
