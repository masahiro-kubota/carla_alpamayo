#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

cd "${REPO_ROOT}"

if ! command -v uv >/dev/null 2>&1; then
  echo "uv is required but was not found in PATH." >&2
  exit 1
fi

extra_args=()

if [[ -n "${PILOTNET_MANIFEST_PATHS:-}" ]]; then
  for manifest_path in ${PILOTNET_MANIFEST_PATHS}; do
    extra_args+=(--manifest-path "${manifest_path}")
  done
fi

if [[ -n "${PILOTNET_INCLUDE_COMMANDS:-}" ]]; then
  for command_name in ${PILOTNET_INCLUDE_COMMANDS}; do
    extra_args+=(--include-command "${command_name}")
  done
fi

if [[ -n "${PILOTNET_TARGET_STEER_FIELD:-}" ]]; then
  extra_args+=(--target-steer-field "${PILOTNET_TARGET_STEER_FIELD}")
fi

if [[ -n "${PILOTNET_FALLBACK_TARGET_STEER_FIELD:-}" ]]; then
  extra_args+=(--fallback-target-steer-field "${PILOTNET_FALLBACK_TARGET_STEER_FIELD}")
fi

if [[ -n "${PILOTNET_INIT_CHECKPOINT:-}" ]]; then
  extra_args+=(--init-checkpoint "${PILOTNET_INIT_CHECKPOINT}")
fi

PYTHONPATH="" uv run python -m pipelines.train.train_pilotnet \
  --manifest-glob "${PILOTNET_MANIFEST_GLOB:-data/manifests/episodes/town01_pilotnet_loop_*.jsonl}" \
  --epochs "${PILOTNET_EPOCHS:-8}" \
  --batch-size "${PILOTNET_BATCH_SIZE:-64}" \
  --num-workers "${PILOTNET_NUM_WORKERS:-4}" \
  --split-mode "${PILOTNET_SPLIT_MODE:-frame}" \
  --command-conditioning "${PILOTNET_COMMAND_CONDITIONING:-none}" \
  "${extra_args[@]}" \
  "$@"
