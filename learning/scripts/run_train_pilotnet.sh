#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

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

if [[ -n "${PILOTNET_FRAME_STACK:-}" ]]; then
  extra_args+=(--frame-stack "${PILOTNET_FRAME_STACK}")
fi

if [[ -n "${PILOTNET_ROUTE_CONDITIONING:-}" ]]; then
  extra_args+=(--route-conditioning "${PILOTNET_ROUTE_CONDITIONING}")
fi

if [[ -n "${PILOTNET_ROUTE_LOOKAHEAD_M:-}" ]]; then
  extra_args+=(--route-lookahead-m "${PILOTNET_ROUTE_LOOKAHEAD_M}")
fi

if [[ -n "${PILOTNET_ROUTE_TARGET_NORMALIZATION_M:-}" ]]; then
  extra_args+=(--route-target-normalization-m "${PILOTNET_ROUTE_TARGET_NORMALIZATION_M}")
fi

PYTHONPATH="" uv run python -m learning.pipelines.train.train_pilotnet \
  --manifest-glob "${PILOTNET_MANIFEST_GLOB:-data/manifests/episodes/town01_pilotnet_loop_*.jsonl}" \
  --epochs "${PILOTNET_EPOCHS:-8}" \
  --batch-size "${PILOTNET_BATCH_SIZE:-64}" \
  --num-workers "${PILOTNET_NUM_WORKERS:-4}" \
  --split-mode "${PILOTNET_SPLIT_MODE:-frame}" \
  --command-conditioning "${PILOTNET_COMMAND_CONDITIONING:-none}" \
  "${extra_args[@]}" \
  "$@"
