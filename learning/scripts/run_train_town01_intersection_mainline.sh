#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

cd "${REPO_ROOT}"

PYTHONPATH="" uv run python -m learning.pipelines.train.train_pilotnet \
  --manifest-glob "data/manifests/episodes/town01_pilotnet_loop_*.jsonl" \
  --manifest-glob "data/manifests/episodes/town01_pilotnet_loop_ccw_*.jsonl" \
  --manifest-glob "data/manifests/episodes/town01_right_focus_bottom_*.jsonl" \
  --manifest-glob "data/manifests/episodes/town01_right_focus_ne_*.jsonl" \
  --manifest-glob "data/manifests/episodes/town01_right_focus_se_*.jsonl" \
  --manifest-glob "data/manifests/episodes/town01_right_focus_sw_*.jsonl" \
  --manifest-glob "data/manifests/episodes/town01_right_focus_upper_band_*.jsonl" \
  --manifest-glob "data/manifests/episodes/town01_straight_focus_*.jsonl" \
  --manifest-glob "data/manifests/episodes/town01_left_focus_*.jsonl" \
  --manifest-glob "data/manifests/episodes/town01_curve_focus_*.jsonl" \
  --manifest-glob "data/manifests/episodes/town01_movement_*_train_*.jsonl" \
  --manifest-glob "data/manifests/corrections/town01_movement_*.jsonl" \
  --manifest-path "data/manifests/corrections/town01_eval_200434_lower_left_window.jsonl" \
  --manifest-path "data/manifests/corrections/town01_eval_202110_bottom_right_window.jsonl" \
  --manifest-path "data/manifests/corrections/town01_eval_203551_bottom_route_window.jsonl" \
  --manifest-path "data/manifests/corrections/town01_eval_203114_bottom_right_window.jsonl" \
  --manifest-path "data/manifests/corrections/town01_eval_210230_bottom_route_window.jsonl" \
  --manifest-path "data/manifests/corrections/town01_eval_210244_bottom_right_window.jsonl" \
  --manifest-path "data/manifests/corrections/town01_eval_210816_nw_right_window.jsonl" \
  --manifest-path "data/manifests/corrections/town01_eval_230633_upper_band_mid_window.jsonl" \
  --manifest-path "data/manifests/corrections/town01_eval_230644_upper_band_long_window.jsonl" \
  --epochs "${TOWN01_INTERSECTION_EPOCHS:-1}" \
  --batch-size "${TOWN01_INTERSECTION_BATCH_SIZE:-64}" \
  --num-workers "${TOWN01_INTERSECTION_NUM_WORKERS:-4}" \
  --split-mode episode \
  --frame-stack 3 \
  --command-conditioning branch \
  --target-steer-field expert_steer \
  --fallback-target-steer-field steer \
  --learning-rate "${TOWN01_INTERSECTION_LR:-1e-5}" \
  --weight-decay "${TOWN01_INTERSECTION_WEIGHT_DECAY:-1e-4}" \
  --seed "${TOWN01_INTERSECTION_SEED:-7}" \
  --init-checkpoint "${TOWN01_INTERSECTION_INIT_CHECKPOINT:-outputs/train/pilotnet_branch_fs3_20260321_231852/best.pt}" \
  "$@"
