#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ACCEPTED_CONFIG="${REPO_ROOT}/outputs/train/pilotnet_branch_fs3_20260321_231852/config.json"

cd "${REPO_ROOT}"

PYTHONPATH="" uv run python - <<'PY' "$ACCEPTED_CONFIG" "$@"
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

config_path = Path(sys.argv[1])
extra_args = sys.argv[2:]

config = json.loads(config_path.read_text())
command: list[str] = [
    sys.executable,
    "-m",
    "pipelines.train.train_pilotnet",
]

for manifest_path in config["manifest_paths"]:
    command.extend(["--manifest-path", manifest_path])

for arg_name in (
    "output_dir",
    "epochs",
    "batch_size",
    "num_workers",
    "learning_rate",
    "weight_decay",
    "train_ratio",
    "split_mode",
    "image_width",
    "image_height",
    "crop_top_ratio",
    "frame_stack",
    "speed_norm_mps",
    "target_steer_field",
    "fallback_target_steer_field",
    "route_conditioning",
    "route_lookahead_m",
    "route_target_normalization_m",
    "carla_host",
    "carla_port",
    "command_conditioning",
    "command_embedding_dim",
    "seed",
    "device",
):
    value = config.get(arg_name)
    if value is None:
        continue
    command.extend([f"--{arg_name.replace('_', '-')}", str(value)])

if config.get("rebalance_commands") is not None:
    command.append("--rebalance-commands" if config["rebalance_commands"] else "--no-rebalance-commands")

init_checkpoint = config.get("init_checkpoint_path")
if init_checkpoint:
    command.extend(["--init-checkpoint", init_checkpoint])

include_commands = config.get("include_commands")
if include_commands:
    for command_name in include_commands:
        command.extend(["--include-command", command_name])

command.extend(extra_args)

subprocess.run(command, check=True)
PY
