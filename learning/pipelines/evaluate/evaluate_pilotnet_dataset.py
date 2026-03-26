from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path

import torch
from torch.utils.data import DataLoader

from libs.carla_utils import load_route_geometries_for_ids
from libs.project import PROJECT_ROOT, relative_to_project
from learning.libs.ml import (
    PilotNetDataset,
    attach_route_target_points,
    load_episode_records,
    load_pilotnet_runtime,
    normalize_command,
    select_device,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Evaluate a PilotNet checkpoint on recorded manifests without CARLA closed-loop rollout."
    )
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--manifest-glob", action="append", default=None)
    parser.add_argument("--manifest-path", action="append", default=None)
    parser.add_argument("--include-command", action="append", default=None)
    parser.add_argument("--target-steer-field", default="steer")
    parser.add_argument("--fallback-target-steer-field", default="steer")
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--num-workers", type=int, default=4)
    parser.add_argument("--device", default=None)
    parser.add_argument("--carla-host", default="127.0.0.1")
    parser.add_argument("--carla-port", type=int, default=2000)
    parser.add_argument("--summary-output", default=None)
    return parser


def resolve_manifest_paths(manifest_globs: list[str], manifest_paths: list[str]) -> list[Path]:
    resolved_paths: list[Path] = []
    seen_paths: set[Path] = set()
    for manifest_path in manifest_paths:
        path = Path(manifest_path)
        if not path.is_absolute():
            path = PROJECT_ROOT / path
        resolved_path = path.resolve()
        if not resolved_path.exists():
            raise FileNotFoundError(f"Manifest path does not exist: {manifest_path}")
        if resolved_path in seen_paths:
            continue
        seen_paths.add(resolved_path)
        resolved_paths.append(resolved_path)
    for manifest_glob in manifest_globs:
        for path in sorted(PROJECT_ROOT.glob(manifest_glob)):
            resolved_path = path.resolve()
            if resolved_path in seen_paths:
                continue
            seen_paths.add(resolved_path)
            resolved_paths.append(resolved_path)
    return resolved_paths


def display_path(path: Path) -> str:
    try:
        return relative_to_project(path)
    except ValueError:
        return str(path)


def build_summary_output_path(explicit_path: str | None) -> Path | None:
    if explicit_path is None:
        return None
    output_path = Path(explicit_path)
    if not output_path.is_absolute():
        output_path = PROJECT_ROOT / output_path
    return output_path


def mean_or_zero(total: float, count: int) -> float:
    if count <= 0:
        return 0.0
    return total / count


def build_per_command_metrics(
    *,
    counts: dict[str, int],
    mae_sums: dict[str, float],
    mse_sums: dict[str, float],
    prediction_sums: dict[str, float],
    target_sums: dict[str, float],
) -> dict[str, dict[str, float | int]]:
    metrics: dict[str, dict[str, float | int]] = {}
    for command_name in sorted(counts):
        count = counts[command_name]
        mean_abs_error = mean_or_zero(mae_sums[command_name], count)
        mean_squared_error = mean_or_zero(mse_sums[command_name], count)
        metrics[command_name] = {
            "count": count,
            "mae": round(mean_abs_error, 6),
            "mse": round(mean_squared_error, 6),
            "rmse": round(math.sqrt(mean_squared_error), 6),
            "mean_prediction": round(mean_or_zero(prediction_sums[command_name], count), 6),
            "mean_target": round(mean_or_zero(target_sums[command_name], count), 6),
        }
    return metrics


def main() -> None:
    args = build_parser().parse_args()

    checkpoint_path = Path(args.checkpoint).resolve()
    if not checkpoint_path.exists():
        raise SystemExit(f"Checkpoint not found: {checkpoint_path}")

    manifest_paths = resolve_manifest_paths(
        manifest_globs=args.manifest_glob or [],
        manifest_paths=args.manifest_path or [],
    )
    if not manifest_paths:
        raise SystemExit("No manifests matched. Pass --manifest-path or --manifest-glob.")

    include_commands = None
    if args.include_command:
        include_commands = {normalize_command(command_name) for command_name in args.include_command}

    frames = load_episode_records(
        manifest_paths,
        target_steer_field=args.target_steer_field,
        fallback_target_steer_field=args.fallback_target_steer_field,
        include_commands=include_commands,
    )
    if not frames:
        raise SystemExit("No usable frames were found in the selected manifests.")

    device = select_device(args.device)
    runtime = load_pilotnet_runtime(checkpoint_path, device)
    model_config = runtime.model_config

    if runtime.requires_route_point():
        route_ids = sorted({frame.route_id for frame in frames})
        route_geometries = load_route_geometries_for_ids(
            route_ids,
            host=args.carla_host,
            port=args.carla_port,
        )
        frames = attach_route_target_points(
            frames,
            route_geometries=route_geometries,
            lookahead_m=float(model_config.get("route_lookahead_m", 8.0)),
            target_normalization_m=float(model_config.get("route_target_normalization_m", 20.0)),
        )

    dataset = PilotNetDataset(
        frames,
        image_width=int(model_config["image_width"]),
        image_height=int(model_config["image_height"]),
        crop_top_ratio=float(model_config["crop_top_ratio"]),
        speed_norm_mps=float(model_config["speed_norm_mps"]),
        frame_stack=int(model_config.get("frame_stack", 1)),
    )
    dataloader = DataLoader(
        dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
    )

    command_conditioning = str(model_config.get("command_conditioning", "none"))
    use_command_input = command_conditioning in {"embedding", "branch"}

    total_count = 0
    mae_sum = 0.0
    mse_sum = 0.0
    prediction_sum = 0.0
    target_sum = 0.0
    command_counts: dict[str, int] = defaultdict(int)
    command_mae_sums: dict[str, float] = defaultdict(float)
    command_mse_sums: dict[str, float] = defaultdict(float)
    command_prediction_sums: dict[str, float] = defaultdict(float)
    command_target_sums: dict[str, float] = defaultdict(float)

    model = runtime.model
    model.eval()
    with torch.no_grad():
        for batch in dataloader:
            image = batch["image"].to(device)
            speed = batch["speed"].to(device)
            target = batch["target_steer"].to(device)
            command_index = batch["command_index"].to(device) if use_command_input else None
            route_point = batch["route_point"].to(device) if runtime.requires_route_point() else None

            prediction = model(image, speed, command_index, route_point)
            abs_error = (prediction - target).abs().squeeze(1)
            squared_error = ((prediction - target) ** 2).squeeze(1)
            prediction_values = prediction.squeeze(1).cpu().tolist()
            target_values = target.squeeze(1).cpu().tolist()
            abs_error_values = abs_error.cpu().tolist()
            squared_error_values = squared_error.cpu().tolist()

            total_count += len(prediction_values)
            mae_sum += float(sum(abs_error_values))
            mse_sum += float(sum(squared_error_values))
            prediction_sum += float(sum(prediction_values))
            target_sum += float(sum(target_values))

            for command_name, predicted_steer, target_steer, sample_abs_error, sample_squared_error in zip(
                batch["command_name"],
                prediction_values,
                target_values,
                abs_error_values,
                squared_error_values,
            ):
                normalized_command = normalize_command(command_name)
                command_counts[normalized_command] += 1
                command_mae_sums[normalized_command] += float(sample_abs_error)
                command_mse_sums[normalized_command] += float(sample_squared_error)
                command_prediction_sums[normalized_command] += float(predicted_steer)
                command_target_sums[normalized_command] += float(target_steer)

    mean_abs_error = mean_or_zero(mae_sum, total_count)
    mean_squared_error = mean_or_zero(mse_sum, total_count)
    summary = {
        "evaluation_type": "offline_dataset",
        "checkpoint_path": display_path(checkpoint_path),
        "manifest_paths": [display_path(path) for path in manifest_paths],
        "frame_count": total_count,
        "episode_count": len({frame.episode_id for frame in frames}),
        "route_count": len({frame.route_id for frame in frames}),
        "model_name": runtime.checkpoint.get("model_name"),
        "command_conditioning": command_conditioning,
        "route_conditioning": "target-point" if runtime.requires_route_point() else "none",
        "target_steer_field": args.target_steer_field,
        "fallback_target_steer_field": args.fallback_target_steer_field,
        "mae": round(mean_abs_error, 6),
        "mse": round(mean_squared_error, 6),
        "rmse": round(math.sqrt(mean_squared_error), 6),
        "mean_prediction": round(mean_or_zero(prediction_sum, total_count), 6),
        "mean_target": round(mean_or_zero(target_sum, total_count), 6),
        "per_command": build_per_command_metrics(
            counts=command_counts,
            mae_sums=command_mae_sums,
            mse_sums=command_mse_sums,
            prediction_sums=command_prediction_sums,
            target_sums=command_target_sums,
        ),
    }

    output_path = build_summary_output_path(args.summary_output)
    if output_path is not None:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2)
            handle.write("\n")

    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
