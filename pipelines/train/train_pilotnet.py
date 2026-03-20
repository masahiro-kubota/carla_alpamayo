from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime
import json
from pathlib import Path
import random
from math import sqrt

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import torch
from torch import nn
from torch.utils.data import DataLoader
from tqdm import tqdm

from libs.ml import PilotNet, PilotNetDataset, command_vocab_size, load_episode_records, normalize_command
from libs.ml.driving_dataset import split_frames


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Train a PilotNet-style front-camera-to-steer model.")
    parser.add_argument(
        "--manifest-glob",
        action="append",
        default=None,
    )
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--epochs", type=int, default=8)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--num-workers", type=int, default=4)
    parser.add_argument("--learning-rate", type=float, default=1e-4)
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    parser.add_argument("--train-ratio", type=float, default=0.9)
    parser.add_argument(
        "--split-mode",
        choices=("frame", "episode"),
        default="frame",
    )
    parser.add_argument("--image-width", type=int, default=200)
    parser.add_argument("--image-height", type=int, default=66)
    parser.add_argument("--crop-top-ratio", type=float, default=0.35)
    parser.add_argument("--speed-norm-mps", type=float, default=10.0)
    parser.add_argument(
        "--command-conditioning",
        choices=("none", "embedding"),
        default="none",
    )
    parser.add_argument("--command-embedding-dim", type=int, default=8)
    parser.add_argument(
        "--rebalance-commands",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--device", default=None)
    return parser


def select_device(explicit_device: str | None) -> torch.device:
    if explicit_device:
        return torch.device(explicit_device)
    if torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("cpu")


def build_output_dir(explicit_output_dir: str | None, default_prefix: str) -> Path:
    if explicit_output_dir:
        output_dir = Path(explicit_output_dir)
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = PROJECT_ROOT / "outputs" / "train" / f"{default_prefix}_{timestamp}"
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def build_command_weight_map(frames: list, enabled: bool) -> dict[str, float]:
    if not enabled:
        return {}
    counts = Counter(normalize_command(frame.command) for frame in frames)
    if not counts:
        return {}
    max_count = max(counts.values())
    weights: dict[str, float] = {}
    for command_name, count in counts.items():
        weights[command_name] = min(5.0, sqrt(max_count / count))
    return weights


def load_frames(manifest_glob: str) -> tuple[list[Path], list]:
    manifest_paths = sorted(PROJECT_ROOT.glob(manifest_glob))
    if not manifest_paths:
        raise FileNotFoundError(f"No manifests matched: {manifest_glob}")
    frames = load_episode_records(manifest_paths)
    if not frames:
        raise RuntimeError("No usable frames were found in the selected manifests.")
    return manifest_paths, frames


def load_frames_from_globs(manifest_globs: list[str]) -> tuple[list[Path], list]:
    manifest_paths: list[Path] = []
    seen_paths: set[Path] = set()
    for manifest_glob in manifest_globs:
        for path in sorted(PROJECT_ROOT.glob(manifest_glob)):
            if path in seen_paths:
                continue
            seen_paths.add(path)
            manifest_paths.append(path)
    if not manifest_paths:
        raise FileNotFoundError(f"No manifests matched: {manifest_globs}")
    frames = load_episode_records(manifest_paths)
    if not frames:
        raise RuntimeError("No usable frames were found in the selected manifests.")
    return manifest_paths, frames


def split_frames_by_episode(
    frames: list,
    train_ratio: float,
    seed: int,
) -> tuple[list, list]:
    rng = random.Random(seed)
    episode_ids = sorted({frame.episode_id for frame in frames})
    rng.shuffle(episode_ids)
    if len(episode_ids) < 2:
        raise RuntimeError("Episode-level split requires at least two episodes.")
    target_train = max(1, int(len(episode_ids) * train_ratio))
    target_train = min(target_train, len(episode_ids) - 1)
    train_episode_ids = set(episode_ids[:target_train])
    train = [frame for frame in frames if frame.episode_id in train_episode_ids]
    val = [frame for frame in frames if frame.episode_id not in train_episode_ids]
    if not train or not val:
        raise RuntimeError("Episode-level split produced an empty partition.")
    return train, val


def weighted_mse(prediction: torch.Tensor, target: torch.Tensor, sample_weight: torch.Tensor) -> torch.Tensor:
    return (((prediction - target) ** 2) * sample_weight).mean()


def evaluate_epoch(
    model: PilotNet,
    data_loader: DataLoader,
    device: torch.device,
) -> dict[str, float]:
    model.eval()
    total_loss = 0.0
    total_mse = 0.0
    total_mae = 0.0
    count = 0
    with torch.no_grad():
        for batch in data_loader:
            image = batch["image"].to(device)
            speed = batch["speed"].to(device)
            command_index = batch["command_index"].to(device)
            target = batch["target_steer"].to(device)
            sample_weight = batch["sample_weight"].to(device)

            prediction = model(image, speed, command_index)
            loss = weighted_mse(prediction, target, sample_weight)
            mse = ((prediction - target) ** 2).mean()
            mae = (prediction - target).abs().mean()
            batch_size = image.shape[0]
            total_loss += float(loss.item()) * batch_size
            total_mse += float(mse.item()) * batch_size
            total_mae += float(mae.item()) * batch_size
            count += batch_size
    return {
        "loss": total_loss / count,
        "mse": total_mse / count,
        "mae": total_mae / count,
    }


def main() -> None:
    args = build_parser().parse_args()
    random.seed(args.seed)
    torch.manual_seed(args.seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(args.seed)

    default_prefix = "pilotnet_cmd" if args.command_conditioning == "embedding" else "pilotnet"
    output_dir = build_output_dir(args.output_dir, default_prefix=default_prefix)
    device = select_device(args.device)
    manifest_globs = args.manifest_glob or ["data/manifests/episodes/town01_pilotnet_loop_*.jsonl"]
    manifest_paths, frames = load_frames_from_globs(manifest_globs)
    if args.split_mode == "episode":
        train_frames, val_frames = split_frames_by_episode(frames, train_ratio=args.train_ratio, seed=args.seed)
    else:
        train_frames, val_frames = split_frames(frames, train_ratio=args.train_ratio, seed=args.seed)
    if not train_frames or not val_frames:
        raise RuntimeError("Train/val split produced an empty partition. Adjust train_ratio or add more data.")

    command_weight_map = build_command_weight_map(train_frames, enabled=args.rebalance_commands)

    train_dataset = PilotNetDataset(
        train_frames,
        image_width=args.image_width,
        image_height=args.image_height,
        crop_top_ratio=args.crop_top_ratio,
        speed_norm_mps=args.speed_norm_mps,
        command_weight_map=command_weight_map,
    )
    val_dataset = PilotNetDataset(
        val_frames,
        image_width=args.image_width,
        image_height=args.image_height,
        crop_top_ratio=args.crop_top_ratio,
        speed_norm_mps=args.speed_norm_mps,
        command_weight_map=command_weight_map,
    )

    train_loader = DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.num_workers,
        pin_memory=device.type == "cuda",
    )
    val_loader = DataLoader(
        val_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=device.type == "cuda",
    )

    if args.command_conditioning == "embedding":
        model = PilotNet(
            image_height=args.image_height,
            image_width=args.image_width,
            command_vocab_size=command_vocab_size(),
            command_embedding_dim=args.command_embedding_dim,
        ).to(device)
        model_name = "pilotnet_commanded"
    else:
        model = PilotNet(image_height=args.image_height, image_width=args.image_width).to(device)
        model_name = "pilotnet"
    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=args.learning_rate,
        weight_decay=args.weight_decay,
    )

    history: list[dict[str, float]] = []
    best_val_loss = float("inf")
    best_checkpoint_path = output_dir / "best.pt"
    scaler = torch.amp.GradScaler("cuda", enabled=device.type == "cuda")

    run_config = {
        "manifest_globs": manifest_globs,
        "manifest_paths": [str(path.relative_to(PROJECT_ROOT)) for path in manifest_paths],
        "train_episode_ids": sorted({frame.episode_id for frame in train_frames}),
        "val_episode_ids": sorted({frame.episode_id for frame in val_frames}),
        "route_ids": sorted({frame.route_id for frame in frames}),
        "command_counts": dict(sorted(Counter(frame.command for frame in frames).items())),
        "frame_count": len(frames),
        "train_frame_count": len(train_frames),
        "val_frame_count": len(val_frames),
        "seed": args.seed,
        "device": str(device),
        "epochs": args.epochs,
        "batch_size": args.batch_size,
        "learning_rate": args.learning_rate,
        "weight_decay": args.weight_decay,
        "image_width": args.image_width,
        "image_height": args.image_height,
        "crop_top_ratio": args.crop_top_ratio,
        "speed_norm_mps": args.speed_norm_mps,
        "command_conditioning": args.command_conditioning,
        "command_embedding_dim": args.command_embedding_dim,
        "command_weight_map": command_weight_map,
        "rebalance_commands": args.rebalance_commands,
        "train_ratio": args.train_ratio,
        "split_mode": args.split_mode,
    }
    with (output_dir / "config.json").open("w", encoding="utf-8") as handle:
        json.dump(run_config, handle, indent=2)
        handle.write("\n")

    for epoch in range(1, args.epochs + 1):
        model.train()
        train_loss_sum = 0.0
        train_mse_sum = 0.0
        train_mae_sum = 0.0
        count = 0
        progress = tqdm(train_loader, desc=f"epoch {epoch}/{args.epochs}", leave=False)
        for batch in progress:
            image = batch["image"].to(device, non_blocking=True)
            speed = batch["speed"].to(device, non_blocking=True)
            command_index = batch["command_index"].to(device, non_blocking=True)
            target = batch["target_steer"].to(device, non_blocking=True)
            sample_weight = batch["sample_weight"].to(device, non_blocking=True)

            optimizer.zero_grad(set_to_none=True)
            with torch.autocast(device_type=device.type, dtype=torch.float16, enabled=device.type == "cuda"):
                prediction = model(image, speed, command_index)
                loss = weighted_mse(prediction, target, sample_weight)
                mse = ((prediction - target) ** 2).mean()
                mae = (prediction - target).abs().mean()

            scaler.scale(loss).backward()
            scaler.step(optimizer)
            scaler.update()

            batch_size = image.shape[0]
            train_loss_sum += float(loss.item()) * batch_size
            train_mse_sum += float(mse.item()) * batch_size
            train_mae_sum += float(mae.item()) * batch_size
            count += batch_size
            progress.set_postfix(loss=f"{loss.item():.4f}", mae=f"{mae.item():.4f}")

        train_metrics = {
            "loss": train_loss_sum / count,
            "mse": train_mse_sum / count,
            "mae": train_mae_sum / count,
        }
        val_metrics = evaluate_epoch(model, val_loader, device)
        metrics = {
            "epoch": epoch,
            "train_loss": train_metrics["loss"],
            "train_mse": train_metrics["mse"],
            "train_mae": train_metrics["mae"],
            "val_loss": val_metrics["loss"],
            "val_mse": val_metrics["mse"],
            "val_mae": val_metrics["mae"],
        }
        history.append(metrics)
        print(json.dumps(metrics))

        if val_metrics["loss"] < best_val_loss:
            best_val_loss = val_metrics["loss"]
            torch.save(
                {
                    "model_state_dict": model.state_dict(),
                    "model_name": model_name,
                    "model_config": {
                        "image_width": args.image_width,
                        "image_height": args.image_height,
                        "crop_top_ratio": args.crop_top_ratio,
                        "speed_norm_mps": args.speed_norm_mps,
                        "command_conditioning": args.command_conditioning,
                        "command_embedding_dim": args.command_embedding_dim,
                        "command_vocab_size": command_vocab_size() if args.command_conditioning == "embedding" else 0,
                    },
                    "train_run_config": run_config,
                    "best_epoch": epoch,
                    "best_val_loss": best_val_loss,
                },
                best_checkpoint_path,
            )

    with (output_dir / "metrics.json").open("w", encoding="utf-8") as handle:
        json.dump(history, handle, indent=2)
        handle.write("\n")

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot([item["epoch"] for item in history], [item["train_loss"] for item in history], label="train loss")
    ax.plot([item["epoch"] for item in history], [item["val_loss"] for item in history], label="val loss")
    ax.set_xlabel("epoch")
    ax.set_ylabel("weighted MSE")
    ax.set_title("PilotNet training curve")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / "loss_curve.png", dpi=180)
    plt.close(fig)

    summary = {
        "best_checkpoint_path": str(best_checkpoint_path.relative_to(PROJECT_ROOT)),
        "best_val_loss": best_val_loss,
        "best_epoch": min(history, key=lambda item: item["val_loss"])["epoch"],
        "final_train_loss": history[-1]["train_loss"],
        "final_val_loss": history[-1]["val_loss"],
        "model_name": model_name,
        "device": str(device),
        "frame_count": len(frames),
        "train_frame_count": len(train_frames),
        "val_frame_count": len(val_frames),
    }
    with (output_dir / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
        handle.write("\n")

    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
