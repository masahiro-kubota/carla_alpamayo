from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import random
from typing import Any, Iterable

from PIL import Image
import torch
from torch.utils.data import Dataset

from .commands import command_to_index, normalize_command
from .preprocessing import preprocess_pil_rgb


PROJECT_ROOT = Path(__file__).resolve().parents[2]


@dataclass(slots=True)
class EpisodeFrame:
    image_path: Path
    speed_mps: float
    target_steer: float
    episode_id: str
    route_id: str
    command: str


def resolve_target_steer(
    raw: dict[str, Any],
    *,
    target_steer_field: str,
    fallback_target_steer_field: str | None,
) -> float | None:
    candidate_fields = [target_steer_field]
    if fallback_target_steer_field and fallback_target_steer_field != target_steer_field:
        candidate_fields.append(fallback_target_steer_field)
    for field_name in candidate_fields:
        value = raw.get(field_name)
        if value is not None:
            return float(value)
    return None


def load_episode_records(
    manifest_paths: Iterable[Path],
    *,
    include_failed_episodes: bool = True,
    max_abs_steer: float = 1.0,
    target_steer_field: str = "steer",
    fallback_target_steer_field: str | None = "steer",
    include_commands: set[str] | None = None,
) -> list[EpisodeFrame]:
    frames: list[EpisodeFrame] = []
    for manifest_path in manifest_paths:
        with manifest_path.open("r", encoding="utf-8") as handle:
            for line in handle:
                raw = json.loads(line)
                if raw["collision"]:
                    continue
                if not include_failed_episodes and not raw["success"]:
                    continue
                command_name = normalize_command(str(raw["command"]))
                if include_commands is not None and command_name not in include_commands:
                    continue
                steer = resolve_target_steer(
                    raw,
                    target_steer_field=target_steer_field,
                    fallback_target_steer_field=fallback_target_steer_field,
                )
                if steer is None:
                    continue
                steer = max(-max_abs_steer, min(max_abs_steer, steer))
                frames.append(
                    EpisodeFrame(
                        image_path=PROJECT_ROOT / raw["front_rgb_path"],
                        speed_mps=float(raw["speed"]),
                        target_steer=steer,
                        episode_id=str(raw["episode_id"]),
                        route_id=str(raw["route_id"]),
                        command=command_name,
                    )
                )
    return frames


def split_frames(
    frames: list[EpisodeFrame],
    train_ratio: float,
    seed: int,
) -> tuple[list[EpisodeFrame], list[EpisodeFrame]]:
    rng = random.Random(seed)
    indices = list(range(len(frames)))
    rng.shuffle(indices)
    cut = int(len(indices) * train_ratio)
    train_indices = set(indices[:cut])
    train = [frame for idx, frame in enumerate(frames) if idx in train_indices]
    val = [frame for idx, frame in enumerate(frames) if idx not in train_indices]
    return train, val


class PilotNetDataset(Dataset[dict[str, Any]]):
    def __init__(
        self,
        frames: list[EpisodeFrame],
        *,
        image_width: int = 200,
        image_height: int = 66,
        crop_top_ratio: float = 0.35,
        speed_norm_mps: float = 10.0,
        command_weight_map: dict[str, float] | None = None,
    ) -> None:
        self.frames = frames
        self.image_width = image_width
        self.image_height = image_height
        self.crop_top_ratio = crop_top_ratio
        self.speed_norm_mps = speed_norm_mps
        self.command_weight_map = command_weight_map or {}

    def __len__(self) -> int:
        return len(self.frames)

    def __getitem__(self, index: int) -> dict[str, Any]:
        frame = self.frames[index]
        with Image.open(frame.image_path) as image:
            image_tensor = preprocess_pil_rgb(
                image,
                image_width=self.image_width,
                image_height=self.image_height,
                crop_top_ratio=self.crop_top_ratio,
            )
        speed_tensor = torch.tensor([frame.speed_mps / self.speed_norm_mps], dtype=torch.float32)
        steer_tensor = torch.tensor([frame.target_steer], dtype=torch.float32)
        command_name = frame.command
        command_weight = self.command_weight_map.get(command_name, 1.0)
        sample_weight = torch.tensor([(1.0 + 4.0 * abs(frame.target_steer)) * command_weight], dtype=torch.float32)
        return {
            "image": image_tensor,
            "speed": speed_tensor,
            "command_index": torch.tensor(command_to_index(command_name), dtype=torch.long),
            "command_name": command_name,
            "target_steer": steer_tensor,
            "sample_weight": sample_weight,
            "episode_id": frame.episode_id,
        }
