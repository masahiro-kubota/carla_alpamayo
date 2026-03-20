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
    steer: float
    episode_id: str
    route_id: str
    command: str


def load_episode_records(
    manifest_paths: Iterable[Path],
    *,
    include_failed_episodes: bool = True,
    max_abs_steer: float = 1.0,
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
                steer = float(raw["steer"])
                steer = max(-max_abs_steer, min(max_abs_steer, steer))
                frames.append(
                    EpisodeFrame(
                        image_path=PROJECT_ROOT / raw["front_rgb_path"],
                        speed_mps=float(raw["speed"]),
                        steer=steer,
                        episode_id=str(raw["episode_id"]),
                        route_id=str(raw["route_id"]),
                        command=str(raw["command"]),
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
        steer_tensor = torch.tensor([frame.steer], dtype=torch.float32)
        command_name = normalize_command(frame.command)
        command_weight = self.command_weight_map.get(command_name, 1.0)
        sample_weight = torch.tensor([(1.0 + 4.0 * abs(frame.steer)) * command_weight], dtype=torch.float32)
        return {
            "image": image_tensor,
            "speed": speed_tensor,
            "command_index": torch.tensor(command_to_index(command_name), dtype=torch.long),
            "command_name": command_name,
            "target_steer": steer_tensor,
            "sample_weight": sample_weight,
            "episode_id": frame.episode_id,
        }
