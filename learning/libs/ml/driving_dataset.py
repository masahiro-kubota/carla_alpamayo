from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import random
from typing import Any, Iterable

from PIL import Image
import torch
from torch.utils.data import Dataset

from libs.carla_utils import RouteGeometry, compute_local_target_point
from libs.project import PROJECT_ROOT

from .commands import command_to_index, normalize_command
from .preprocessing import preprocess_pil_rgb


@dataclass(slots=True)
class EpisodeFrame:
    image_path: Path
    speed_mps: float
    target_steer: float
    episode_id: str
    frame_id: int
    route_id: str
    command: str
    vehicle_x: float | None = None
    vehicle_y: float | None = None
    vehicle_yaw_deg: float | None = None
    route_target_x: float | None = None
    route_target_y: float | None = None


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
                        frame_id=int(raw["frame_id"]),
                        route_id=str(raw["route_id"]),
                        command=command_name,
                        vehicle_x=float(raw["vehicle_x"]) if raw.get("vehicle_x") is not None else None,
                        vehicle_y=float(raw["vehicle_y"]) if raw.get("vehicle_y") is not None else None,
                        vehicle_yaw_deg=float(raw["vehicle_yaw_deg"]) if raw.get("vehicle_yaw_deg") is not None else None,
                        route_target_x=float(raw["route_target_x"]) if raw.get("route_target_x") is not None else None,
                        route_target_y=float(raw["route_target_y"]) if raw.get("route_target_y") is not None else None,
                    )
                )
    return frames


def attach_route_target_points(
    frames: list[EpisodeFrame],
    *,
    route_geometries: dict[str, RouteGeometry],
    lookahead_m: float,
    target_normalization_m: float,
) -> list[EpisodeFrame]:
    sorted_frames = sorted(frames, key=lambda frame: (frame.episode_id, frame.frame_id))
    route_indices_by_episode: dict[str, int] = {}
    enriched_frames: list[EpisodeFrame] = []
    for frame in sorted_frames:
        if frame.route_target_x is not None and frame.route_target_y is not None:
            enriched_frames.append(frame)
            continue
        if frame.vehicle_x is None or frame.vehicle_y is None or frame.vehicle_yaw_deg is None:
            raise RuntimeError(
                f"Route conditioning requires pose fields, but {frame.episode_id}:{frame.frame_id} is missing pose."
            )
        route_geometry = route_geometries.get(frame.route_id)
        if route_geometry is None:
            raise KeyError(f"Missing route geometry for route_id={frame.route_id}")
        route_target, route_index = compute_local_target_point(
            route_geometry,
            vehicle_x=frame.vehicle_x,
            vehicle_y=frame.vehicle_y,
            vehicle_yaw_deg=frame.vehicle_yaw_deg,
            lookahead_m=lookahead_m,
            target_normalization_m=target_normalization_m,
            previous_route_index=route_indices_by_episode.get(frame.episode_id),
        )
        route_indices_by_episode[frame.episode_id] = route_index
        enriched_frames.append(
            EpisodeFrame(
                image_path=frame.image_path,
                speed_mps=frame.speed_mps,
                target_steer=frame.target_steer,
                episode_id=frame.episode_id,
                frame_id=frame.frame_id,
                route_id=frame.route_id,
                command=frame.command,
                vehicle_x=frame.vehicle_x,
                vehicle_y=frame.vehicle_y,
                vehicle_yaw_deg=frame.vehicle_yaw_deg,
                route_target_x=route_target[0],
                route_target_y=route_target[1],
            )
        )
    return enriched_frames


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
        frame_stack: int = 1,
    ) -> None:
        if frame_stack < 1:
            raise ValueError("frame_stack must be >= 1")
        self.frames = sorted(frames, key=lambda frame: (frame.episode_id, frame.frame_id))
        self.image_width = image_width
        self.image_height = image_height
        self.crop_top_ratio = crop_top_ratio
        self.speed_norm_mps = speed_norm_mps
        self.command_weight_map = command_weight_map or {}
        self.frame_stack = frame_stack
        self.stack_indices = self._build_stack_indices()

    def __len__(self) -> int:
        return len(self.frames)

    def _build_stack_indices(self) -> list[list[int]]:
        stack_indices: list[list[int]] = []
        episode_history: dict[str, list[int]] = {}
        for index, frame in enumerate(self.frames):
            history = episode_history.setdefault(frame.episode_id, [])
            current_stack = (history + [index])[-self.frame_stack :]
            while len(current_stack) < self.frame_stack:
                current_stack.insert(0, current_stack[0])
            stack_indices.append(current_stack)
            history.append(index)
        return stack_indices

    def _load_image_tensor(self, image_path: Path) -> torch.Tensor:
        with Image.open(image_path) as image:
            return preprocess_pil_rgb(
                image,
                image_width=self.image_width,
                image_height=self.image_height,
                crop_top_ratio=self.crop_top_ratio,
            )

    def __getitem__(self, index: int) -> dict[str, Any]:
        frame = self.frames[index]
        image_tensor = torch.cat(
            [self._load_image_tensor(self.frames[stack_index].image_path) for stack_index in self.stack_indices[index]],
            dim=0,
        )
        speed_tensor = torch.tensor([frame.speed_mps / self.speed_norm_mps], dtype=torch.float32)
        steer_tensor = torch.tensor([frame.target_steer], dtype=torch.float32)
        command_name = frame.command
        command_weight = self.command_weight_map.get(command_name, 1.0)
        sample_weight = torch.tensor([(1.0 + 4.0 * abs(frame.target_steer)) * command_weight], dtype=torch.float32)
        route_point_tensor = torch.tensor(
            [
                frame.route_target_x if frame.route_target_x is not None else 0.0,
                frame.route_target_y if frame.route_target_y is not None else 0.0,
            ],
            dtype=torch.float32,
        )
        return {
            "image": image_tensor,
            "speed": speed_tensor,
            "command_index": torch.tensor(command_to_index(command_name), dtype=torch.long),
            "command_name": command_name,
            "target_steer": steer_tensor,
            "sample_weight": sample_weight,
            "episode_id": frame.episode_id,
            "route_point": route_point_tensor,
        }
