from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

import torch

from .commands import command_to_index
from .pilotnet import PilotNet
from .preprocessing import preprocess_numpy_rgb

if TYPE_CHECKING:
    from collections.abc import Sequence

    import numpy as np


def select_device(explicit_device: str | None) -> torch.device:
    if explicit_device:
        return torch.device(explicit_device)
    if torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("cpu")


def build_pilotnet_from_checkpoint(checkpoint: dict[str, Any], device: torch.device) -> PilotNet:
    model_config = checkpoint["model_config"]
    model = PilotNet(
        image_channels=int(model_config.get("image_channels", 3)),
        image_height=int(model_config["image_height"]),
        image_width=int(model_config["image_width"]),
        frame_stack=int(
            model_config.get("frame_stack", max(1, int(model_config.get("image_channels", 3)) // 3))
        ),
        temporal_fusion=str(model_config.get("temporal_fusion", "flatten")),
        temporal_hidden_dim=int(model_config.get("temporal_hidden_dim", 256)),
        route_point_dim=int(model_config.get("route_point_dim", 0)),
        command_vocab_size=int(model_config.get("command_vocab_size", 0)),
        command_embedding_dim=int(model_config.get("command_embedding_dim", 0)),
        command_branching=bool(model_config.get("command_branching", False)),
    ).to(device)
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()
    return model


@dataclass(slots=True)
class PilotNetInferenceRuntime:
    model: PilotNet
    checkpoint: dict[str, Any]
    device: torch.device
    checkpoint_path: Path | None = None

    @property
    def model_config(self) -> dict[str, Any]:
        return self.checkpoint["model_config"]

    @property
    def frame_stack(self) -> int:
        return int(
            self.model_config.get(
                "frame_stack", max(1, int(self.model_config.get("image_channels", 3)) // 3)
            )
        )

    def requires_route_point(self) -> bool:
        return int(self.model_config.get("route_point_dim", 0)) > 0

    def predict_steer(
        self,
        *,
        rgb_history: Sequence[np.ndarray],
        speed_mps: float,
        command: str,
        route_point: tuple[float, float] | None,
    ) -> float:
        if not rgb_history:
            raise ValueError("rgb_history must contain at least one frame.")

        image_tensors = [
            preprocess_numpy_rgb(
                rgb_array,
                image_width=int(self.model_config["image_width"]),
                image_height=int(self.model_config["image_height"]),
                crop_top_ratio=float(self.model_config["crop_top_ratio"]),
            )
            for rgb_array in rgb_history[-self.frame_stack :]
        ]
        while len(image_tensors) < self.frame_stack:
            image_tensors.insert(0, image_tensors[0])
        image_tensor = torch.cat(image_tensors, dim=0).unsqueeze(0).to(self.device)
        speed_tensor = torch.tensor(
            [[speed_mps / float(self.model_config["speed_norm_mps"])]],
            dtype=torch.float32,
            device=self.device,
        )

        command_index = None
        if self.model_config.get("command_conditioning", "none") in {"embedding", "branch"}:
            command_index = torch.tensor(
                [command_to_index(command)], dtype=torch.long, device=self.device
            )

        route_point_tensor = None
        if self.requires_route_point():
            if route_point is None:
                raise ValueError(
                    "route_point is required when the checkpoint expects route conditioning."
                )
            route_point_tensor = torch.tensor(
                [route_point], dtype=torch.float32, device=self.device
            )

        with torch.no_grad():
            prediction = self.model(image_tensor, speed_tensor, command_index, route_point_tensor)
        return float(prediction.squeeze().clamp(-1.0, 1.0).item())


def load_pilotnet_runtime(checkpoint_path: Path, device: torch.device) -> PilotNetInferenceRuntime:
    resolved_path = Path(checkpoint_path).resolve()
    checkpoint = torch.load(resolved_path, map_location=device)
    model = build_pilotnet_from_checkpoint(checkpoint, device)
    return PilotNetInferenceRuntime(
        model=model,
        checkpoint=checkpoint,
        device=device,
        checkpoint_path=resolved_path,
    )
