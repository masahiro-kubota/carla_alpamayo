from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from learning.libs.ml import PilotNetInferenceRuntime
from learning.libs.ml import load_pilotnet_runtime as _load_pilotnet_runtime
from learning.libs.ml import select_device as _select_device

if TYPE_CHECKING:
    import torch


def select_device(explicit_device: str | None) -> torch.device:
    return _select_device(explicit_device)


def load_pilotnet_runtime(
    checkpoint_path: str | Path,
    device: torch.device,
) -> PilotNetInferenceRuntime:
    return _load_pilotnet_runtime(Path(checkpoint_path), device)


__all__ = [
    "PilotNetInferenceRuntime",
    "load_pilotnet_runtime",
    "select_device",
]
