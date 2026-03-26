from __future__ import annotations

from PIL import Image
import numpy as np
import torch
import torch.nn.functional as F


def preprocess_pil_rgb(
    image: Image.Image,
    image_width: int = 200,
    image_height: int = 66,
    crop_top_ratio: float = 0.35,
) -> torch.Tensor:
    rgb = image.convert("RGB")
    width, height = rgb.size
    top = int(height * crop_top_ratio)
    cropped = rgb.crop((0, top, width, height))
    resized = cropped.resize((image_width, image_height), Image.Resampling.BILINEAR)
    array = np.asarray(resized, dtype=np.float32) / 255.0
    return torch.from_numpy(array).permute(2, 0, 1).contiguous()


def preprocess_numpy_rgb(
    rgb_array: np.ndarray,
    image_width: int = 200,
    image_height: int = 66,
    crop_top_ratio: float = 0.35,
) -> torch.Tensor:
    if rgb_array.ndim != 3 or rgb_array.shape[2] != 3:
        raise ValueError(f"Expected HxWx3 RGB input, got shape {rgb_array.shape!r}")

    tensor = torch.from_numpy(np.asarray(rgb_array, dtype=np.float32) / 255.0).permute(2, 0, 1).unsqueeze(0)
    height = tensor.shape[2]
    top = int(height * crop_top_ratio)
    tensor = tensor[:, :, top:, :]
    tensor = F.interpolate(
        tensor,
        size=(image_height, image_width),
        mode="bilinear",
        align_corners=False,
    )
    return tensor.squeeze(0).contiguous()
