from __future__ import annotations

import torch
from torch import nn


class PilotNet(nn.Module):
    def __init__(
        self,
        image_channels: int = 3,
        image_height: int = 66,
        image_width: int = 200,
        speed_feature_dim: int = 16,
    ) -> None:
        super().__init__()
        self.image_height = image_height
        self.image_width = image_width

        self.conv = nn.Sequential(
            nn.Conv2d(image_channels, 24, kernel_size=5, stride=2),
            nn.ELU(),
            nn.Conv2d(24, 36, kernel_size=5, stride=2),
            nn.ELU(),
            nn.Conv2d(36, 48, kernel_size=5, stride=2),
            nn.ELU(),
            nn.Conv2d(48, 64, kernel_size=3),
            nn.ELU(),
            nn.Conv2d(64, 64, kernel_size=3),
            nn.ELU(),
        )

        with torch.no_grad():
            dummy = torch.zeros(1, image_channels, image_height, image_width)
            flattened_dim = int(self.conv(dummy).flatten(1).shape[1])

        self.speed_mlp = nn.Sequential(
            nn.Linear(1, speed_feature_dim),
            nn.ELU(),
            nn.Linear(speed_feature_dim, speed_feature_dim),
            nn.ELU(),
        )
        self.head = nn.Sequential(
            nn.Linear(flattened_dim + speed_feature_dim, 100),
            nn.ELU(),
            nn.Linear(100, 50),
            nn.ELU(),
            nn.Linear(50, 10),
            nn.ELU(),
            nn.Linear(10, 1),
        )

    def forward(self, image: torch.Tensor, speed: torch.Tensor) -> torch.Tensor:
        image_features = self.conv(image).flatten(1)
        speed_features = self.speed_mlp(speed)
        features = torch.cat([image_features, speed_features], dim=1)
        return self.head(features)
