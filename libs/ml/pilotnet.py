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
        route_feature_dim: int = 16,
        route_point_dim: int = 0,
        command_vocab_size: int = 0,
        command_embedding_dim: int = 0,
        command_branching: bool = False,
    ) -> None:
        super().__init__()
        self.image_height = image_height
        self.image_width = image_width
        self.route_point_dim = route_point_dim
        self.command_vocab_size = command_vocab_size
        self.command_embedding_dim = command_embedding_dim
        self.command_branching = command_branching

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
        if route_point_dim > 0:
            self.route_mlp = nn.Sequential(
                nn.Linear(route_point_dim, route_feature_dim),
                nn.ELU(),
                nn.Linear(route_feature_dim, route_feature_dim),
                nn.ELU(),
            )
            extra_route_dim = route_feature_dim
        else:
            self.route_mlp = None
            extra_route_dim = 0
        if command_branching:
            if command_vocab_size <= 0:
                raise ValueError("command_vocab_size must be > 0 when command branching is enabled.")
            self.command_embedding = None
            extra_command_dim = 0
        elif command_vocab_size > 0 and command_embedding_dim > 0:
            self.command_embedding = nn.Embedding(command_vocab_size, command_embedding_dim)
            extra_command_dim = command_embedding_dim
        else:
            self.command_embedding = None
            extra_command_dim = 0
        fused_dim = flattened_dim + speed_feature_dim + extra_command_dim + extra_route_dim
        if command_branching:
            self.shared_head = nn.Sequential(
                nn.Linear(fused_dim, 100),
                nn.ELU(),
                nn.Linear(100, 50),
                nn.ELU(),
            )
            self.command_heads = nn.ModuleList(
                [
                    nn.Sequential(
                        nn.Linear(50, 10),
                        nn.ELU(),
                        nn.Linear(10, 1),
                    )
                    for _ in range(command_vocab_size)
                ]
            )
            self.head = None
        else:
            self.shared_head = None
            self.command_heads = None
            self.head = nn.Sequential(
                nn.Linear(fused_dim, 100),
                nn.ELU(),
                nn.Linear(100, 50),
                nn.ELU(),
                nn.Linear(50, 10),
                nn.ELU(),
                nn.Linear(10, 1),
            )

    def forward(
        self,
        image: torch.Tensor,
        speed: torch.Tensor,
        command_index: torch.Tensor | None = None,
        route_point: torch.Tensor | None = None,
    ) -> torch.Tensor:
        image_features = self.conv(image).flatten(1)
        speed_features = self.speed_mlp(speed)
        features = [image_features, speed_features]
        if self.route_mlp is not None:
            if route_point is None:
                raise ValueError("route_point is required when route conditioning is enabled.")
            route_features = self.route_mlp(route_point)
            features.append(route_features)
        if self.command_embedding is not None:
            if command_index is None:
                raise ValueError("command_index is required when command conditioning is enabled.")
            command_features = self.command_embedding(command_index.long().view(-1))
            features.append(command_features)
        features = torch.cat(features, dim=1)
        if self.command_branching:
            if command_index is None:
                raise ValueError("command_index is required when command branching is enabled.")
            shared_features = self.shared_head(features)
            outputs = torch.empty((shared_features.shape[0], 1), dtype=shared_features.dtype, device=shared_features.device)
            command_ids = command_index.long().view(-1)
            for branch_index, branch_head in enumerate(self.command_heads):
                mask = command_ids == branch_index
                if mask.any():
                    outputs[mask] = branch_head(shared_features[mask])
            return outputs
        return self.head(features)
