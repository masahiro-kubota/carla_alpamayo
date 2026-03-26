from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Protocol

from ad_stack.world_model.scene_state import SceneState


@dataclass(slots=True)
class VehicleCommand:
    steer: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0
    hand_brake: bool = False
    reverse: bool = False

    def bounded(self) -> "VehicleCommand":
        return VehicleCommand(
            steer=max(-1.0, min(1.0, self.steer)),
            throttle=max(0.0, min(1.0, self.throttle)),
            brake=max(0.0, min(1.0, self.brake)),
            hand_brake=self.hand_brake,
            reverse=self.reverse,
        )


@dataclass(slots=True)
class ControlDecision:
    command: VehicleCommand
    behavior: str
    planner_state: str = "nominal"
    debug: dict[str, Any] = field(default_factory=dict)


class AutonomyAgent(Protocol):
    name: str

    def reset(self) -> None:
        """Reset any per-episode internal state."""

    def step(self, scene_state: SceneState) -> ControlDecision:
        """Produce a bounded vehicle command from the current scene state."""
