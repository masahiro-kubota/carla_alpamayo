from __future__ import annotations

from dataclasses import dataclass, field

from ad_stack.agents.base import VehicleCommand
from ad_stack.planning.motion.plan import MotionPlan
from ad_stack.world_model.scene_state import SceneState


@dataclass(slots=True)
class VehicleControllerConfig:
    throttle_gain: float = 0.5
    brake_gain: float = 0.7


@dataclass(slots=True)
class VehicleController:
    config: VehicleControllerConfig = field(default_factory=VehicleControllerConfig)

    def compute(self, scene_state: SceneState, motion_plan: MotionPlan) -> VehicleCommand:
        speed_error = motion_plan.target_speed_mps - scene_state.ego.speed_mps
        if speed_error >= 0.0:
            throttle = min(1.0, speed_error * self.config.throttle_gain)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(1.0, (-speed_error) * self.config.brake_gain)
        return VehicleCommand(
            steer=motion_plan.target_steer,
            throttle=throttle,
            brake=brake,
        ).bounded()
