from __future__ import annotations

from dataclasses import dataclass, field
from math import radians

from ad_stack.planning.behavior.planner import BehaviorCommand
from ad_stack.world_model.scene_state import SceneState


@dataclass(slots=True)
class MotionPlan:
    behavior: str
    target_speed_mps: float
    target_steer: float
    desired_lane_change: str | None = None


@dataclass(slots=True)
class MotionPlannerConfig:
    lateral_error_gain: float = 0.35
    heading_error_gain: float = 0.7
    lane_change_steer_offset: float = 0.15
    max_steer: float = 1.0


@dataclass(slots=True)
class MotionPlanner:
    config: MotionPlannerConfig = field(default_factory=MotionPlannerConfig)

    def plan(self, scene_state: SceneState, behavior: BehaviorCommand) -> MotionPlan:
        heading_error_rad = radians(scene_state.route.heading_error_deg)
        target_steer = (
            scene_state.route.target_steer
            - (self.config.lateral_error_gain * scene_state.route.lateral_error_m)
            - (self.config.heading_error_gain * heading_error_rad)
        )
        if behavior.desired_lane_change == "left":
            target_steer += self.config.lane_change_steer_offset
        elif behavior.desired_lane_change == "right":
            target_steer -= self.config.lane_change_steer_offset
        target_steer = max(-self.config.max_steer, min(self.config.max_steer, target_steer))
        return MotionPlan(
            behavior=behavior.behavior,
            target_speed_mps=behavior.target_speed_mps,
            target_steer=target_steer,
            desired_lane_change=behavior.desired_lane_change,
        )
