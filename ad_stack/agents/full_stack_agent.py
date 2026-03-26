from __future__ import annotations

from dataclasses import dataclass, field

from ad_stack.agents.base import ControlDecision
from ad_stack.control.vehicle_controller import VehicleController
from ad_stack.planning.behavior.planner import BehaviorPlanner
from ad_stack.planning.motion.plan import MotionPlanner
from ad_stack.safety.emergency_brake import EmergencyBrakeGuard
from ad_stack.world_model.scene_state import SceneState


@dataclass(slots=True)
class FullStackAgent:
    """Reference composition for a rule-based full driving stack."""

    behavior_planner: BehaviorPlanner = field(default_factory=BehaviorPlanner)
    motion_planner: MotionPlanner = field(default_factory=MotionPlanner)
    controller: VehicleController = field(default_factory=VehicleController)
    safety_guard: EmergencyBrakeGuard = field(default_factory=EmergencyBrakeGuard)
    name: str = field(init=False, default="full_stack_agent")

    def reset(self) -> None:
        return None

    def step(self, scene_state: SceneState) -> ControlDecision:
        behavior = self.behavior_planner.plan(scene_state)
        motion_plan = self.motion_planner.plan(scene_state, behavior)
        command = self.controller.compute(scene_state, motion_plan)
        decision = ControlDecision(
            command=command,
            behavior=behavior.behavior,
            planner_state="full_stack_nominal",
            debug={
                "behavior_reasoning": list(behavior.reasoning),
                "target_speed_mps": motion_plan.target_speed_mps,
                "target_steer": motion_plan.target_steer,
            },
        )
        return self.safety_guard.apply(scene_state, decision)
