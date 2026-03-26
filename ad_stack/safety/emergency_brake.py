from __future__ import annotations

from dataclasses import dataclass, field

from ad_stack.agents.base import ControlDecision, VehicleCommand
from ad_stack.world_model.scene_state import SceneState


@dataclass(slots=True)
class EmergencyBrakeConfig:
    obstacle_distance_m: float = 5.0
    red_light_distance_m: float = 6.0


@dataclass(slots=True)
class EmergencyBrakeGuard:
    config: EmergencyBrakeConfig = field(default_factory=EmergencyBrakeConfig)

    def apply(self, scene_state: SceneState, decision: ControlDecision) -> ControlDecision:
        lead_vehicle = scene_state.closest_lead_vehicle(self.config.obstacle_distance_m)
        red_light = scene_state.closest_red_light(self.config.red_light_distance_m)
        if lead_vehicle is None and red_light is None:
            decision.command = decision.command.bounded()
            return decision

        debug = dict(decision.debug)
        if lead_vehicle is not None:
            debug["emergency_reason"] = "lead_vehicle_too_close"
            debug["lead_vehicle_distance_m"] = lead_vehicle.longitudinal_distance_m
        else:
            debug["emergency_reason"] = "red_light_too_close"
            debug["red_light_distance_m"] = red_light.distance_to_stop_line_m if red_light is not None else None
        return ControlDecision(
            command=VehicleCommand(steer=decision.command.steer, throttle=0.0, brake=1.0),
            behavior=decision.behavior,
            planner_state="emergency_brake",
            debug=debug,
        )
