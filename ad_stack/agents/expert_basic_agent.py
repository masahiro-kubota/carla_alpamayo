from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from ad_stack.agents.base import ControlDecision, VehicleCommand
from ad_stack.world_model.scene_state import SceneState
from libs.carla_utils import ensure_carla_agents_on_path, road_option_name


@dataclass(slots=True)
class ExpertBasicAgentConfig:
    target_speed_kmh: float = 30.0
    ignore_traffic_lights: bool = True
    ignore_stop_signs: bool = True
    ignore_vehicles: bool = False
    sampling_resolution_m: float = 2.0


class ExpertBasicAgent:
    """Adapter that exposes CARLA's BasicAgent through the local agent interface."""

    name = "expert_basic_agent"

    def __init__(
        self,
        vehicle: Any,
        world_map: Any,
        *,
        config: ExpertBasicAgentConfig | None = None,
    ) -> None:
        ensure_carla_agents_on_path()
        from agents.navigation.basic_agent import BasicAgent

        self.config = config or ExpertBasicAgentConfig()
        self._agent = BasicAgent(
            vehicle,
            target_speed=self.config.target_speed_kmh,
            opt_dict={
                "ignore_traffic_lights": self.config.ignore_traffic_lights,
                "ignore_stop_signs": self.config.ignore_stop_signs,
                "ignore_vehicles": self.config.ignore_vehicles,
                "sampling_resolution": self.config.sampling_resolution_m,
            },
            map_inst=world_map,
        )

    def set_global_plan(self, trace: list[tuple[Any, Any]]) -> None:
        self._agent.set_global_plan(trace, stop_waypoint_creation=True, clean_queue=True)

    def remaining_waypoints(self) -> int:
        return len(self._agent.get_local_planner().get_plan())

    def current_behavior(self) -> str:
        return road_option_name(self._agent.get_local_planner().target_road_option)

    def done(self) -> bool:
        return bool(self._agent.done())

    def reset(self) -> None:
        return None

    def step(self, scene_state: SceneState) -> ControlDecision:
        del scene_state
        control = self._agent.run_step()
        target_option = self._agent.get_local_planner().target_road_option
        return ControlDecision(
            command=VehicleCommand(
                steer=float(control.steer),
                throttle=float(control.throttle),
                brake=float(control.brake),
                hand_brake=bool(control.hand_brake),
                reverse=bool(control.reverse),
            ).bounded(),
            behavior=road_option_name(target_option),
            planner_state="carla_basic_agent",
            debug={
                "remaining_waypoints": len(self._agent.get_local_planner().get_plan()),
            },
        )
