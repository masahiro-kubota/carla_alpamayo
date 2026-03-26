from __future__ import annotations

from dataclasses import dataclass, field

from ad_stack.world_model.scene_state import SceneState
from ad_stack.world_model.tracked_object import TrackedObject


@dataclass(slots=True)
class BehaviorCommand:
    behavior: str
    target_speed_mps: float
    desired_lane_change: str | None = None
    reasoning: tuple[str, ...] = ()


@dataclass(slots=True)
class BehaviorPlannerConfig:
    cruise_speed_mps: float = 8.3
    follow_distance_m: float = 12.0
    red_light_buffer_m: float = 18.0
    overtake_enabled: bool = True


@dataclass(slots=True)
class BehaviorPlanner:
    config: BehaviorPlannerConfig = field(default_factory=BehaviorPlannerConfig)

    def plan(self, scene_state: SceneState) -> BehaviorCommand:
        red_light = scene_state.closest_red_light(self.config.red_light_buffer_m)
        if red_light is not None:
            return BehaviorCommand(
                behavior="stop_for_red_light",
                target_speed_mps=0.0,
                reasoning=("red_light_ahead",),
            )

        lead_vehicle = scene_state.closest_lead_vehicle(self.config.follow_distance_m)
        if lead_vehicle is not None:
            if self._should_overtake(scene_state, lead_vehicle):
                return BehaviorCommand(
                    behavior="overtake",
                    target_speed_mps=min(self.config.cruise_speed_mps, lead_vehicle.speed_mps + 2.0),
                    desired_lane_change="left",
                    reasoning=("slow_lead_vehicle", "left_lane_available"),
                )
            return BehaviorCommand(
                behavior="follow_vehicle",
                target_speed_mps=min(self.config.cruise_speed_mps, lead_vehicle.speed_mps),
                reasoning=("lead_vehicle_ahead",),
            )

        if scene_state.metadata.get("blocked_lane"):
            return BehaviorCommand(
                behavior="avoid_obstacle",
                target_speed_mps=max(self.config.cruise_speed_mps * 0.5, 2.0),
                desired_lane_change="left",
                reasoning=("blocked_lane",),
            )

        target_speed_mps = scene_state.route.target_speed_mps or self.config.cruise_speed_mps
        return BehaviorCommand(
            behavior="lane_follow",
            target_speed_mps=min(self.config.cruise_speed_mps, target_speed_mps),
            reasoning=("default_cruise",),
        )

    def _should_overtake(self, scene_state: SceneState, lead_vehicle: TrackedObject) -> bool:
        if not self.config.overtake_enabled:
            return False
        left_lane_open = scene_state.ego.adjacent_lanes_open.get("left", False)
        slow_enough = lead_vehicle.speed_mps + 1.0 < scene_state.ego.speed_mps
        return left_lane_open and slow_enough
