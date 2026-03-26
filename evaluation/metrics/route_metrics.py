from __future__ import annotations

from dataclasses import dataclass

from ad_stack.agents.base import ControlDecision
from ad_stack.world_model.scene_state import SceneState


@dataclass(slots=True)
class RouteMetrics:
    route_completion_ratio: float
    collision_count: int
    lane_invasion_count: int
    min_lead_distance_m: float | None


class RouteMetricsAccumulator:
    def __init__(self) -> None:
        self._collision_count = 0
        self._lane_invasion_count = 0
        self._min_lead_distance_m: float | None = None

    def observe(self, scene_state: SceneState, decision: ControlDecision) -> None:
        del decision
        self._collision_count += int(bool(scene_state.metadata.get("collision", False)))
        self._lane_invasion_count += int(bool(scene_state.metadata.get("lane_invasion", False)))
        lead_vehicle = scene_state.closest_lead_vehicle()
        if lead_vehicle is None or lead_vehicle.longitudinal_distance_m is None:
            return
        if self._min_lead_distance_m is None:
            self._min_lead_distance_m = lead_vehicle.longitudinal_distance_m
            return
        self._min_lead_distance_m = min(self._min_lead_distance_m, lead_vehicle.longitudinal_distance_m)

    def snapshot(self, final_scene: SceneState | None) -> RouteMetrics:
        return RouteMetrics(
            route_completion_ratio=final_scene.route.progress_ratio if final_scene is not None else 0.0,
            collision_count=self._collision_count,
            lane_invasion_count=self._lane_invasion_count,
            min_lead_distance_m=self._min_lead_distance_m,
        )
