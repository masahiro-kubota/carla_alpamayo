from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from ad_stack.world_model.ego_state import EgoState
from ad_stack.world_model.route_state import RouteState
from ad_stack.world_model.tracked_object import ActorKind, TrackedObject
from ad_stack.world_model.traffic_light_state import LightColor, TrafficLightState


@dataclass(slots=True)
class SceneState:
    timestamp_s: float
    town_id: str
    ego: EgoState
    route: RouteState
    tracked_objects: tuple[TrackedObject, ...] = field(default_factory=tuple)
    traffic_lights: tuple[TrafficLightState, ...] = field(default_factory=tuple)
    metadata: dict[str, Any] = field(default_factory=dict)

    def closest_lead_vehicle(self, max_distance_m: float | None = None) -> TrackedObject | None:
        lead_vehicles = [
            actor
            for actor in self.tracked_objects
            if actor.kind == ActorKind.VEHICLE and actor.is_ahead
        ]
        if max_distance_m is not None:
            lead_vehicles = [
                actor
                for actor in lead_vehicles
                if actor.longitudinal_distance_m is not None and actor.longitudinal_distance_m <= max_distance_m
            ]
        if not lead_vehicles:
            return None
        return min(lead_vehicles, key=lambda actor: actor.longitudinal_distance_m or float("inf"))

    def closest_red_light(self, max_distance_m: float | None = None) -> TrafficLightState | None:
        red_lights = [
            light
            for light in self.traffic_lights
            if light.affecting_ego and light.color == LightColor.RED
        ]
        if max_distance_m is not None:
            red_lights = [
                light
                for light in red_lights
                if light.distance_to_stop_line_m is not None and light.distance_to_stop_line_m <= max_distance_m
            ]
        if not red_lights:
            return None
        return min(red_lights, key=lambda light: light.distance_to_stop_line_m or float("inf"))
