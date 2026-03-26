from __future__ import annotations

from typing import Any

from ad_stack.world_model import EgoState, RouteState, SceneState


class ObservationBuilder:
    """Build the shared scene representation consumed by planners and agents."""

    def build(
        self,
        *,
        timestamp_s: float,
        town_id: str,
        ego: EgoState,
        route: RouteState,
        tracked_objects: tuple[object, ...] = (),
        traffic_lights: tuple[object, ...] = (),
        metadata: dict[str, Any] | None = None,
    ) -> SceneState:
        return SceneState(
            timestamp_s=timestamp_s,
            town_id=town_id,
            ego=ego,
            route=route,
            tracked_objects=tracked_objects,
            traffic_lights=traffic_lights,
            metadata=dict(metadata or {}),
        )
