from __future__ import annotations

from typing import Any


def select_active_light(traffic_lights: tuple[Any, ...]) -> Any | None:
    candidates = [light for light in traffic_lights if light.affects_ego]
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda light: (
            float(light.stop_line_distance_m)
            if light.stop_line_distance_m is not None
            else float(light.distance_m)
        ),
    )


def resolve_active_light(
    *,
    traffic_lights: tuple[Any, ...],
    timestamp_s: float,
    latched_red_light: Any | None,
    latched_red_until_s: float,
    red_latch_seconds: float,
) -> tuple[Any | None, Any | None, float]:
    active_light = select_active_light(traffic_lights)
    if active_light is not None:
        if active_light.state == "red":
            return (
                active_light,
                active_light,
                timestamp_s + red_latch_seconds,
            )
        return active_light, None, -1.0

    if latched_red_light is not None and timestamp_s <= latched_red_until_s:
        return latched_red_light, latched_red_light, latched_red_until_s

    return None, None, -1.0

