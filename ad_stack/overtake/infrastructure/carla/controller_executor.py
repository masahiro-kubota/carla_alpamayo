from __future__ import annotations

from collections import deque
from typing import Any


def consume_waypoint_queue(
    *,
    vehicle_location: Any,
    waypoints: deque[Any],
    sampling_resolution_m: float,
) -> Any | None:
    if not waypoints:
        return None
    acceptance_radius_m = min(1.0, max(0.6, sampling_resolution_m * 0.4))
    while len(waypoints) > 1:
        next_waypoint = waypoints[0]
        if vehicle_location.distance(next_waypoint.transform.location) > acceptance_radius_m:
            break
        waypoints.popleft()
    return waypoints[0]


def run_tracking_control(
    *,
    local_agent: Any,
    overtake_controller: Any,
    target_speed_kmh: float,
    target_waypoint: Any | None,
) -> Any:
    if target_waypoint is not None:
        return overtake_controller.run_step(max(0.0, target_speed_kmh), target_waypoint)
    local_agent.set_target_speed(max(0.0, target_speed_kmh))
    return local_agent.run_step()

