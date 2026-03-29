from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Any


@dataclass(slots=True)
class OvertakeExecutionQueue:
    target_lane_id: str | None = None
    _waypoints: deque[Any] = field(default_factory=deque)

    def clear(self) -> None:
        self.target_lane_id = None
        self._waypoints.clear()

    def activate_waypoint_plan(self, *, target_lane_id: str | None, waypoints: list[Any]) -> None:
        self.target_lane_id = target_lane_id
        self._waypoints = deque(waypoints)

    def activate_trace_plan(
        self,
        *,
        local_agent: Any,
        trace: list[tuple[Any, Any]],
        update_waypoints: bool,
    ) -> None:
        local_agent.set_global_plan(
            trace,
            stop_waypoint_creation=True,
            clean_queue=True,
        )
        if update_waypoints:
            self._waypoints = deque(waypoint for waypoint, _option in trace)

    def consume_next_waypoint(
        self,
        *,
        vehicle_location: Any,
        sampling_resolution_m: float,
    ) -> Any | None:
        next_waypoint = consume_waypoint_queue(
            vehicle_location=vehicle_location,
            waypoints=self._waypoints,
            sampling_resolution_m=sampling_resolution_m,
        )
        if next_waypoint is None:
            self.target_lane_id = None
        return next_waypoint


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
