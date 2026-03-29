from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Literal

from .controller_executor import OvertakeExecutionQueue
from .route_alignment import (
    build_base_trace_execution_plan,
    build_overtake_waypoint_execution_plan,
    build_rejoin_waypoint_execution_plan,
)


@dataclass(slots=True)
class OvertakeExecutionManager:
    local_agent: Any
    sampling_resolution_m: float
    _queue: OvertakeExecutionQueue = field(default_factory=OvertakeExecutionQueue)
    _lane_change_path_available: bool = False
    _lane_change_path_failure_reason: str | None = None

    @property
    def target_lane_id(self) -> str | None:
        return self._queue.target_lane_id

    @property
    def lane_change_path_available(self) -> bool:
        return self._lane_change_path_available

    @property
    def lane_change_path_failure_reason(self) -> str | None:
        return self._lane_change_path_failure_reason

    def reset(self) -> None:
        self._queue.clear()
        self._lane_change_path_available = False
        self._lane_change_path_failure_reason = None

    def clear(self) -> None:
        self._queue.clear()

    def activate_overtake_plan(
        self,
        *,
        carla_module: Any,
        direction: Literal["left", "right"],
        route_index: int | None,
        base_trace: list[tuple[Any, Any]],
        route_point_to_trace_index: list[int],
        distance_same_lane_m: float,
        lane_change_distance_m: float,
        overtake_hold_distance_m: float,
    ) -> bool:
        plan = build_overtake_waypoint_execution_plan(
            carla_module=carla_module,
            direction=direction,
            route_index=route_index,
            base_trace=base_trace,
            route_point_to_trace_index=route_point_to_trace_index,
            distance_same_lane_m=distance_same_lane_m,
            lane_change_distance_m=lane_change_distance_m,
            overtake_hold_distance_m=overtake_hold_distance_m,
        )
        if not plan.available:
            self._set_lane_change_status(
                available=False,
                failure_reason=plan.failure_reason,
            )
            return False
        self._activate_waypoint_plan(
            target_lane_id=plan.target_lane_id,
            waypoints=plan.waypoints,
        )
        return True

    def try_activate_rejoin_plan(
        self,
        *,
        carla_module: Any,
        direction: Literal["left", "right"] | None,
        route_index: int | None,
        base_trace: list[tuple[Any, Any]],
        route_point_to_trace_index: list[int],
        origin_lane_id: str | None,
        target_lane_id: str | None,
        lane_change_distance_m: float,
    ) -> bool:
        plan = build_rejoin_waypoint_execution_plan(
            carla_module=carla_module,
            direction=direction,
            route_index=route_index,
            base_trace=base_trace,
            route_point_to_trace_index=route_point_to_trace_index,
            origin_lane_id=origin_lane_id,
            target_lane_id=target_lane_id,
            lane_change_distance_m=lane_change_distance_m,
            sampling_resolution_m=self.sampling_resolution_m,
        )
        if not plan.available:
            self._set_lane_change_status(
                available=False,
                failure_reason=plan.failure_reason,
            )
            return False
        self._activate_waypoint_plan(
            target_lane_id=plan.target_lane_id,
            waypoints=plan.waypoints,
        )
        return True

    def prepare_abort_return(
        self,
        *,
        carla_module: Any,
        direction: Literal["left", "right"] | None,
        route_index: int | None,
        base_trace: list[tuple[Any, Any]],
        route_point_to_trace_index: list[int],
        origin_lane_id: str | None,
        target_lane_id: str | None,
        lane_change_distance_m: float,
    ) -> None:
        if self.try_activate_rejoin_plan(
            carla_module=carla_module,
            direction=direction,
            route_index=route_index,
            base_trace=base_trace,
            route_point_to_trace_index=route_point_to_trace_index,
            origin_lane_id=origin_lane_id,
            target_lane_id=target_lane_id,
            lane_change_distance_m=lane_change_distance_m,
        ):
            return

        fallback_trace = build_base_trace_execution_plan(
            base_trace=base_trace,
            route_point_to_trace_index=route_point_to_trace_index,
            route_index=route_index,
            trace_offset=8,
        )
        if fallback_trace is None:
            return
        self._activate_trace_plan(trace=fallback_trace.trace, update_waypoints=True)

    def resume_base_route(
        self,
        *,
        route_index: int | None,
        base_trace: list[tuple[Any, Any]],
        route_point_to_trace_index: list[int],
    ) -> bool:
        remaining_trace = build_base_trace_execution_plan(
            base_trace=base_trace,
            route_point_to_trace_index=route_point_to_trace_index,
            route_index=route_index,
            trace_offset=1,
        )
        if remaining_trace is None:
            return False
        self._activate_trace_plan(trace=remaining_trace.trace, update_waypoints=False)
        return True

    def consume_next_waypoint(self, *, vehicle_location: Any) -> Any | None:
        return self._queue.consume_next_waypoint(
            vehicle_location=vehicle_location,
            sampling_resolution_m=self.sampling_resolution_m,
        )

    def _activate_waypoint_plan(self, *, target_lane_id: str | None, waypoints: list[Any]) -> None:
        self._queue.activate_waypoint_plan(
            target_lane_id=target_lane_id,
            waypoints=waypoints,
        )
        self._set_lane_change_status(available=True, failure_reason=None)

    def _activate_trace_plan(self, *, trace: list[tuple[Any, Any]], update_waypoints: bool) -> None:
        self._queue.activate_trace_plan(
            local_agent=self.local_agent,
            trace=trace,
            update_waypoints=update_waypoints,
        )

    def _set_lane_change_status(self, *, available: bool, failure_reason: str | None) -> None:
        self._lane_change_path_available = available
        self._lane_change_path_failure_reason = failure_reason
