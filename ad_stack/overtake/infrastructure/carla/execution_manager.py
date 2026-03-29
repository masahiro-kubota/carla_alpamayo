from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Literal

from ad_stack.overtake.application.execution_contract import (
    ExecutionActivationResult,
    LaneChangePathStatus,
)

from .controller_executor import OvertakeExecutionQueue
from .route_alignment import (
    build_base_trace_execution_plan,
    build_overtake_waypoint_execution_plan,
    build_rejoin_waypoint_execution_plan,
)


@dataclass(slots=True)
class OvertakeExecutionManager:
    sampling_resolution_m: float
    _queue: OvertakeExecutionQueue = field(default_factory=OvertakeExecutionQueue)
    _lane_change_path: LaneChangePathStatus = field(
        default_factory=lambda: LaneChangePathStatus(available=False)
    )

    @property
    def target_lane_id(self) -> str | None:
        return self._queue.target_lane_id

    @property
    def lane_change_path(self) -> LaneChangePathStatus:
        return self._lane_change_path

    def reset(self) -> None:
        self._queue.clear()
        self._lane_change_path = LaneChangePathStatus(available=False)

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
    ) -> ExecutionActivationResult:
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
        if not plan.lane_change_path.available:
            self._set_lane_change_status(plan.lane_change_path)
            return ExecutionActivationResult(
                outcome="unavailable",
                target_lane_id=None,
                lane_change_path=plan.lane_change_path,
            )
        activation = self._activate_waypoint_plan(
            target_lane_id=plan.target_lane_id,
            waypoints=plan.waypoints,
        )
        return activation

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
    ) -> ExecutionActivationResult:
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
        if not plan.lane_change_path.available:
            self._set_lane_change_status(plan.lane_change_path)
            return ExecutionActivationResult(
                outcome="unavailable",
                target_lane_id=None,
                lane_change_path=plan.lane_change_path,
            )
        activation = self._activate_waypoint_plan(
            target_lane_id=plan.target_lane_id,
            waypoints=plan.waypoints,
        )
        return activation

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
    ) -> ExecutionActivationResult:
        rejoin_activation = self.try_activate_rejoin_plan(
            carla_module=carla_module,
            direction=direction,
            route_index=route_index,
            base_trace=base_trace,
            route_point_to_trace_index=route_point_to_trace_index,
            origin_lane_id=origin_lane_id,
            target_lane_id=target_lane_id,
            lane_change_distance_m=lane_change_distance_m,
        )
        if rejoin_activation.activated:
            return rejoin_activation

        fallback_trace = build_base_trace_execution_plan(
            base_trace=base_trace,
            route_point_to_trace_index=route_point_to_trace_index,
            route_index=route_index,
            trace_offset=8,
        )
        if fallback_trace is None:
            return ExecutionActivationResult(
                outcome="missing_base_trace",
                target_lane_id=self.target_lane_id,
                lane_change_path=self._lane_change_path,
            )
        self._activate_trace_plan(trace=fallback_trace.trace, update_waypoints=True)
        return ExecutionActivationResult(
            outcome="fallback_trace_activated",
            target_lane_id=self.target_lane_id,
            lane_change_path=self._lane_change_path,
        )

    def resume_base_route(
        self,
        *,
        route_index: int | None,
        base_trace: list[tuple[Any, Any]],
        route_point_to_trace_index: list[int],
    ) -> ExecutionActivationResult:
        del route_index, base_trace, route_point_to_trace_index
        self._queue.clear()
        return ExecutionActivationResult(
            outcome="base_route_resumed",
            target_lane_id=self.target_lane_id,
            lane_change_path=self._lane_change_path,
        )

    def advance(self, *, vehicle_location: Any) -> None:
        self._queue.advance(
            vehicle_location=vehicle_location,
            sampling_resolution_m=self.sampling_resolution_m,
        )

    def remaining_waypoints(self) -> list[Any]:
        return self._queue.remaining_waypoints()

    def _activate_waypoint_plan(
        self,
        *,
        target_lane_id: str | None,
        waypoints: list[Any],
    ) -> ExecutionActivationResult:
        self._queue.activate_waypoint_plan(
            target_lane_id=target_lane_id,
            waypoints=waypoints,
        )
        lane_change_path = LaneChangePathStatus(available=True)
        self._set_lane_change_status(lane_change_path)
        return ExecutionActivationResult(
            outcome="activated",
            target_lane_id=target_lane_id,
            lane_change_path=lane_change_path,
        )

    def _activate_trace_plan(self, *, trace: list[tuple[Any, Any]], update_waypoints: bool) -> None:
        self._queue.activate_trace_plan(trace=trace, update_waypoints=update_waypoints)

    def _set_lane_change_status(self, status: LaneChangePathStatus) -> None:
        self._lane_change_path = status
