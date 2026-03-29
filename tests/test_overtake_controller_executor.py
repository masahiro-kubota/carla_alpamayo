from __future__ import annotations

import unittest
from collections import deque
from dataclasses import dataclass

from ad_stack.overtake.infrastructure.carla import (
    OvertakeExecutionManager,
    OvertakeExecutionQueue,
    consume_waypoint_queue,
    run_tracking_control,
)


@dataclass
class _FakeLocation:
    x: float
    y: float
    z: float = 0.0

    def distance(self, other: "_FakeLocation") -> float:
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return (dx * dx + dy * dy + dz * dz) ** 0.5


@dataclass
class _FakeTransform:
    location: _FakeLocation
    rotation: "_FakeRotation | None" = None


class _FakeWaypoint:
    def __init__(
        self,
        location: _FakeLocation,
        *,
        road_id: int = 15,
        lane_id: int = -1,
        lane_type: str = "Driving",
    ) -> None:
        self.transform = _FakeTransform(location=location, rotation=_FakeRotation())
        self.road_id = road_id
        self.lane_id = lane_id
        self.lane_type = lane_type
        self._left_lane: "_FakeWaypoint | None" = None
        self._right_lane: "_FakeWaypoint | None" = None

    def set_left_lane(self, waypoint: "_FakeWaypoint | None") -> None:
        self._left_lane = waypoint

    def set_right_lane(self, waypoint: "_FakeWaypoint | None") -> None:
        self._right_lane = waypoint

    def get_left_lane(self) -> "_FakeWaypoint | None":
        return self._left_lane

    def get_right_lane(self) -> "_FakeWaypoint | None":
        return self._right_lane


@dataclass
class _FakeRotation:
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0


class _FakeCarla:
    class LaneType:
        Driving = "Driving"

    Location = _FakeLocation
    Rotation = _FakeRotation
    Transform = _FakeTransform


def _build_trace(*, count: int = 10) -> tuple[list[tuple[_FakeWaypoint, None]], list[int]]:
    trace: list[tuple[_FakeWaypoint, None]] = []
    for idx in range(count):
        waypoint = _FakeWaypoint(_FakeLocation(float(idx * 2), 0.0, 0.0), road_id=15, lane_id=-1)
        adjacent = _FakeWaypoint(
            _FakeLocation(float(idx * 2), 3.5, 0.0),
            road_id=15,
            lane_id=1,
        )
        waypoint.set_left_lane(adjacent)
        adjacent.set_right_lane(waypoint)
        trace.append((waypoint, None))
    return trace, list(range(count))


class _FakeController:
    def __init__(self) -> None:
        self.calls: list[tuple[float, _FakeWaypoint]] = []

    def run_step(self, target_speed_kmh: float, target_waypoint: _FakeWaypoint) -> str:
        self.calls.append((target_speed_kmh, target_waypoint))
        return "overtake-control"


class _FakeAgent:
    def __init__(self) -> None:
        self.target_speeds: list[float] = []
        self.run_calls = 0
        self.plans: list[list[tuple[_FakeWaypoint, None]]] = []

    def set_target_speed(self, target_speed_kmh: float) -> None:
        self.target_speeds.append(target_speed_kmh)

    def set_global_plan(
        self,
        trace: list[tuple[_FakeWaypoint, None]],
        *,
        stop_waypoint_creation: bool,
        clean_queue: bool,
    ) -> None:
        self.plans.append(trace)

    def run_step(self) -> str:
        self.run_calls += 1
        return "base-control"


class OvertakeControllerExecutorTests(unittest.TestCase):
    def test_consume_waypoint_queue_discards_close_points(self) -> None:
        waypoints = deque(
            [
                _FakeWaypoint(_FakeLocation(0.2, 0.0, 0.0)),
                _FakeWaypoint(_FakeLocation(3.0, 0.0, 0.0)),
            ]
        )

        next_waypoint = consume_waypoint_queue(
            vehicle_location=_FakeLocation(0.0, 0.0, 0.0),
            waypoints=waypoints,
            sampling_resolution_m=2.0,
        )

        self.assertEqual(next_waypoint.transform.location.x, 3.0)
        self.assertEqual(len(waypoints), 1)

    def test_run_tracking_control_uses_overtake_controller_when_target_present(self) -> None:
        agent = _FakeAgent()
        controller = _FakeController()
        waypoint = _FakeWaypoint(_FakeLocation(5.0, 0.0, 0.0))

        control = run_tracking_control(
            local_agent=agent,
            overtake_controller=controller,
            target_speed_kmh=30.0,
            target_waypoint=waypoint,
        )

        self.assertEqual(control, "overtake-control")
        self.assertEqual(len(controller.calls), 1)
        self.assertEqual(agent.run_calls, 0)

    def test_run_tracking_control_uses_base_agent_without_target(self) -> None:
        agent = _FakeAgent()
        controller = _FakeController()

        control = run_tracking_control(
            local_agent=agent,
            overtake_controller=controller,
            target_speed_kmh=20.0,
            target_waypoint=None,
        )

        self.assertEqual(control, "base-control")
        self.assertEqual(agent.target_speeds, [20.0])
        self.assertEqual(agent.run_calls, 1)
        self.assertEqual(controller.calls, [])

    def test_execution_queue_tracks_target_lane_and_consumes_waypoints(self) -> None:
        queue = OvertakeExecutionQueue()
        queue.activate_waypoint_plan(
            target_lane_id="15:1",
            waypoints=[
                _FakeWaypoint(_FakeLocation(0.2, 0.0, 0.0)),
                _FakeWaypoint(_FakeLocation(3.0, 0.0, 0.0)),
            ],
        )

        next_waypoint = queue.consume_next_waypoint(
            vehicle_location=_FakeLocation(0.0, 0.0, 0.0),
            sampling_resolution_m=2.0,
        )

        self.assertEqual(queue.target_lane_id, "15:1")
        self.assertEqual(next_waypoint.transform.location.x, 3.0)

    def test_execution_queue_activates_trace_plan(self) -> None:
        queue = OvertakeExecutionQueue()
        agent = _FakeAgent()
        trace = [
            (_FakeWaypoint(_FakeLocation(1.0, 0.0, 0.0)), None),
            (_FakeWaypoint(_FakeLocation(2.0, 0.0, 0.0)), None),
        ]

        queue.activate_trace_plan(
            local_agent=agent,
            trace=trace,
            update_waypoints=True,
        )

        self.assertEqual(len(agent.plans), 1)
        self.assertEqual(queue.consume_next_waypoint(
            vehicle_location=_FakeLocation(0.0, 0.0, 0.0),
            sampling_resolution_m=2.0,
        ).transform.location.x, 1.0)

    def test_execution_manager_activates_overtake_plan(self) -> None:
        agent = _FakeAgent()
        manager = OvertakeExecutionManager(local_agent=agent, sampling_resolution_m=2.0)
        base_trace, route_point_to_trace_index = _build_trace()

        activation = manager.activate_overtake_plan(
            carla_module=_FakeCarla,
            direction="left",
            route_index=0,
            base_trace=base_trace,
            route_point_to_trace_index=route_point_to_trace_index,
            distance_same_lane_m=4.0,
            lane_change_distance_m=4.0,
            overtake_hold_distance_m=4.0,
        )

        self.assertTrue(activation.activated)
        self.assertEqual(activation.outcome, "activated")
        self.assertTrue(manager.lane_change_path.available)
        self.assertIsNone(manager.lane_change_path.failure_reason)
        self.assertEqual(manager.target_lane_id, "15:1")
        next_waypoint = manager.consume_next_waypoint(vehicle_location=_FakeLocation(0.0, 0.0, 0.0))
        self.assertIsNotNone(next_waypoint)

    def test_execution_manager_prepares_abort_return_with_fallback_trace(self) -> None:
        agent = _FakeAgent()
        manager = OvertakeExecutionManager(local_agent=agent, sampling_resolution_m=2.0)
        base_trace, route_point_to_trace_index = _build_trace()

        activation = manager.prepare_abort_return(
            carla_module=_FakeCarla,
            direction=None,
            route_index=0,
            base_trace=base_trace,
            route_point_to_trace_index=route_point_to_trace_index,
            origin_lane_id="15:-1",
            target_lane_id="15:1",
            lane_change_distance_m=4.0,
        )

        self.assertTrue(activation.activated)
        self.assertEqual(activation.outcome, "fallback_trace_activated")
        self.assertFalse(manager.lane_change_path.available)
        self.assertEqual(manager.lane_change_path.failure_reason, "missing_rejoin_context")
        self.assertEqual(len(agent.plans), 1)
        next_waypoint = manager.consume_next_waypoint(vehicle_location=_FakeLocation(0.0, 0.0, 0.0))
        self.assertIsNotNone(next_waypoint)
        self.assertGreaterEqual(next_waypoint.transform.location.x, 16.0)
