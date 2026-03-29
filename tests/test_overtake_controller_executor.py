from __future__ import annotations

import unittest
from collections import deque
from dataclasses import dataclass

from ad_stack.overtake.infrastructure.carla import (
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


@dataclass
class _FakeWaypoint:
    transform: _FakeTransform


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
                _FakeWaypoint(_FakeTransform(_FakeLocation(0.2, 0.0, 0.0))),
                _FakeWaypoint(_FakeTransform(_FakeLocation(3.0, 0.0, 0.0))),
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
        waypoint = _FakeWaypoint(_FakeTransform(_FakeLocation(5.0, 0.0, 0.0)))

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
                _FakeWaypoint(_FakeTransform(_FakeLocation(0.2, 0.0, 0.0))),
                _FakeWaypoint(_FakeTransform(_FakeLocation(3.0, 0.0, 0.0))),
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
            (_FakeWaypoint(_FakeTransform(_FakeLocation(1.0, 0.0, 0.0))), None),
            (_FakeWaypoint(_FakeTransform(_FakeLocation(2.0, 0.0, 0.0))), None),
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
