from __future__ import annotations

import unittest
from dataclasses import dataclass

from ad_stack.overtake.infrastructure.carla import (
    build_base_trace_execution_plan,
    build_overtake_waypoint_execution_plan,
    build_rejoin_waypoint_execution_plan,
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
class _FakeRotation:
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0


@dataclass
class _FakeTransform:
    location: _FakeLocation
    rotation: _FakeRotation


class _FakeWaypoint:
    def __init__(
        self,
        *,
        road_id: int,
        lane_id: int,
        location: _FakeLocation,
        lane_type: str = "Driving",
    ) -> None:
        self.road_id = road_id
        self.lane_id = lane_id
        self.transform = _FakeTransform(location=location, rotation=_FakeRotation())
        self.lane_type = lane_type
        self._left_lane: _FakeWaypoint | None = None
        self._right_lane: _FakeWaypoint | None = None

    def set_left_lane(self, waypoint: "_FakeWaypoint | None") -> None:
        self._left_lane = waypoint

    def set_right_lane(self, waypoint: "_FakeWaypoint | None") -> None:
        self._right_lane = waypoint

    def get_left_lane(self) -> "_FakeWaypoint | None":
        return self._left_lane

    def get_right_lane(self) -> "_FakeWaypoint | None":
        return self._right_lane


class _FakeCarla:
    class LaneType:
        Driving = "Driving"

    @dataclass
    class Location:
        x: float
        y: float
        z: float = 0.0

        def distance(self, other: "_FakeCarla.Location") -> float:
            dx = self.x - other.x
            dy = self.y - other.y
            dz = self.z - other.z
            return (dx * dx + dy * dy + dz * dz) ** 0.5

    @dataclass
    class Rotation:
        pitch: float = 0.0
        yaw: float = 0.0
        roll: float = 0.0

    @dataclass
    class Transform:
        location: "_FakeCarla.Location"
        rotation: "_FakeCarla.Rotation"


def _build_trace() -> tuple[list[tuple[_FakeWaypoint, None]], list[int]]:
    base_trace: list[tuple[_FakeWaypoint, None]] = []
    for idx in range(7):
        location = _FakeLocation(float(idx * 2), 0.0, 0.0)
        waypoint = _FakeWaypoint(road_id=15, lane_id=-1, location=location)
        adjacent = _FakeWaypoint(
            road_id=15,
            lane_id=1,
            location=_FakeLocation(location.x, 3.5, 0.0),
        )
        waypoint.set_left_lane(adjacent)
        adjacent.set_right_lane(waypoint)
        base_trace.append((waypoint, None))
    return base_trace, list(range(len(base_trace)))


class OvertakeRouteAlignmentTests(unittest.TestCase):
    def test_build_overtake_waypoint_execution_plan_materializes_path(self) -> None:
        base_trace, route_point_to_trace_index = _build_trace()

        plan = build_overtake_waypoint_execution_plan(
            carla_module=_FakeCarla,
            direction="left",
            route_index=0,
            base_trace=base_trace,
            route_point_to_trace_index=route_point_to_trace_index,
            distance_same_lane_m=4.0,
            lane_change_distance_m=4.0,
            overtake_hold_distance_m=4.0,
        )

        self.assertTrue(plan.available)
        self.assertEqual(plan.target_lane_id, "15:1")
        self.assertGreater(len(plan.waypoints), 0)
        self.assertEqual(plan.waypoints[0].transform.location.y, 0.0)
        self.assertEqual(plan.waypoints[-1].transform.location.y, 3.5)

    def test_build_rejoin_waypoint_execution_plan_requires_context(self) -> None:
        base_trace, route_point_to_trace_index = _build_trace()

        plan = build_rejoin_waypoint_execution_plan(
            carla_module=_FakeCarla,
            direction=None,
            route_index=0,
            base_trace=base_trace,
            route_point_to_trace_index=route_point_to_trace_index,
            origin_lane_id="15:-1",
            target_lane_id="15:1",
            lane_change_distance_m=4.0,
            sampling_resolution_m=2.0,
        )

        self.assertFalse(plan.available)
        self.assertEqual(plan.failure_reason, "missing_rejoin_context")

    def test_build_base_trace_execution_plan_slices_from_route_index_and_offset(self) -> None:
        base_trace, route_point_to_trace_index = _build_trace()

        plan = build_base_trace_execution_plan(
            base_trace=base_trace,
            route_point_to_trace_index=route_point_to_trace_index,
            route_index=2,
            trace_offset=1,
        )

        assert plan is not None
        self.assertEqual(len(plan.trace), 4)
        self.assertEqual(plan.trace[0][0].transform.location.x, 6.0)
        self.assertEqual(plan.waypoints[0].transform.location.x, 6.0)
