from __future__ import annotations

import unittest
from dataclasses import dataclass

from ad_stack.overtake import build_stopped_obstacle_targets
from ad_stack.overtake.infrastructure.carla import (
    build_overtake_pass_snapshot,
    build_overtake_scene_snapshot,
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
    yaw: float = 0.0


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
        s: float,
        location: _FakeLocation,
        lane_type: str = "Driving",
    ) -> None:
        self.road_id = road_id
        self.lane_id = lane_id
        self.s = s
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


class _FakeWorldMap:
    def __init__(self, mapping: dict[tuple[float, float, float], _FakeWaypoint]) -> None:
        self._mapping = mapping

    def get_waypoint(self, location: _FakeLocation, lane_type: str | None = None) -> _FakeWaypoint:
        return self._mapping[(location.x, location.y, location.z)]


class _FakeCarla:
    class LaneType:
        Driving = "Driving"

    @dataclass
    class Location:
        x: float
        y: float
        z: float = 0.0


@dataclass
class _TrackedActor:
    actor_id: int
    lane_id: str
    x_m: float
    y_m: float
    speed_mps: float
    relation: str
    is_ahead: bool
    longitudinal_distance_m: float | None


class OvertakeSceneSnapshotTests(unittest.TestCase):
    def test_prefers_same_lane_stopped_target_and_builds_context(self) -> None:
        ego_waypoint = _FakeWaypoint(
            road_id=15,
            lane_id=-1,
            s=0.0,
            location=_FakeLocation(0.0, 0.0, 0.0),
        )
        ego_waypoint.set_left_lane(
            _FakeWaypoint(
                road_id=15,
                lane_id=1,
                s=0.0,
                location=_FakeLocation(0.0, 3.5, 0.0),
            )
        )

        snapshot = build_overtake_scene_snapshot(
            tracked_objects=(
                _TrackedActor(
                    actor_id=101,
                    lane_id="15:-1",
                    x_m=18.0,
                    y_m=0.0,
                    speed_mps=0.0,
                    relation="same_lane",
                    is_ahead=True,
                    longitudinal_distance_m=18.0,
                ),
            ),
            timestamp_s=12.0,
            current_speed_mps=6.0,
            current_lane_id="15:-1",
            route_target_lane_id="15:-1",
            route_index=None,
            ego_waypoint=ego_waypoint,
            adjacent_lanes_open={"left": True, "right": False},
            target_speed_kmh=30.0,
            follow_headway_seconds=1.8,
            stopped_speed_threshold_mps=0.3,
            cluster_merge_gap_m=10.0,
            cluster_max_member_speed_mps=0.5,
            target_policy=build_stopped_obstacle_targets,
            active_signal_state=None,
            signal_stop_distance_m=None,
            allow_overtake=True,
            preferred_direction="left_first",
            world_map=_FakeWorldMap({}),
            carla_module=_FakeCarla,
            base_trace=[],
            route_point_to_trace_index=[],
            route_point_progress_m=[],
        )

        self.assertEqual(snapshot.active_target.primary_actor_id, 101)
        self.assertEqual(snapshot.lead_distance_m, 18.0)
        self.assertTrue(snapshot.decision_context.left_lane.lane_open)
        self.assertEqual(snapshot.decision_context.active_target.primary_actor_id, 101)
        self.assertLess(snapshot.follow_target_speed_kmh, 30.0)

    def test_uses_route_aligned_target_when_same_lane_relation_is_unavailable(self) -> None:
        ego_waypoint = _FakeWaypoint(
            road_id=15,
            lane_id=1,
            s=0.0,
            location=_FakeLocation(0.0, 0.0, 0.0),
        )
        left_waypoint = _FakeWaypoint(
            road_id=15,
            lane_id=-1,
            s=0.0,
            location=_FakeLocation(0.0, 3.5, 0.0),
        )
        ego_waypoint.set_left_lane(left_waypoint)
        route_waypoint = _FakeWaypoint(
            road_id=13,
            lane_id=-1,
            s=24.0,
            location=_FakeLocation(24.0, 0.0, 0.0),
        )
        route_waypoint.set_left_lane(left_waypoint)
        world_map = _FakeWorldMap(
            {
                (24.0, 0.0, 0.0): route_waypoint,
            }
        )
        route_start_waypoint = _FakeWaypoint(
            road_id=15,
            lane_id=1,
            s=0.0,
            location=_FakeLocation(0.0, -2.0, 0.0),
        )

        snapshot = build_overtake_scene_snapshot(
            tracked_objects=(
                _TrackedActor(
                    actor_id=301,
                    lane_id="13:-1",
                    x_m=24.0,
                    y_m=0.0,
                    speed_mps=0.0,
                    relation="left_lane",
                    is_ahead=True,
                    longitudinal_distance_m=24.0,
                ),
            ),
            timestamp_s=3.0,
            current_speed_mps=5.0,
            current_lane_id="15:1",
            route_target_lane_id="15:1",
            route_index=0,
            ego_waypoint=ego_waypoint,
            adjacent_lanes_open={"left": True, "right": False},
            target_speed_kmh=30.0,
            follow_headway_seconds=1.8,
            stopped_speed_threshold_mps=0.3,
            cluster_merge_gap_m=10.0,
            cluster_max_member_speed_mps=0.5,
            target_policy=build_stopped_obstacle_targets,
            active_signal_state=None,
            signal_stop_distance_m=None,
            allow_overtake=True,
            preferred_direction="left_first",
            world_map=world_map,
            carla_module=_FakeCarla,
            base_trace=[(route_start_waypoint, None), (route_waypoint, None)],
            route_point_to_trace_index=[0, 1],
            route_point_progress_m=[0.0, 24.0],
        )

        self.assertIsNone(snapshot.lead_vehicle)
        self.assertEqual(snapshot.active_target.primary_actor_id, 301)
        self.assertEqual(snapshot.decision_context.active_target.lane_id, "13:-1")
        self.assertEqual(snapshot.lead_distance_m, 24.0)

    def test_build_overtake_pass_snapshot_prefers_route_relative_progress(self) -> None:
        route_start_waypoint = _FakeWaypoint(
            road_id=15,
            lane_id=1,
            s=0.0,
            location=_FakeLocation(0.0, 0.0, 0.0),
        )
        route_mid_waypoint = _FakeWaypoint(
            road_id=15,
            lane_id=1,
            s=10.0,
            location=_FakeLocation(10.0, 0.0, 0.0),
        )
        tracked = (
            _TrackedActor(
                actor_id=401,
                lane_id="15:1",
                x_m=10.0,
                y_m=0.0,
                speed_mps=0.0,
                relation="same_lane",
                is_ahead=True,
                longitudinal_distance_m=8.0,
            ),
        )

        snapshot = build_overtake_pass_snapshot(
            tracked_objects=tracked,
            target_actor_id=401,
            target_member_actor_ids=(401,),
            route_index=0,
            base_trace=[(route_start_waypoint, None), (route_mid_waypoint, None)],
            route_point_to_trace_index=[0, 1],
            route_point_progress_m=[0.0, 10.0],
        )

        self.assertTrue(snapshot.target_actor_visible)
        self.assertEqual(snapshot.target_longitudinal_distance_m, 10.0)
        self.assertEqual(snapshot.target_exit_longitudinal_distance_m, 10.0)


if __name__ == "__main__":
    unittest.main()
