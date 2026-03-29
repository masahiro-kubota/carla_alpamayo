from __future__ import annotations

import unittest
from dataclasses import dataclass

from simulation.environment_config import EnvironmentConfigSpec
from tests.integration.ad_stack._shared.overtake_scenario_contract import (
    build_overtake_scenario_validation,
    warm_up_and_build_overtake_scenario_validation,
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
        is_junction: bool = False,
    ) -> None:
        self.road_id = road_id
        self.lane_id = lane_id
        self.s = s
        self.transform = _FakeTransform(location=location, rotation=_FakeRotation())
        self.lane_type = lane_type
        self.is_junction = is_junction
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


class _FakeActor:
    def __init__(self, actor_id: int, location: _FakeLocation) -> None:
        self.id = actor_id
        self._location = location

    def get_location(self) -> _FakeLocation:
        return self._location


class _FakeWorldMap:
    def __init__(self, mapping: dict[tuple[float, float, float], _FakeWaypoint]) -> None:
        self._mapping = mapping

    def get_waypoint(self, location: _FakeLocation) -> _FakeWaypoint:
        return self._mapping[(location.x, location.y, location.z)]


class _FakeWorld:
    def __init__(self, world_map: _FakeWorldMap) -> None:
        self._world_map = world_map
        self.tick_count = 0

    def tick(self) -> None:
        self.tick_count += 1

    def get_map(self) -> _FakeWorldMap:
        return self._world_map


class StoppedObstacleScenarioValidationTest(unittest.TestCase):
    def test_returns_missing_obstacle_actor_when_no_npcs(self) -> None:
        environment = EnvironmentConfigSpec(
            name="test",
            town="Town01",
            overtake_scenario={"scenario_kind": "clear"},
        )
        ego_location = _FakeLocation(0.0, 0.0, 0.0)
        ego_waypoint = _FakeWaypoint(road_id=1, lane_id=1, s=0.0, location=ego_location)
        world_map = _FakeWorldMap({(0.0, 0.0, 0.0): ego_waypoint})
        ego_vehicle = _FakeActor(1, ego_location)

        result = build_overtake_scenario_validation(
            environment_config=environment,
            world_map=world_map,
            route_trace=[(ego_waypoint, None)],
            ego_vehicle=ego_vehicle,
            npc_actor_refs=[],
            driving_lane_type="Driving",
        )

        self.assertEqual(result["errors"], ["obstacle_actor_missing"])
        self.assertFalse(result["valid"])

    def test_builds_valid_clear_snapshot(self) -> None:
        ego_location = _FakeLocation(0.0, 0.0, 0.0)
        obstacle_location = _FakeLocation(10.0, 0.0, 0.0)
        ego_waypoint = _FakeWaypoint(road_id=1, lane_id=1, s=0.0, location=ego_location)
        obstacle_waypoint = _FakeWaypoint(
            road_id=1,
            lane_id=1,
            s=10.0,
            location=obstacle_location,
        )
        left_waypoint = _FakeWaypoint(
            road_id=1,
            lane_id=-1,
            s=10.0,
            location=_FakeLocation(10.0, 3.5, 0.0),
        )
        obstacle_waypoint.set_left_lane(left_waypoint)
        world_map = _FakeWorldMap(
            {
                (ego_location.x, ego_location.y, ego_location.z): ego_waypoint,
                (obstacle_location.x, obstacle_location.y, obstacle_location.z): obstacle_waypoint,
            }
        )
        ego_vehicle = _FakeActor(1, ego_location)
        obstacle_actor = _FakeActor(201, obstacle_location)
        environment = EnvironmentConfigSpec(
            name="clear_case",
            town="Town01",
            overtake_scenario={"scenario_kind": "clear"},
        )

        result = build_overtake_scenario_validation(
            environment_config=environment,
            world_map=world_map,
            route_trace=[(ego_waypoint, None), (obstacle_waypoint, None)],
            ego_vehicle=ego_vehicle,
            npc_actor_refs=[obstacle_actor],
            driving_lane_type="Driving",
        )

        self.assertTrue(result["valid"])
        self.assertEqual(result["errors"], [])
        self.assertEqual(result["scenario_kind"], "clear")
        self.assertEqual(result["snapshot"]["ego_lane_id"], "1:1")
        self.assertEqual(result["snapshot"]["obstacle_actor_id"], 201)
        self.assertEqual(result["snapshot"]["obstacle_lane_id"], "1:1")
        self.assertEqual(result["snapshot"]["obstacle_route_lane_id"], "1:1")
        self.assertEqual(result["snapshot"]["route_target_lane_id"], "1:1")
        self.assertTrue(result["snapshot"]["left_lane_is_driving"])
        self.assertFalse(result["snapshot"]["right_lane_is_driving"])
        self.assertAlmostEqual(result["snapshot"]["ego_to_obstacle_longitudinal_distance_m"], 10.0)

    def test_uses_forward_projection_when_same_lane_s_is_not_progressive(self) -> None:
        ego_location = _FakeLocation(0.0, 0.0, 0.0)
        obstacle_location = _FakeLocation(0.0, 10.0, 0.0)
        ego_waypoint = _FakeWaypoint(road_id=1, lane_id=1, s=5.0, location=ego_location)
        ego_waypoint.transform.rotation.yaw = 90.0
        obstacle_waypoint = _FakeWaypoint(
            road_id=1,
            lane_id=1,
            s=5.0,
            location=obstacle_location,
        )
        left_waypoint = _FakeWaypoint(
            road_id=1,
            lane_id=-1,
            s=5.0,
            location=_FakeLocation(3.5, 10.0, 0.0),
        )
        obstacle_waypoint.set_left_lane(left_waypoint)
        world_map = _FakeWorldMap(
            {
                (ego_location.x, ego_location.y, ego_location.z): ego_waypoint,
                (obstacle_location.x, obstacle_location.y, obstacle_location.z): obstacle_waypoint,
            }
        )
        ego_vehicle = _FakeActor(1, ego_location)
        obstacle_actor = _FakeActor(201, obstacle_location)
        environment = EnvironmentConfigSpec(
            name="clear_case",
            town="Town01",
            overtake_scenario={"scenario_kind": "clear"},
        )

        result = build_overtake_scenario_validation(
            environment_config=environment,
            world_map=world_map,
            route_trace=[(ego_waypoint, None), (obstacle_waypoint, None)],
            ego_vehicle=ego_vehicle,
            npc_actor_refs=[obstacle_actor],
            driving_lane_type="Driving",
        )

        self.assertTrue(result["valid"])
        self.assertAlmostEqual(result["snapshot"]["ego_to_obstacle_longitudinal_distance_m"], 10.0)

    def test_warm_up_helper_ticks_world_before_building_snapshot(self) -> None:
        ego_location = _FakeLocation(0.0, 0.0, 0.0)
        obstacle_location = _FakeLocation(10.0, 0.0, 0.0)
        ego_waypoint = _FakeWaypoint(road_id=1, lane_id=1, s=0.0, location=ego_location)
        obstacle_waypoint = _FakeWaypoint(road_id=1, lane_id=1, s=10.0, location=obstacle_location)
        left_waypoint = _FakeWaypoint(
            road_id=1,
            lane_id=-1,
            s=10.0,
            location=_FakeLocation(10.0, 3.5, 0.0),
        )
        obstacle_waypoint.set_left_lane(left_waypoint)
        world_map = _FakeWorldMap(
            {
                (ego_location.x, ego_location.y, ego_location.z): ego_waypoint,
                (obstacle_location.x, obstacle_location.y, obstacle_location.z): obstacle_waypoint,
            }
        )
        world = _FakeWorld(world_map)
        ego_vehicle = _FakeActor(1, ego_location)
        obstacle_actor = _FakeActor(201, obstacle_location)
        environment = EnvironmentConfigSpec(
            name="clear_case",
            town="Town01",
            overtake_scenario={"scenario_kind": "clear"},
        )

        result = warm_up_and_build_overtake_scenario_validation(
            world=world,
            environment_config=environment,
            route_trace=[(ego_waypoint, None), (obstacle_waypoint, None)],
            ego_vehicle=ego_vehicle,
            npc_actor_refs=[obstacle_actor],
            driving_lane_type="Driving",
            warmup_ticks=3,
        )

        self.assertEqual(world.tick_count, 3)
        self.assertTrue(result["valid"])

    def test_route_target_lane_uses_nearest_route_waypoint_to_ego(self) -> None:
        ego_location = _FakeLocation(0.0, 0.0, 0.0)
        obstacle_location = _FakeLocation(10.0, 0.0, 0.0)
        ego_waypoint = _FakeWaypoint(road_id=1, lane_id=1, s=0.0, location=ego_location)
        obstacle_waypoint = _FakeWaypoint(road_id=1, lane_id=1, s=10.0, location=obstacle_location)
        left_waypoint = _FakeWaypoint(
            road_id=1,
            lane_id=-1,
            s=10.0,
            location=_FakeLocation(10.0, 3.5, 0.0),
        )
        route_future_waypoint = _FakeWaypoint(
            road_id=2,
            lane_id=-1,
            s=40.0,
            location=_FakeLocation(100.0, 100.0, 0.0),
        )
        obstacle_waypoint.set_left_lane(left_waypoint)
        world_map = _FakeWorldMap(
            {
                (ego_location.x, ego_location.y, ego_location.z): ego_waypoint,
                (obstacle_location.x, obstacle_location.y, obstacle_location.z): obstacle_waypoint,
            }
        )
        ego_vehicle = _FakeActor(1, ego_location)
        obstacle_actor = _FakeActor(201, obstacle_location)
        environment = EnvironmentConfigSpec(
            name="clear_case",
            town="Town01",
            overtake_scenario={"scenario_kind": "clear"},
        )

        result = build_overtake_scenario_validation(
            environment_config=environment,
            world_map=world_map,
            route_trace=[(route_future_waypoint, None), (ego_waypoint, None), (obstacle_waypoint, None)],
            ego_vehicle=ego_vehicle,
            npc_actor_refs=[obstacle_actor],
            driving_lane_type="Driving",
        )

        self.assertTrue(result["valid"])
        self.assertEqual(result["snapshot"]["route_target_lane_id"], "1:1")

    def test_curve_clear_allows_obstacle_on_future_route_lane(self) -> None:
        ego_location = _FakeLocation(0.0, 0.0, 0.0)
        obstacle_location = _FakeLocation(10.0, 10.0, 0.0)
        ego_waypoint = _FakeWaypoint(road_id=15, lane_id=1, s=0.0, location=ego_location)
        obstacle_waypoint = _FakeWaypoint(
            road_id=13,
            lane_id=-1,
            s=4.0,
            location=obstacle_location,
        )
        left_waypoint = _FakeWaypoint(
            road_id=13,
            lane_id=1,
            s=4.0,
            location=_FakeLocation(13.5, 10.0, 0.0),
        )
        obstacle_waypoint.set_left_lane(left_waypoint)
        world_map = _FakeWorldMap(
            {
                (ego_location.x, ego_location.y, ego_location.z): ego_waypoint,
                (obstacle_location.x, obstacle_location.y, obstacle_location.z): obstacle_waypoint,
            }
        )
        ego_vehicle = _FakeActor(1, ego_location)
        obstacle_actor = _FakeActor(201, obstacle_location)
        route_curve_waypoint = _FakeWaypoint(
            road_id=13,
            lane_id=-1,
            s=4.0,
            location=_FakeLocation(10.0, 10.0, 0.0),
        )
        environment = EnvironmentConfigSpec(
            name="curve_case",
            town="Town01",
            overtake_scenario={"scenario_kind": "curve_clear"},
        )

        result = build_overtake_scenario_validation(
            environment_config=environment,
            world_map=world_map,
            route_trace=[(ego_waypoint, None), (route_curve_waypoint, None)],
            ego_vehicle=ego_vehicle,
            npc_actor_refs=[obstacle_actor],
            driving_lane_type="Driving",
        )

        self.assertTrue(result["valid"])
        self.assertEqual(result["errors"], [])
        self.assertEqual(result["snapshot"]["obstacle_lane_id"], "13:-1")
        self.assertEqual(result["snapshot"]["obstacle_route_lane_id"], "13:-1")


if __name__ == "__main__":
    unittest.main()
