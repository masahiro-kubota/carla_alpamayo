from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from simulation.environment_config import load_environment_config
from tests.integration.ad_stack._shared.overtake_scenario_contract import (
    parse_overtake_scenario_config,
)


class StoppedObstacleEnvironmentConfigTests(unittest.TestCase):
    def test_load_environment_config_parses_spawn_transform_and_scenario_metadata(self) -> None:
        payload = {
            "name": "stopped_obstacle_fixture",
            "town": "Town01",
            "max_seconds": 60.0,
            "npc_vehicles": [
                {
                    "spawn_transform": {
                        "x": 1.0,
                        "y": 2.0,
                        "z": 0.5,
                        "yaw_deg": 90.0,
                    },
                    "npc_profile_id": "stopped_obstacle_profile_v1",
                    "lane_behavior": "keep_lane",
                }
            ],
            "overtake_scenario": {
                "scenario_kind": "curve_clear",
                "obstacle_npc_index": 0,
                "route_aligned_adjacent_lane_available": True,
                "nearest_junction_distance_m": 42.0,
            },
            "traffic_light_overrides": [],
            "traffic_light_schedules": [],
        }
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "environment.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            config = load_environment_config(path)

        self.assertEqual(config.name, "stopped_obstacle_fixture")
        self.assertEqual(len(config.npc_vehicles), 1)
        npc = config.npc_vehicles[0]
        self.assertIsNone(npc.spawn_index)
        self.assertIsNotNone(npc.spawn_transform)
        assert npc.spawn_transform is not None
        self.assertEqual(npc.spawn_transform.x, 1.0)
        self.assertEqual(npc.spawn_transform.yaw_deg, 90.0)
        self.assertIsNotNone(config.overtake_scenario)
        scenario_config = parse_overtake_scenario_config(config.overtake_scenario)
        assert scenario_config is not None
        self.assertEqual(scenario_config.scenario_kind, "curve_clear")
        self.assertEqual(scenario_config.obstacle_npc_index, 0)
        self.assertTrue(scenario_config.route_aligned_adjacent_lane_available)
        self.assertEqual(scenario_config.nearest_junction_distance_m, 42.0)


if __name__ == "__main__":
    unittest.main()
