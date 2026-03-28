from __future__ import annotations

import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest

from simulation.pipelines.route_loop_run_config import load_route_loop_run_config
from simulation.pipelines.run_route_loop import build_parser


def _valid_run_config_payload() -> dict:
    return {
        "name": "town01_perimeter_cw_expert",
        "description": "Unit test payload.",
        "mode": "evaluate",
        "scenario": {
            "route_config_path": "scenarios/routes/town01_perimeter_cw.json",
            "weather": "ClearNoon",
            "goal_tolerance_m": 10.0,
            "max_stop_seconds": 10.0,
            "stationary_speed_threshold_mps": 0.5,
            "max_seconds": 120.0,
            "environment_config_path": "scenarios/environments/town01_perimeter_ten_npc_120s.json",
        },
        "runtime": {
            "host": "127.0.0.1",
            "port": 2000,
            "vehicle_filter": "vehicle.tesla.model3",
            "fixed_delta_seconds": 0.05,
            "sensor_timeout": 2.0,
            "camera_width": 1280,
            "camera_height": 720,
            "camera_fov": 90,
            "target_speed_kmh": None,
            "seed": 7,
        },
        "policy": {
            "kind": "expert",
            "checkpoint_path": None,
            "expert_config_path": "ad_stack/configs/expert/default.json",
            "device": None,
            "steer_smoothing": 1.0,
            "max_steer_delta": None,
            "ignore_traffic_lights": False,
            "ignore_stop_signs": True,
            "ignore_vehicles": False,
        },
        "artifacts": {
            "record_video": True,
            "video_crf": 23,
            "video_fps": None,
            "record_mcap": True,
            "mcap_jpeg_quality": 85,
            "mcap_segment_seconds": 600.0,
            "record_hz": 10.0,
            "mcap_map_scope": "full",
        },
        "preview": {
            "show_front_camera": False,
            "preview_scale": 1.0,
        },
    }


class RouteLoopRunConfigTests(unittest.TestCase):
    def test_load_route_loop_run_config_builds_request(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            config_path = Path(temp_dir) / "run_config.json"
            config_path.write_text(json.dumps(_valid_run_config_payload()), encoding="utf-8")

            loaded = load_route_loop_run_config(config_path)

        self.assertEqual(loaded.request.mode, "evaluate")
        self.assertEqual(loaded.request.policy.kind, "expert")
        self.assertEqual(loaded.request.runtime.port, 2000)
        self.assertEqual(str(loaded.request.scenario.route_config_path), "scenarios/routes/town01_perimeter_cw.json")
        self.assertFalse(loaded.preview.show_front_camera)

    def test_load_route_loop_run_config_rejects_unknown_key(self) -> None:
        payload = _valid_run_config_payload()
        payload["runtime"]["unexpected"] = 1
        with tempfile.TemporaryDirectory() as temp_dir:
            config_path = Path(temp_dir) / "run_config.json"
            config_path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "unsupported keys"):
                load_route_loop_run_config(config_path)

    def test_build_parser_rejects_non_json_cli_overrides(self) -> None:
        parser = build_parser()
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                parser.parse_args(["--port", "2000"])

    def test_build_parser_accepts_multiple_json_paths(self) -> None:
        parser = build_parser()
        args = parser.parse_args(["a.json", "b.json", "c.json"])
        self.assertEqual(args.config_paths, ["a.json", "b.json", "c.json"])


if __name__ == "__main__":
    unittest.main()
