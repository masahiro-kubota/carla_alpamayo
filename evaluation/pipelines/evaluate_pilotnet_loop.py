from __future__ import annotations

import argparse
import json
from pathlib import Path

from ad_stack import ArtifactSpec, PolicySpec, RouteLoopScenarioSpec, RunRequest, RuntimeSpec, run

DEFAULT_ROUTE_CONFIG_PATH = Path("data_collection/configs/routes/town01_pilotnet_loop.json")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Evaluate a learned PilotNet steer policy on the Town01 fixed loop.")
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--route-config", default=str(DEFAULT_ROUTE_CONFIG_PATH))
    parser.add_argument("--vehicle-filter", default="vehicle.tesla.model3")
    parser.add_argument("--fixed-delta-seconds", type=float, default=0.05)
    parser.add_argument("--sensor-timeout", type=float, default=2.0)
    parser.add_argument("--target-speed-kmh", type=float, default=30.0)
    parser.add_argument("--camera-width", type=int, default=320)
    parser.add_argument("--camera-height", type=int, default=180)
    parser.add_argument("--camera-fov", type=int, default=90)
    parser.add_argument("--goal-tolerance-m", type=float, default=10.0)
    parser.add_argument("--max-stop-seconds", type=float, default=10.0)
    parser.add_argument("--stationary-speed-threshold-mps", type=float, default=0.5)
    parser.add_argument("--max-seconds", type=float, default=600.0)
    parser.add_argument("--weather", default="ClearNoon")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--device", default=None)
    parser.add_argument("--steer-smoothing", type=float, default=1.0)
    parser.add_argument("--max-steer-delta", type=float, default=None)
    parser.add_argument(
        "--record-video",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--video-crf", type=int, default=23)
    parser.add_argument(
        "--ignore-traffic-lights",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--ignore-stop-signs",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--ignore-vehicles",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    request = RunRequest(
        mode="evaluate",
        scenario=RouteLoopScenarioSpec(
            route_config_path=Path(args.route_config),
            weather=args.weather,
            goal_tolerance_m=args.goal_tolerance_m,
            max_stop_seconds=args.max_stop_seconds,
            stationary_speed_threshold_mps=args.stationary_speed_threshold_mps,
            max_seconds=args.max_seconds,
        ),
        runtime=RuntimeSpec(
            host=args.host,
            port=args.port,
            vehicle_filter=args.vehicle_filter,
            fixed_delta_seconds=args.fixed_delta_seconds,
            sensor_timeout=args.sensor_timeout,
            camera_width=args.camera_width,
            camera_height=args.camera_height,
            camera_fov=args.camera_fov,
            target_speed_kmh=args.target_speed_kmh,
            seed=args.seed,
        ),
        policy=PolicySpec(
            kind="learned",
            checkpoint_path=Path(args.checkpoint),
            device=args.device,
            steer_smoothing=args.steer_smoothing,
            max_steer_delta=args.max_steer_delta,
            ignore_traffic_lights=args.ignore_traffic_lights,
            ignore_stop_signs=args.ignore_stop_signs,
            ignore_vehicles=args.ignore_vehicles,
        ),
        artifacts=ArtifactSpec(
            record_video=args.record_video,
            video_crf=args.video_crf,
        ),
    )
    result = run(request)
    print(json.dumps(result.summary, indent=2))


if __name__ == "__main__":
    main()
