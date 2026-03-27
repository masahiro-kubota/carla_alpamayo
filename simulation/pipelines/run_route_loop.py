from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

from ad_stack import ArtifactSpec, PolicySpec, RouteLoopScenarioSpec, RunRequest, RuntimeSpec, run
from simulation.pipelines.front_camera_preview import FrontCameraPreview, has_display

DEFAULT_ROUTE_CONFIG_PATH = Path("scenarios/routes/town01_pilotnet_loop.json")


def _jsonable(value):
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _build_resolved_request_payload(request: RunRequest) -> dict[str, object]:
    return {
        "mode": request.mode,
        "scenario": _jsonable(asdict(request.scenario)),
        "runtime": _jsonable(asdict(request.runtime)),
        "policy": {
            "kind": request.policy.kind,
            "checkpoint_path": str(request.policy.checkpoint_path) if request.policy.checkpoint_path else None,
            "device": request.policy.device,
            "steer_smoothing": request.policy.steer_smoothing,
            "max_steer_delta": request.policy.max_steer_delta,
            "ignore_traffic_lights": request.policy.ignore_traffic_lights,
            "ignore_stop_signs": request.policy.ignore_stop_signs,
            "ignore_vehicles": request.policy.ignore_vehicles,
            "preview_enabled": request.policy.preview_sink is not None,
        },
        "artifacts": _jsonable(asdict(request.artifacts)),
    }


def _write_run_metadata(
    *,
    output_dir: Path,
    args: argparse.Namespace,
    request: RunRequest,
    summary_path: Path | None,
) -> tuple[Path, Path]:
    cli_args_path = output_dir / "cli_args.json"
    run_request_path = output_dir / "run_request.json"

    cli_args_payload = {
        "entrypoint": "simulation.pipelines.run_route_loop",
        "argv": sys.argv[1:],
        "parsed_args": _jsonable(vars(args)),
    }
    with cli_args_path.open("w", encoding="utf-8") as handle:
        json.dump(cli_args_payload, handle, indent=2)
        handle.write("\n")

    with run_request_path.open("w", encoding="utf-8") as handle:
        json.dump(_build_resolved_request_payload(request), handle, indent=2)
        handle.write("\n")

    if summary_path is not None and summary_path.exists():
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        summary["cli_args_path"] = str(cli_args_path.relative_to(output_dir.parent.parent.parent))
        summary["run_request_path"] = str(run_request_path.relative_to(output_dir.parent.parent.parent))
        summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    return cli_args_path, run_request_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the Town01 fixed loop via ad_stack.run(request)."
    )
    parser.add_argument("--checkpoint", default=None)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--route-config", default=str(DEFAULT_ROUTE_CONFIG_PATH))
    parser.add_argument("--traffic-setup", default=None)
    parser.add_argument("--policy-kind", choices=("expert", "learned"), default=None)
    parser.add_argument("--vehicle-filter", default="vehicle.tesla.model3")
    parser.add_argument("--fixed-delta-seconds", type=float, default=0.05)
    parser.add_argument("--sensor-timeout", type=float, default=2.0)
    parser.add_argument("--target-speed-kmh", type=float, default=30.0)
    parser.add_argument("--camera-width", type=int, default=1280)
    parser.add_argument("--camera-height", type=int, default=720)
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
    parser.add_argument("--video-fps", type=float, default=None)
    parser.add_argument("--video-crf", type=int, default=23)
    parser.add_argument("--record-hz", type=float, default=10.0)
    parser.add_argument(
        "--record-mcap",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--mcap-jpeg-quality", type=int, default=85)
    parser.add_argument("--mcap-map-scope", choices=("full", "near_route"), default="full")
    parser.add_argument(
        "--show-front-camera",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    parser.add_argument("--preview-scale", type=float, default=1.0)
    parser.add_argument(
        "--ignore-traffic-lights",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    parser.add_argument(
        "--ignore-stop-signs",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--ignore-vehicles",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    preview: FrontCameraPreview | None = None
    preview_enabled = args.show_front_camera
    if preview_enabled and not has_display():
        print("DISPLAY is not set, disabling front camera preview. Export DISPLAY=:1 to enable it.")
        preview_enabled = False

    policy_kind = args.policy_kind
    if policy_kind is None:
        policy_kind = "learned" if args.checkpoint else "expert"
    args.policy_kind = policy_kind

    if policy_kind == "learned" and not args.checkpoint:
        parser.error("--checkpoint is required when --policy-kind=learned")
    if policy_kind == "expert" and args.checkpoint:
        parser.error("--checkpoint is only valid for learned evaluation")

    try:
        if preview_enabled:
            preview = FrontCameraPreview(
                source_width=args.camera_width,
                source_height=args.camera_height,
                display_scale=args.preview_scale,
                title="CARLA Front Camera (Route Loop)",
            )
        args.show_front_camera = preview_enabled

        def preview_sink(rgb_array, status_text: str) -> bool | None:
            if preview is not None and not preview.closed:
                preview.update(rgb_array, status_text)
                return True
            if preview is not None and preview.closed:
                return False
            return True

        request = RunRequest(
            mode="evaluate",
            scenario=RouteLoopScenarioSpec(
                route_config_path=Path(args.route_config),
                weather=args.weather,
                goal_tolerance_m=args.goal_tolerance_m,
                max_stop_seconds=args.max_stop_seconds,
                stationary_speed_threshold_mps=args.stationary_speed_threshold_mps,
                max_seconds=args.max_seconds,
                traffic_setup_path=Path(args.traffic_setup) if args.traffic_setup else None,
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
                kind=policy_kind,
                checkpoint_path=Path(args.checkpoint) if args.checkpoint else None,
                device=args.device,
                steer_smoothing=args.steer_smoothing,
                max_steer_delta=args.max_steer_delta,
                ignore_traffic_lights=args.ignore_traffic_lights,
                ignore_stop_signs=args.ignore_stop_signs,
                ignore_vehicles=args.ignore_vehicles,
                preview_sink=preview_sink if preview_enabled else None,
            ),
            artifacts=ArtifactSpec(
                record_video=args.record_video,
                video_fps=args.video_fps,
                video_crf=args.video_crf,
                record_mcap=args.record_mcap,
                mcap_jpeg_quality=args.mcap_jpeg_quality,
                record_hz=args.record_hz,
                mcap_map_scope=args.mcap_map_scope,
            ),
        )
        result = run(request)
        if result.output_dir is not None:
            cli_args_path, run_request_path = _write_run_metadata(
                output_dir=result.output_dir,
                args=args,
                request=request,
                summary_path=result.summary_path,
            )
            result.summary["cli_args_path"] = str(cli_args_path.relative_to(result.output_dir.parent.parent.parent))
            result.summary["run_request_path"] = str(run_request_path.relative_to(result.output_dir.parent.parent.parent))
    finally:
        if preview is not None:
            preview.close()

    print(json.dumps(result.summary, indent=2))


if __name__ == "__main__":
    main()
