from __future__ import annotations

import argparse
import json
from pathlib import Path
import select
import sys
import termios
import tty

from ad_stack import InteractiveScenarioSpec, PolicySpec, RunRequest, RuntimeSpec, run
from simulation.pipelines.front_camera_preview import FrontCameraPreview, has_display

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CHECKPOINT_PATH = REPO_ROOT / "outputs" / "train" / "pilotnet_best" / "best.pt"
DEFAULT_TOWN = "Town01"
DEFAULT_SPAWN_INDEX = 70
COMMAND_KEY_BINDINGS = {
    "w": "lanefollow",
    "a": "left",
    "s": "straight",
    "d": "right",
}


class RawKeyboardInput:
    def __init__(self) -> None:
        self._fd: int | None = None
        self._old_settings: list | None = None

    def __enter__(self) -> "RawKeyboardInput":
        if not sys.stdin.isatty():
            raise SystemExit("interactive_command_drive requires a TTY. Run it from a terminal.")
        self._fd = sys.stdin.fileno()
        self._old_settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._fd is not None and self._old_settings is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)

    def read_keys(self) -> list[str]:
        if self._fd is None:
            return []
        keys: list[str] = []
        while True:
            readable, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not readable:
                break
            chunk = sys.stdin.read(1)
            if not chunk:
                break
            keys.append(chunk)
        return keys

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Drive a learned steering policy with manual W/A/S/D command input on Town01."
    )
    parser.add_argument("--checkpoint", default=str(DEFAULT_CHECKPOINT_PATH))
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--town", default=DEFAULT_TOWN)
    parser.add_argument("--spawn-index", type=int, default=DEFAULT_SPAWN_INDEX)
    parser.add_argument("--vehicle-filter", default="vehicle.tesla.model3")
    parser.add_argument("--fixed-delta-seconds", type=float, default=0.05)
    parser.add_argument("--sensor-timeout", type=float, default=2.0)
    parser.add_argument("--target-speed-kmh", type=float, default=20.0)
    parser.add_argument("--camera-width", type=int, default=1280)
    parser.add_argument("--camera-height", type=int, default=720)
    parser.add_argument("--camera-fov", type=int, default=90)
    parser.add_argument(
        "--show-front-camera",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--preview-scale", type=float, default=1.0)
    parser.add_argument("--weather", default="ClearNoon")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--device", default=None)
    parser.add_argument("--steer-smoothing", type=float, default=1.0)
    parser.add_argument("--max-steer-delta", type=float, default=0.12)
    parser.add_argument("--max-seconds", type=float, default=1800.0)
    parser.add_argument("--spectator-follow-distance-m", type=float, default=7.0)
    parser.add_argument("--spectator-height-m", type=float, default=3.0)
    return parser


def print_instructions() -> None:
    print("Interactive command drive")
    print("  w: lanefollow")
    print("  a: left")
    print("  s: straight")
    print("  d: right")
    print("  q: quit")
    print("Command stays active until you change it.")


def main() -> None:
    args = build_parser().parse_args()
    checkpoint_path = Path(args.checkpoint).resolve()
    if not checkpoint_path.exists():
        raise SystemExit(f"Checkpoint not found: {checkpoint_path}")

    preview: FrontCameraPreview | None = None
    preview_enabled = args.show_front_camera
    if preview_enabled and not has_display():
        print("DISPLAY is not set, disabling front camera preview. Export DISPLAY=:1 to enable it.")
        preview_enabled = False

    result = None
    try:
        if preview_enabled:
            preview = FrontCameraPreview(
                source_width=args.camera_width,
                source_height=args.camera_height,
                display_scale=args.preview_scale,
                title="CARLA Front Camera",
            )

        print_instructions()
        with RawKeyboardInput() as keyboard:
            def command_provider(current_command: str) -> tuple[str, bool]:
                for key in keyboard.read_keys():
                    normalized_key = key.lower()
                    if normalized_key in COMMAND_KEY_BINDINGS:
                        current_command = COMMAND_KEY_BINDINGS[normalized_key]
                    elif normalized_key == "q":
                        return current_command, True
                return current_command, False

            def status_sink(status_text: str) -> None:
                sys.stdout.write(f"\r{status_text}")
                sys.stdout.flush()

            def preview_sink(rgb_array: np.ndarray, status_text: str) -> bool | None:
                if preview is not None and not preview.closed:
                    preview.update(rgb_array, status_text)
                    return True
                if preview is not None and preview.closed:
                    return False
                return True

            result = run(
                RunRequest(
                    mode="interactive",
                    scenario=InteractiveScenarioSpec(
                        town=args.town,
                        spawn_index=args.spawn_index,
                        weather=args.weather,
                        max_seconds=args.max_seconds,
                        spectator_follow_distance_m=args.spectator_follow_distance_m,
                        spectator_height_m=args.spectator_height_m,
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
                        kind="interactive",
                        checkpoint_path=checkpoint_path,
                        device=args.device,
                        steer_smoothing=args.steer_smoothing,
                        max_steer_delta=args.max_steer_delta,
                        initial_command="lanefollow",
                        command_provider=command_provider,
                        status_sink=status_sink,
                        preview_sink=preview_sink if preview_enabled else None,
                    ),
                )
            )
    finally:
        if preview is not None:
            preview.close()

    print()
    if result is not None:
        print(json.dumps(result.summary, indent=2))


if __name__ == "__main__":
    main()
