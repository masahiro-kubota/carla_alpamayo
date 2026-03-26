from __future__ import annotations

import argparse
from collections import deque
import math
import os
from pathlib import Path
import queue
import select
import sys
import termios
import tty

import numpy as np
from PIL import Image, ImageTk
import tkinter as tk

try:
    import carla
except ModuleNotFoundError:
    carla = None  # type: ignore[assignment]

from ad_stack import load_pilotnet_runtime, select_device
from libs.project import PROJECT_ROOT
from libs.carla_utils import (
    FrameEventTracker,
    attach_sensor,
    destroy_actors,
    require_blueprint,
    setup_world,
    speed_mps,
    wait_for_image,
)
from evaluation.pipelines.common import carla_image_to_rgb_array, resolve_weather, smooth_steer
DEFAULT_CHECKPOINT_PATH = (
    PROJECT_ROOT / "outputs" / "train" / "pilotnet_best" / "best.pt"
)
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


class SpeedController:
    def __init__(
        self,
        *,
        target_speed_kmh: float,
        throttle_gain: float = 0.06,
        brake_gain: float = 0.08,
        derivative_gain: float = 0.01,
        max_throttle: float = 0.65,
        max_brake: float = 0.5,
    ) -> None:
        self.target_speed_kmh = target_speed_kmh
        self.throttle_gain = throttle_gain
        self.brake_gain = brake_gain
        self.derivative_gain = derivative_gain
        self.max_throttle = max_throttle
        self.max_brake = max_brake
        self.previous_error = 0.0

    def step(self, current_speed_mps: float) -> tuple[float, float]:
        current_speed_kmh = current_speed_mps * 3.6
        error = self.target_speed_kmh - current_speed_kmh
        derivative = error - self.previous_error
        self.previous_error = error
        if error >= 0.0:
            throttle = min(self.max_throttle, max(0.0, (self.throttle_gain * error) + (self.derivative_gain * derivative)))
            return throttle, 0.0
        brake = min(self.max_brake, max(0.0, self.brake_gain * (-error)))
        return 0.0, brake


class FrontCameraPreview:
    def __init__(self, *, source_width: int, source_height: int, display_scale: float) -> None:
        self.root = tk.Tk()
        self.root.title("CARLA Front Camera")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.closed = False
        self.image_label = tk.Label(self.root)
        self.image_label.pack()
        self.status_label = tk.Label(
            self.root,
            text="",
            anchor="w",
            justify="left",
            font=("TkFixedFont", 11),
        )
        self.status_label.pack(fill="x")
        self._photo_image: ImageTk.PhotoImage | None = None
        self.source_width = source_width
        self.source_height = source_height
        self.display_width = max(1, int(round(source_width * display_scale)))
        self.display_height = max(1, int(round(source_height * display_scale)))
        self.root.update()

    def update(self, rgb_array: np.ndarray, status_text: str) -> None:
        if self.closed:
            return
        image = Image.fromarray(rgb_array)
        if image.size != (self.display_width, self.display_height):
            image = image.resize((self.display_width, self.display_height))
        self._photo_image = ImageTk.PhotoImage(image=image)
        self.image_label.configure(image=self._photo_image)
        self.status_label.configure(text=status_text)
        self.root.update_idletasks()
        self.root.update()

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        self.root.destroy()


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
    parser.add_argument("--camera-width", type=int, default=320)
    parser.add_argument("--camera-height", type=int, default=180)
    parser.add_argument("--camera-fov", type=int, default=90)
    parser.add_argument(
        "--show-front-camera",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--preview-scale", type=float, default=2.0)
    parser.add_argument("--weather", default="ClearNoon")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--device", default=None)
    parser.add_argument("--steer-smoothing", type=float, default=1.0)
    parser.add_argument("--max-steer-delta", type=float, default=0.12)
    parser.add_argument("--max-seconds", type=float, default=1800.0)
    parser.add_argument("--spectator-follow-distance-m", type=float, default=7.0)
    parser.add_argument("--spectator-height-m", type=float, default=3.0)
    return parser

def update_spectator(
    spectator: "carla.Actor",
    vehicle_transform: "carla.Transform",
    *,
    follow_distance_m: float,
    height_m: float,
) -> None:
    yaw_rad = math.radians(vehicle_transform.rotation.yaw)
    vehicle_location = vehicle_transform.location
    spectator_location = carla.Location(
        x=vehicle_location.x - (follow_distance_m * math.cos(yaw_rad)),
        y=vehicle_location.y - (follow_distance_m * math.sin(yaw_rad)),
        z=vehicle_location.z + height_m,
    )
    spectator_rotation = carla.Rotation(
        pitch=-15.0,
        yaw=vehicle_transform.rotation.yaw,
        roll=0.0,
    )
    spectator.set_transform(carla.Transform(spectator_location, spectator_rotation))


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
    if carla is None:
        raise SystemExit(
            "The 'carla' Python package is not installed. "
            "Run 'uv sync' after confirming the CARLA wheel path in pyproject.toml."
        )

    checkpoint_path = Path(args.checkpoint).resolve()
    if not checkpoint_path.exists():
        raise SystemExit(f"Checkpoint not found: {checkpoint_path}")

    device = select_device(args.device)
    runtime = load_pilotnet_runtime(checkpoint_path, device)
    model_config = runtime.model_config
    if int(model_config.get("route_point_dim", 0)) > 0:
        raise SystemExit("This app only supports checkpoints without route-point conditioning.")

    max_frame_stack = runtime.frame_stack
    frame_events = FrameEventTracker()
    speed_controller = SpeedController(target_speed_kmh=args.target_speed_kmh)
    current_command = "lanefollow"
    previous_applied_steer: float | None = None
    rgb_history: deque[np.ndarray] = deque(maxlen=max_frame_stack)
    preview: FrontCameraPreview | None = None

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    world, original_settings = setup_world(client, args.town, args.fixed_delta_seconds)
    world.set_weather(resolve_weather(carla, args.weather))

    image_queue: queue.Queue[carla.Image] = queue.Queue()
    actors: list[carla.Actor] = []
    frame_index = 0
    elapsed_seconds = 0.0

    try:
        spawn_points = world.get_map().get_spawn_points()
        if args.spawn_index < 0 or args.spawn_index >= len(spawn_points):
            raise SystemExit(f"--spawn-index must be within [0, {len(spawn_points) - 1}]")

        vehicle_blueprint = require_blueprint(world, args.vehicle_filter)
        vehicle_blueprint.set_attribute("role_name", "hero")
        vehicle = world.try_spawn_actor(vehicle_blueprint, spawn_points[args.spawn_index])
        if vehicle is None:
            raise RuntimeError(f"Failed to spawn ego vehicle at spawn index {args.spawn_index}.")
        actors.append(vehicle)

        camera_blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
        camera_blueprint.set_attribute("image_size_x", str(args.camera_width))
        camera_blueprint.set_attribute("image_size_y", str(args.camera_height))
        camera_blueprint.set_attribute("fov", str(args.camera_fov))
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_blueprint, camera_transform, attach_to=vehicle)
        actors.append(camera)
        camera.listen(image_queue.put)

        collision_sensor = attach_sensor(world, "sensor.other.collision", carla.Transform(), vehicle)
        actors.append(collision_sensor)
        collision_sensor.listen(lambda _event: frame_events.mark_collision())

        lane_sensor = attach_sensor(world, "sensor.other.lane_invasion", carla.Transform(), vehicle)
        actors.append(lane_sensor)
        lane_sensor.listen(lambda _event: frame_events.mark_lane_invasion())

        world_frame = world.tick()
        current_image = wait_for_image(image_queue, world_frame, args.sensor_timeout)
        spectator = world.get_spectator()

        if args.show_front_camera:
            if not os.environ.get("DISPLAY"):
                print("DISPLAY is not set, disabling front camera preview. Export DISPLAY=:1 to enable it.")
            else:
                preview = FrontCameraPreview(
                    source_width=args.camera_width,
                    source_height=args.camera_height,
                    display_scale=args.preview_scale,
                )

        print_instructions()
        with RawKeyboardInput() as keyboard:
            while elapsed_seconds < args.max_seconds:
                for key in keyboard.read_keys():
                    normalized_key = key.lower()
                    if normalized_key in COMMAND_KEY_BINDINGS:
                        current_command = COMMAND_KEY_BINDINGS[normalized_key]
                    elif normalized_key == "q":
                        raise KeyboardInterrupt

                rgb_history.append(carla_image_to_rgb_array(current_image))
                current_speed = speed_mps(vehicle)
                predicted_steer_raw = runtime.predict_steer(
                    rgb_history=list(rgb_history),
                    speed_mps=current_speed,
                    command=current_command,
                    route_point=None,
                )
                predicted_steer = smooth_steer(
                    predicted_steer_raw,
                    previous_applied_steer,
                    smoothing=args.steer_smoothing,
                    max_delta=args.max_steer_delta,
                )
                previous_applied_steer = predicted_steer
                throttle, brake = speed_controller.step(current_speed)
                vehicle.apply_control(
                    carla.VehicleControl(
                        throttle=throttle,
                        steer=predicted_steer,
                        brake=brake,
                        hand_brake=False,
                        reverse=False,
                        manual_gear_shift=False,
                    )
                )
                update_spectator(
                    spectator,
                    vehicle.get_transform(),
                    follow_distance_m=args.spectator_follow_distance_m,
                    height_m=args.spectator_height_m,
                )

                collision, lane_invasion = frame_events.consume_frame_flags()
                frame_index += 1
                elapsed_seconds = frame_index * args.fixed_delta_seconds
                status_core = (
                    f"cmd={current_command:<10} speed={current_speed * 3.6:5.1f} km/h "
                    f"steer={predicted_steer:+.3f} throttle={throttle:.2f} brake={brake:.2f} "
                    f"collisions={frame_events.collision_count} lane={frame_events.lane_invasion_count}"
                )
                status = f"\r{status_core}"
                if collision:
                    status += "  [collision]"
                elif lane_invasion:
                    status += "  [lane]"
                sys.stdout.write(status)
                sys.stdout.flush()
                if preview is not None and not preview.closed:
                    preview.update(rgb_history[-1], status_core)
                elif preview is not None and preview.closed:
                    raise KeyboardInterrupt

                world_frame = world.tick()
                current_image = wait_for_image(image_queue, world_frame, args.sensor_timeout)
    except KeyboardInterrupt:
        pass
    finally:
        if preview is not None:
            preview.close()
        world.apply_settings(original_settings)
        destroy_actors(reversed(actors))
        if frame_index:
            print(
                "\n"
                f"ended seconds={elapsed_seconds:.1f} collisions={frame_events.collision_count} "
                f"lane_invasions={frame_events.lane_invasion_count}"
            )


if __name__ == "__main__":
    main()
