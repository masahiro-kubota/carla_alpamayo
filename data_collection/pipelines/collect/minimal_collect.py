from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
import math
from pathlib import Path
import queue
import random
from typing import Iterable

try:
    import carla
except ModuleNotFoundError:
    carla = None  # type: ignore[assignment]

from libs.project import PROJECT_ROOT
from libs.schemas import EpisodeRecord, append_jsonl


@dataclass(slots=True)
class FrameFlags:
    collision: bool = False
    lane_invasion: bool = False
    episode_success: bool = True

    def consume(self) -> tuple[bool, bool]:
        collision = self.collision
        lane_invasion = self.lane_invasion
        self.collision = False
        self.lane_invasion = False
        return collision, lane_invasion


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Collect a minimal CARLA episode.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--traffic-manager-port", type=int, default=8000)
    parser.add_argument("--town", default="Town01")
    parser.add_argument("--vehicle-filter", default="vehicle.tesla.model3")
    parser.add_argument("--frames", type=int, default=300)
    parser.add_argument("--fixed-delta-seconds", type=float, default=0.05)
    parser.add_argument("--sensor-timeout", type=float, default=2.0)
    parser.add_argument("--image-width", type=int, default=1280)
    parser.add_argument("--image-height", type=int, default=720)
    parser.add_argument("--image-fov", type=int, default=90)
    parser.add_argument("--command", default="lane_follow")
    parser.add_argument("--seed", type=int, default=7)
    return parser


def require_blueprint(world: carla.World, pattern: str) -> carla.ActorBlueprint:
    blueprints = world.get_blueprint_library().filter(pattern)
    if not blueprints:
        raise RuntimeError(f"No blueprint matched pattern: {pattern}")
    return random.choice(blueprints)


def setup_world(client: carla.Client, town: str, fixed_delta_seconds: float) -> tuple[carla.World, carla.WorldSettings]:
    world = client.get_world()
    active_town = world.get_map().name.split("/")[-1]
    if active_town != town:
        world = client.load_world(town)

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = fixed_delta_seconds
    world.apply_settings(settings)
    return world, original_settings


def choose_spawn_transform(world: carla.World, seed: int) -> carla.Transform:
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("No spawn points are available in the loaded map.")

    rng = random.Random(seed)
    return rng.choice(spawn_points)


def build_episode_id(town: str) -> str:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{town.lower()}_{timestamp}"


def wait_for_image(image_queue: queue.Queue[carla.Image], expected_frame: int, timeout: float) -> carla.Image:
    while True:
        image = image_queue.get(timeout=timeout)
        if image.frame >= expected_frame:
            return image


def speed_mps(vehicle: carla.Vehicle) -> float:
    velocity = vehicle.get_velocity()
    return math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)


def relative_to_project(path: Path) -> str:
    return str(path.relative_to(PROJECT_ROOT))


def attach_sensor(
    world: carla.World,
    blueprint_id: str,
    transform: carla.Transform,
    parent: carla.Actor,
) -> carla.Actor:
    blueprint = world.get_blueprint_library().find(blueprint_id)
    return world.spawn_actor(blueprint, transform, attach_to=parent)


def destroy_actors(actors: Iterable[carla.Actor]) -> None:
    for actor in actors:
        if actor is not None:
            actor.destroy()


def main() -> None:
    args = build_parser().parse_args()
    if carla is None:
        raise SystemExit(
            "The 'carla' Python package is not installed. "
            "Install the wheel from ~/sim/carla-0.9.16/PythonAPI/carla/dist first."
        )

    random.seed(args.seed)

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    world, original_settings = setup_world(client, args.town, args.fixed_delta_seconds)
    traffic_manager = client.get_trafficmanager(args.traffic_manager_port)
    traffic_manager.set_synchronous_mode(True)

    episode_id = build_episode_id(args.town)
    image_dir = PROJECT_ROOT / "outputs" / "collect" / episode_id / "front_rgb"
    manifest_path = PROJECT_ROOT / "data" / "manifests" / "episodes" / f"{episode_id}.jsonl"
    image_dir.mkdir(parents=True, exist_ok=True)

    frame_flags = FrameFlags()
    image_queue: queue.Queue[carla.Image] = queue.Queue()
    actors: list[carla.Actor] = []

    try:
        vehicle_blueprint = require_blueprint(world, args.vehicle_filter)
        vehicle_blueprint.set_attribute("role_name", "hero")
        spawn_transform = choose_spawn_transform(world, args.seed)
        vehicle = world.spawn_actor(vehicle_blueprint, spawn_transform)
        actors.append(vehicle)
        vehicle.set_autopilot(True, traffic_manager.get_port())

        camera_blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
        camera_blueprint.set_attribute("image_size_x", str(args.image_width))
        camera_blueprint.set_attribute("image_size_y", str(args.image_height))
        camera_blueprint.set_attribute("fov", str(args.image_fov))
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_blueprint, camera_transform, attach_to=vehicle)
        actors.append(camera)
        camera.listen(image_queue.put)

        collision_sensor = attach_sensor(
            world,
            "sensor.other.collision",
            carla.Transform(),
            vehicle,
        )
        actors.append(collision_sensor)
        collision_sensor.listen(lambda _event: setattr(frame_flags, "collision", True))

        lane_sensor = attach_sensor(
            world,
            "sensor.other.lane_invasion",
            carla.Transform(),
            vehicle,
        )
        actors.append(lane_sensor)
        lane_sensor.listen(lambda _event: setattr(frame_flags, "lane_invasion", True))

        for frame_index in range(args.frames):
            world_frame = world.tick()
            image = wait_for_image(image_queue, world_frame, args.sensor_timeout)

            image_path = image_dir / f"{frame_index:06d}.png"
            image.save_to_disk(str(image_path))

            collision, lane_invasion = frame_flags.consume()
            if collision:
                frame_flags.episode_success = False

            control = vehicle.get_control()
            record = EpisodeRecord(
                episode_id=episode_id,
                frame_id=frame_index,
                town_id=args.town,
                route_id=f"{args.town.lower()}_random_seed_{args.seed}",
                weather_id="default",
                timestamp=image.timestamp,
                front_rgb_path=relative_to_project(image_path),
                speed=speed_mps(vehicle),
                command=args.command,
                steer=control.steer,
                throttle=control.throttle,
                brake=control.brake,
                collision=collision,
                lane_invasion=lane_invasion,
                success=frame_flags.episode_success,
            )
            append_jsonl(manifest_path, record)

            if collision:
                break
    finally:
        traffic_manager.set_synchronous_mode(False)
        world.apply_settings(original_settings)
        destroy_actors(reversed(actors))


if __name__ == "__main__":
    main()
