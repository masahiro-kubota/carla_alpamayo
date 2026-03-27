from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import math
import queue
import random
from typing import TYPE_CHECKING, Iterable

from libs.project import PROJECT_ROOT, relative_to_project

if TYPE_CHECKING:
    import carla


@dataclass(slots=True)
class FrameEventTracker:
    collision_count: int = 0
    lane_invasion_count: int = 0
    _collision_this_frame: bool = False
    _lane_invasion_this_frame: bool = False
    last_collision_actor_id: int | None = None
    last_collision_actor_type_id: str | None = None

    def mark_collision(self, other_actor=None) -> None:
        self.collision_count += 1
        self._collision_this_frame = True
        if other_actor is not None:
            self.last_collision_actor_id = int(getattr(other_actor, "id", 0)) or None
            self.last_collision_actor_type_id = getattr(other_actor, "type_id", None)

    def mark_lane_invasion(self) -> None:
        self.lane_invasion_count += 1
        self._lane_invasion_this_frame = True

    def consume_frame_flags(self) -> tuple[bool, bool]:
        collision = self._collision_this_frame
        lane_invasion = self._lane_invasion_this_frame
        self._collision_this_frame = False
        self._lane_invasion_this_frame = False
        return collision, lane_invasion


def require_blueprint(
    world: "carla.World",
    pattern: str,
    rng: random.Random | None = None,
) -> "carla.ActorBlueprint":
    blueprints = list(world.get_blueprint_library().filter(pattern))
    if not blueprints:
        raise RuntimeError(f"No blueprint matched pattern: {pattern}")
    if rng is None:
        return blueprints[0]
    return rng.choice(blueprints)


def setup_world(
    client: "carla.Client",
    town: str,
    fixed_delta_seconds: float,
) -> tuple["carla.World", "carla.WorldSettings"]:
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


def build_episode_id(prefix: str) -> str:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{prefix.lower()}_{timestamp}"


def wait_for_image(
    image_queue: "queue.Queue[carla.Image]",
    expected_frame: int,
    timeout: float,
) -> "carla.Image":
    while True:
        image = image_queue.get(timeout=timeout)
        if image.frame >= expected_frame:
            return image


def speed_mps(vehicle: "carla.Vehicle") -> float:
    velocity = vehicle.get_velocity()
    return math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)


def attach_sensor(
    world: "carla.World",
    blueprint_id: str,
    transform: "carla.Transform",
    parent: "carla.Actor",
) -> "carla.Actor":
    blueprint = world.get_blueprint_library().find(blueprint_id)
    return world.spawn_actor(blueprint, transform, attach_to=parent)


def destroy_actors(actors: Iterable["carla.Actor"]) -> None:
    for actor in actors:
        if actor is not None:
            actor.destroy()
