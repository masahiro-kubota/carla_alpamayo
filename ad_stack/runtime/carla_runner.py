from __future__ import annotations

from dataclasses import dataclass, field

from libs.carla_utils import require_carla


@dataclass(slots=True)
class CarlaRunnerConfig:
    host: str = "127.0.0.1"
    port: int = 2000
    timeout_seconds: float = 30.0
    fixed_delta_seconds: float = 0.05


@dataclass(slots=True)
class CarlaRunner:
    """Thin runtime wrapper for future full-stack episode runners."""

    config: CarlaRunnerConfig = field(default_factory=CarlaRunnerConfig)

    def connect(self):
        carla = require_carla()
        client = carla.Client(self.config.host, self.config.port)
        client.set_timeout(self.config.timeout_seconds)
        return client

    def load_world(self, town_id: str):
        client = self.connect()
        world = client.load_world(town_id)
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.config.fixed_delta_seconds
        world.apply_settings(settings)
        return world
