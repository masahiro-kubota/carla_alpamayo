from __future__ import annotations

import numpy as np


def resolve_weather(carla_module, weather_name: str):
    weather = getattr(carla_module.WeatherParameters, weather_name, None)
    if weather is None:
        raise ValueError(f"Unknown CARLA weather preset: {weather_name}")
    return weather


def completion_ratio(initial_waypoint_count: int, remaining_count: int) -> float:
    if initial_waypoint_count <= 0:
        return 0.0
    progress = 1.0 - (remaining_count / initial_waypoint_count)
    return max(0.0, min(1.0, progress))


def carla_image_to_rgb_array(image) -> np.ndarray:
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))
    return array[:, :, :3][:, :, ::-1].copy()


def smooth_steer(
    raw_steer: float,
    previous_steer: float | None,
    *,
    smoothing: float,
    max_delta: float | None,
) -> float:
    if previous_steer is None:
        applied = raw_steer
    else:
        applied = ((1.0 - smoothing) * previous_steer) + (smoothing * raw_steer)
    if previous_steer is not None and max_delta is not None:
        delta = applied - previous_steer
        if delta > max_delta:
            applied = previous_steer + max_delta
        elif delta < -max_delta:
            applied = previous_steer - max_delta
    return max(-1.0, min(1.0, applied))
