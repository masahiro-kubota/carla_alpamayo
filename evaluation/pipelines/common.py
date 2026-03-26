from __future__ import annotations

import numpy as np


def resolve_weather(carla_module, weather_name: str):
    weather = getattr(carla_module.WeatherParameters, weather_name, None)
    if weather is None:
        raise ValueError(f"Unknown CARLA weather preset: {weather_name}")
    return weather


def carla_image_to_rgb_array(image) -> np.ndarray:
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))
    return array[:, :, :3][:, :, ::-1].copy()
