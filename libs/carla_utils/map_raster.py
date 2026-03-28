from __future__ import annotations

import json
import queue
from dataclasses import asdict, dataclass
from math import atan, ceil, degrees
from typing import TYPE_CHECKING, Any

from PIL import Image

from libs.project import PROJECT_ROOT, relative_to_project

if TYPE_CHECKING:
    from pathlib import Path

    import carla


@dataclass(slots=True)
class TopdownMapAsset:
    town: str
    image_path: Path
    metadata_path: Path
    frame_id: str
    width: int
    height: int
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    center_x: float
    center_y: float
    pixels_per_meter: float
    padding_m: float
    lane_sampling_m: float
    camera_height_m: float
    camera_fov_deg: float

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["image_path"] = relative_to_project(self.image_path)
        payload["metadata_path"] = relative_to_project(self.metadata_path)
        return payload


def _carla_point_to_foxglove_xy(*, x: float, y: float) -> tuple[float, float]:
    return float(x), float(-y)


def _foxglove_xy_to_carla(*, x: float, y: float) -> tuple[float, float]:
    return float(x), float(-y)


def _lane_centerlines_all_map(
    world_map: carla.Map,
    *,
    lane_sampling_m: float,
) -> list[list[tuple[float, float]]]:
    grouped: dict[tuple[int, int, int], list[tuple[float, float, float]]] = {}
    for waypoint in world_map.generate_waypoints(lane_sampling_m):
        location = waypoint.transform.location
        lane_key = (int(waypoint.road_id), int(waypoint.section_id), int(waypoint.lane_id))
        x_fox, y_fox = _carla_point_to_foxglove_xy(x=float(location.x), y=float(location.y))
        grouped.setdefault(lane_key, []).append(
            (
                float(getattr(waypoint, "s", 0.0)),
                round(x_fox, 3),
                round(y_fox, 3),
            )
        )

    centerlines: list[list[tuple[float, float]]] = []
    for samples in grouped.values():
        samples.sort(key=lambda item: item[0])
        points: list[tuple[float, float]] = []
        for _s, x, y in samples:
            point = (x, y)
            if not points or point != points[-1]:
                points.append(point)
        if len(points) >= 2:
            centerlines.append(points)
    return centerlines


def _compute_topdown_bounds(
    world_map: carla.Map,
    *,
    lane_sampling_m: float,
    padding_m: float,
) -> tuple[float, float, float, float]:
    centerlines = _lane_centerlines_all_map(world_map, lane_sampling_m=lane_sampling_m)
    if not centerlines:
        raise RuntimeError(f"No lane centerlines found for town: {world_map.name}")
    flat_points = [point for line in centerlines for point in line]
    min_x = min(point[0] for point in flat_points) - padding_m
    max_x = max(point[0] for point in flat_points) + padding_m
    min_y = min(point[1] for point in flat_points) - padding_m
    max_y = max(point[1] for point in flat_points) + padding_m
    return min_x, max_x, min_y, max_y


def _capture_rgb_topdown_image(
    world: carla.World,
    *,
    center_x: float,
    center_y: float,
    camera_height_m: float,
    width: int,
    height: int,
    camera_fov_deg: float,
    warmup_ticks: int = 6,
) -> Image.Image:
    import carla

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = original_settings.fixed_delta_seconds or 0.05
    world.apply_settings(settings)

    image_queue: queue.Queue[carla.Image] = queue.Queue()
    sensor: carla.Sensor | None = None
    try:
        blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
        blueprint.set_attribute("image_size_x", str(width))
        blueprint.set_attribute("image_size_y", str(height))
        blueprint.set_attribute("fov", f"{camera_fov_deg:.6f}")
        blueprint.set_attribute("sensor_tick", "0.0")

        capture_x, capture_y = _foxglove_xy_to_carla(x=center_x, y=center_y)
        transform = carla.Transform(
            carla.Location(x=capture_x, y=capture_y, z=camera_height_m),
            carla.Rotation(pitch=-90.0, yaw=0.0, roll=-90.0),
        )
        sensor = world.spawn_actor(blueprint, transform)
        sensor.listen(image_queue.put)

        image: carla.Image | None = None
        for _ in range(max(3, warmup_ticks)):
            world.tick()
            image = image_queue.get(timeout=5.0)
        if image is None:
            raise RuntimeError("Top-down RGB capture did not produce an image.")

        pil_image = Image.frombuffer(
            "RGBA",
            (image.width, image.height),
            bytes(image.raw_data),
            "raw",
            "BGRA",
            0,
            1,
        ).convert("RGB")
        # Flip vertically so image-up aligns with +Y in the stored map/topdown_camera frame.
        return pil_image.transpose(Image.FLIP_TOP_BOTTOM)
    finally:
        if sensor is not None:
            sensor.stop()
            sensor.destroy()
        world.apply_settings(original_settings)


def build_topdown_map_asset(
    world: carla.World,
    *,
    output_image_path: Path,
    output_metadata_path: Path,
    pixels_per_meter: float = 8.0,
    padding_m: float = 20.0,
    lane_sampling_m: float = 2.0,
    frame_id: str = "map/topdown_camera",
    camera_height_m: float = 1000.0,
) -> TopdownMapAsset:
    world_map = world.get_map()
    min_x, max_x, min_y, max_y = _compute_topdown_bounds(
        world_map,
        lane_sampling_m=lane_sampling_m,
        padding_m=padding_m,
    )
    span_x = max_x - min_x
    span_y = max_y - min_y
    width = max(1, int(ceil(span_x * pixels_per_meter)))
    height = max(1, int(ceil(span_y * pixels_per_meter)))
    span_x = max(1e-6, span_x)
    span_y = max(1e-6, span_y)
    required_fov_x = degrees(2.0 * atan(span_x / (2.0 * camera_height_m)))
    required_fov_y = degrees(
        2.0 * atan((span_y * width / max(1, height)) / (2.0 * camera_height_m))
    )
    camera_fov_deg = max(required_fov_x, required_fov_y)
    center_x = (min_x + max_x) / 2.0
    center_y = (min_y + max_y) / 2.0
    image = _capture_rgb_topdown_image(
        world,
        center_x=center_x,
        center_y=center_y,
        camera_height_m=camera_height_m,
        width=width,
        height=height,
        camera_fov_deg=camera_fov_deg,
    )

    output_image_path.parent.mkdir(parents=True, exist_ok=True)
    output_metadata_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_image_path)

    asset = TopdownMapAsset(
        town=world_map.name.split("/")[-1],
        image_path=output_image_path.resolve(),
        metadata_path=output_metadata_path.resolve(),
        frame_id=frame_id,
        width=width,
        height=height,
        min_x=round(min_x, 3),
        max_x=round(max_x, 3),
        min_y=round(min_y, 3),
        max_y=round(max_y, 3),
        center_x=round((min_x + max_x) / 2.0, 3),
        center_y=round((min_y + max_y) / 2.0, 3),
        pixels_per_meter=float(pixels_per_meter),
        padding_m=float(padding_m),
        lane_sampling_m=float(lane_sampling_m),
        camera_height_m=float(camera_height_m),
        camera_fov_deg=float(camera_fov_deg),
    )
    output_metadata_path.write_text(json.dumps(asset.to_dict(), indent=2) + "\n", encoding="utf-8")
    return asset


def default_topdown_map_metadata_path(town: str) -> Path:
    return PROJECT_ROOT / "scenarios" / "maps" / f"{town.lower()}_topdown.json"


def load_topdown_map_asset(path: Path) -> TopdownMapAsset:
    raw = json.loads(path.read_text(encoding="utf-8"))
    return TopdownMapAsset(
        town=str(raw["town"]),
        image_path=(PROJECT_ROOT / raw["image_path"]).resolve(),
        metadata_path=(PROJECT_ROOT / raw["metadata_path"]).resolve(),
        frame_id=str(raw["frame_id"]),
        width=int(raw["width"]),
        height=int(raw["height"]),
        min_x=float(raw["min_x"]),
        max_x=float(raw["max_x"]),
        min_y=float(raw["min_y"]),
        max_y=float(raw["max_y"]),
        center_x=float(raw["center_x"]),
        center_y=float(raw["center_y"]),
        pixels_per_meter=float(raw["pixels_per_meter"]),
        padding_m=float(raw["padding_m"]),
        lane_sampling_m=float(raw["lane_sampling_m"]),
        camera_height_m=float(raw["camera_height_m"]),
        camera_fov_deg=float(raw.get("camera_fov_deg", 0.0)),
    )


def load_default_topdown_map_asset(town: str) -> TopdownMapAsset | None:
    metadata_path = default_topdown_map_metadata_path(town)
    if not metadata_path.exists():
        return None
    return load_topdown_map_asset(metadata_path)
