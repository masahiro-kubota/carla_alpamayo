from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from math import ceil
from pathlib import Path
from typing import TYPE_CHECKING, Any

from PIL import Image, ImageDraw

from libs.project import PROJECT_ROOT, relative_to_project

if TYPE_CHECKING:
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

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["image_path"] = relative_to_project(self.image_path)
        payload["metadata_path"] = relative_to_project(self.metadata_path)
        return payload


def _carla_point_to_foxglove_xy(*, x: float, y: float) -> tuple[float, float]:
    return float(x), float(-y)


def _lane_centerlines_all_map(
    world_map: "carla.Map",
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


def build_topdown_map_asset(
    world_map: "carla.Map",
    *,
    output_image_path: Path,
    output_metadata_path: Path,
    pixels_per_meter: float = 4.0,
    padding_m: float = 20.0,
    lane_sampling_m: float = 2.0,
    lane_width_px: int = 3,
    lane_color: tuple[int, int, int] = (108, 117, 125),
    background_color: tuple[int, int, int] = (247, 244, 236),
    frame_id: str = "map/topdown_camera",
    camera_height_m: float = 1000.0,
) -> TopdownMapAsset:
    centerlines = _lane_centerlines_all_map(world_map, lane_sampling_m=lane_sampling_m)
    if not centerlines:
        raise RuntimeError(f"No lane centerlines found for town: {world_map.name}")

    flat_points = [point for line in centerlines for point in line]
    min_x = min(point[0] for point in flat_points) - padding_m
    max_x = max(point[0] for point in flat_points) + padding_m
    min_y = min(point[1] for point in flat_points) - padding_m
    max_y = max(point[1] for point in flat_points) + padding_m
    span_x = max_x - min_x
    span_y = max_y - min_y
    width = max(1, int(ceil(span_x * pixels_per_meter)))
    height = max(1, int(ceil(span_y * pixels_per_meter)))

    image = Image.new("RGB", (width, height), color=background_color)
    draw = ImageDraw.Draw(image)

    def to_pixel(point: tuple[float, float]) -> tuple[int, int]:
        x, y = point
        px = int(round((x - min_x) * pixels_per_meter))
        py = int(round((max_y - y) * pixels_per_meter))
        return px, py

    for line in centerlines:
        pixel_points = [to_pixel(point) for point in line]
        if len(pixel_points) >= 2:
            draw.line(pixel_points, fill=lane_color, width=lane_width_px, joint="curve")

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
    )


def load_default_topdown_map_asset(town: str) -> TopdownMapAsset | None:
    metadata_path = default_topdown_map_metadata_path(town)
    if not metadata_path.exists():
        return None
    return load_topdown_map_asset(metadata_path)
