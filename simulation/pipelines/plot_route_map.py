from __future__ import annotations

import argparse
from collections import defaultdict
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
from matplotlib import pyplot as plt
from matplotlib.collections import LineCollection

from libs.carla_utils import build_planned_route, load_route_config, require_carla
from libs.project import PROJECT_ROOT, relative_to_project

DEFAULT_ROUTES_DIR = PROJECT_ROOT / "scenarios" / "routes"
DEFAULT_OUTPUT_DIR = DEFAULT_ROUTES_DIR / "previews"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Render route JSONs onto the CARLA map so each scenarios/routes/*.json has a human-readable PNG preview."
        )
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument(
        "--route-config",
        action="append",
        default=[],
        help="Route JSON to render. Repeatable. If omitted, use --all.",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Render every JSON under scenarios/routes/.",
    )
    parser.add_argument(
        "--output-dir",
        default=str(DEFAULT_OUTPUT_DIR),
        help="Directory where preview PNGs are written.",
    )
    parser.add_argument(
        "--map-scope",
        choices=("full", "near_route"),
        default="full",
        help="Draw the whole map or only lane centerlines near the route corridor.",
    )
    parser.add_argument("--lane-sampling-m", type=float, default=5.0)
    parser.add_argument("--corridor-margin-m", type=float, default=35.0)
    parser.add_argument("--padding-m", type=float, default=20.0)
    parser.add_argument("--dpi", type=int, default=180)
    return parser


def _resolve_route_paths(args: argparse.Namespace) -> list[Path]:
    explicit_paths = [Path(path).resolve() for path in args.route_config]
    if args.all:
        explicit_paths.extend(sorted(DEFAULT_ROUTES_DIR.glob("*.json")))
    unique_paths: list[Path] = []
    seen: set[Path] = set()
    for path in explicit_paths:
        resolved = path.resolve()
        if resolved in seen:
            continue
        if not resolved.exists():
            raise FileNotFoundError(f"Route config not found: {resolved}")
        unique_paths.append(resolved)
        seen.add(resolved)
    if not unique_paths:
        raise SystemExit("Specify --route-config PATH or pass --all.")
    return unique_paths


def _load_world_for_town(client: Any, town: str) -> Any:
    world = client.get_world()
    active_town = world.get_map().name.split("/")[-1]
    if active_town != town:
        world = client.load_world(town)
    return world


def _lane_centerlines_all_map(
    world_map: Any,
    *,
    lane_sampling_m: float,
) -> list[list[tuple[float, float]]]:
    grouped: dict[tuple[int, int, int], list[tuple[float, float, float]]] = defaultdict(list)
    for waypoint in world_map.generate_waypoints(lane_sampling_m):
        location = waypoint.transform.location
        lane_key = (int(waypoint.road_id), int(waypoint.section_id), int(waypoint.lane_id))
        grouped[lane_key].append(
            (
                float(getattr(waypoint, "s", 0.0)),
                float(location.x),
                float(location.y),
            )
        )

    centerlines: list[list[tuple[float, float]]] = []
    for samples in grouped.values():
        samples.sort(key=lambda item: item[0])
        points: list[tuple[float, float]] = []
        for _s, x, y in samples:
            point = (round(x, 3), round(y, 3))
            if not points or point != points[-1]:
                points.append(point)
        if len(points) >= 2:
            centerlines.append(points)
    return centerlines


def _lane_centerlines_near_route(
    world_map: Any,
    route_trace: list[tuple[Any, Any]],
    *,
    lane_sampling_m: float,
    corridor_margin_m: float,
) -> list[list[tuple[float, float]]]:
    route_locations = [waypoint.transform.location for waypoint, _ in route_trace]
    if not route_locations:
        return []

    min_x = min(location.x for location in route_locations) - corridor_margin_m
    max_x = max(location.x for location in route_locations) + corridor_margin_m
    min_y = min(location.y for location in route_locations) - corridor_margin_m
    max_y = max(location.y for location in route_locations) + corridor_margin_m
    max_distance_sq = corridor_margin_m * corridor_margin_m

    grouped: dict[tuple[int, int, int], list[tuple[float, float, float]]] = defaultdict(list)
    for waypoint in world_map.generate_waypoints(lane_sampling_m):
        location = waypoint.transform.location
        if location.x < min_x or location.x > max_x or location.y < min_y or location.y > max_y:
            continue

        min_route_distance_sq = min(
            (float(location.x - route_location.x) ** 2 + float(location.y - route_location.y) ** 2)
            for route_location in route_locations
        )
        if min_route_distance_sq > max_distance_sq:
            continue

        lane_key = (int(waypoint.road_id), int(waypoint.section_id), int(waypoint.lane_id))
        grouped[lane_key].append(
            (
                float(getattr(waypoint, "s", 0.0)),
                float(location.x),
                float(location.y),
            )
        )

    centerlines: list[list[tuple[float, float]]] = []
    for samples in grouped.values():
        samples.sort(key=lambda item: item[0])
        points: list[tuple[float, float]] = []
        for _s, x, y in samples:
            point = (round(x, 3), round(y, 3))
            if not points or point != points[-1]:
                points.append(point)
        if len(points) >= 2:
            centerlines.append(points)
    return centerlines


def _segments_from_lines(lines: list[list[tuple[float, float]]]) -> list[list[tuple[float, float]]]:
    segments: list[list[tuple[float, float]]] = []
    for line in lines:
        segments.extend([[start, end] for start, end in zip(line[:-1], line[1:])])
    return segments


def _add_direction_arrows(ax: Any, points: list[tuple[float, float]]) -> None:
    if len(points) < 3:
        return
    arrow_positions = 6
    for arrow_index in range(1, arrow_positions + 1):
        point_index = int((len(points) - 2) * arrow_index / (arrow_positions + 1))
        start = points[point_index]
        end = points[point_index + 1]
        ax.annotate(
            "",
            xy=end,
            xytext=start,
            arrowprops={
                "arrowstyle": "->",
                "color": "#d97706",
                "lw": 1.6,
                "shrinkA": 0.0,
                "shrinkB": 0.0,
            },
            zorder=5,
        )


def _plot_route(
    *,
    route_path: Path,
    route_config: Any,
    planned_route: Any,
    lane_centerlines: list[list[tuple[float, float]]],
    output_path: Path,
    padding_m: float,
    dpi: int,
) -> None:
    figure, ax = plt.subplots(figsize=(10, 10), dpi=dpi)
    figure.patch.set_facecolor("#f7f4ec")
    ax.set_facecolor("#fbfaf7")

    lane_segments = _segments_from_lines(lane_centerlines)
    if lane_segments:
        ax.add_collection(
            LineCollection(
                lane_segments,
                colors="#cfc8bb",
                linewidths=0.6,
                alpha=0.9,
                zorder=1,
            )
        )

    route_points = list(planned_route.xy_points)
    route_segments = [[start, end] for start, end in zip(route_points[:-1], route_points[1:])]
    if route_segments:
        ax.add_collection(
            LineCollection(
                route_segments,
                colors="#d97706",
                linewidths=2.8,
                alpha=0.95,
                zorder=4,
            )
        )
        _add_direction_arrows(ax, route_points)

    anchor_points = [
        (float(transform.location.x), float(transform.location.y))
        for transform in planned_route.anchor_transforms
    ]
    if anchor_points:
        anchor_xs = [point[0] for point in anchor_points]
        anchor_ys = [point[1] for point in anchor_points]
        ax.scatter(anchor_xs, anchor_ys, s=32, c="#2563eb", edgecolors="white", linewidths=0.8, zorder=6)
        start_x, start_y = anchor_points[0]
        ax.scatter([start_x], [start_y], s=72, c="#16a34a", edgecolors="white", linewidths=1.0, zorder=7)
        for anchor_order, ((anchor_x, anchor_y), spawn_index) in enumerate(
            zip(anchor_points, route_config.anchor_spawn_indices),
            start=1,
        ):
            ax.text(
                anchor_x + 2.0,
                anchor_y + 2.0,
                f"{anchor_order}:{spawn_index}",
                fontsize=8,
                color="#1f2937",
                zorder=8,
                bbox={
                    "facecolor": "#ffffffcc",
                    "edgecolor": "#d6d3d1",
                    "linewidth": 0.6,
                    "boxstyle": "round,pad=0.2",
                },
            )

    if route_points:
        xs = [point[0] for point in route_points]
        ys = [point[1] for point in route_points]
        ax.set_xlim(min(xs) - padding_m, max(xs) + padding_m)
        ax.set_ylim(min(ys) - padding_m, max(ys) + padding_m)
    elif lane_centerlines:
        flat_xs = [point[0] for line in lane_centerlines for point in line]
        flat_ys = [point[1] for line in lane_centerlines for point in line]
        ax.set_xlim(min(flat_xs) - padding_m, max(flat_xs) + padding_m)
        ax.set_ylim(min(flat_ys) - padding_m, max(flat_ys) + padding_m)

    ax.set_aspect("equal", adjustable="box")
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)

    anchor_text = ", ".join(str(index) for index in route_config.anchor_spawn_indices)
    title = f"{route_config.name} ({route_config.town})"
    subtitle = (
        f"{route_config.description}\n"
        f"anchors=[{anchor_text}]  length={planned_route.total_length_m:.1f}m  "
        f"segments={len(planned_route.segment_pairs)}  closed_loop={route_config.closed_loop}"
    )
    ax.set_title(title, fontsize=14, color="#111827", pad=16)
    figure.text(0.5, 0.955, subtitle, ha="center", va="top", fontsize=9, color="#4b5563")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)


def main() -> None:
    args = build_parser().parse_args()
    route_paths = _resolve_route_paths(args)

    carla = require_carla()
    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)

    output_dir = Path(args.output_dir).resolve()
    centerline_cache: dict[tuple[str, str], list[list[tuple[float, float]]]] = {}

    for route_path in route_paths:
        route_config = load_route_config(route_path)
        world = _load_world_for_town(client, route_config.town)
        world_map = world.get_map()
        planned_route = build_planned_route(world_map, route_config)

        cache_key = (route_config.town, args.map_scope)
        if cache_key not in centerline_cache:
            centerline_cache[cache_key] = (
                _lane_centerlines_all_map(world_map, lane_sampling_m=args.lane_sampling_m)
                if args.map_scope == "full"
                else _lane_centerlines_near_route(
                    world_map,
                    planned_route.trace,
                    lane_sampling_m=args.lane_sampling_m,
                    corridor_margin_m=args.corridor_margin_m,
                )
            )
        lane_centerlines = centerline_cache[cache_key]
        if args.map_scope == "near_route":
            lane_centerlines = _lane_centerlines_near_route(
                world_map,
                planned_route.trace,
                lane_sampling_m=args.lane_sampling_m,
                corridor_margin_m=args.corridor_margin_m,
            )

        output_path = output_dir / f"{route_config.name}.png"
        _plot_route(
            route_path=route_path,
            route_config=route_config,
            planned_route=planned_route,
            lane_centerlines=lane_centerlines,
            output_path=output_path,
            padding_m=args.padding_m,
            dpi=args.dpi,
        )
        print(
            f"{relative_to_project(route_path)} -> {relative_to_project(output_path)} "
            f"(length={planned_route.total_length_m:.1f}m)"
        )


if __name__ == "__main__":
    main()
