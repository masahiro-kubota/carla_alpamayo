#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import Counter
import json
from math import hypot
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import carla

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CARLA_AGENTS_ROOT = Path("/home/masa/sim/carla-0.9.16/PythonAPI/carla")
if str(DEFAULT_CARLA_AGENTS_ROOT) not in sys.path:
    sys.path.append(str(DEFAULT_CARLA_AGENTS_ROOT))

from agents.navigation.global_route_planner import GlobalRoutePlanner


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Plot a CARLA route on top of the map topology.")
    parser.add_argument(
        "--route-config",
        default=str(PROJECT_ROOT / "configs" / "routes" / "town01_pilotnet_loop.json"),
    )
    parser.add_argument(
        "--output",
        default=str(PROJECT_ROOT / "docs" / "assets" / "town01_pilotnet_loop.png"),
    )
    parser.add_argument(
        "--summary-output",
        default=str(PROJECT_ROOT / "docs" / "assets" / "town01_pilotnet_loop_summary.json"),
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    return parser


def load_route_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def route_length(points: list[tuple[float, float]]) -> float:
    return sum(
        hypot(x2 - x1, y2 - y1)
        for (x1, y1), (x2, y2) in zip(points[:-1], points[1:])
    )


def unique_waypoint_points(route_trace: list[tuple[carla.Waypoint, object]]) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for waypoint, _road_option in route_trace:
        location = waypoint.transform.location
        point = (location.x, location.y)
        if not points or point != points[-1]:
            points.append(point)
    return points


def plot_topology(ax: plt.Axes, world_map: carla.Map) -> None:
    for entry_waypoint, exit_waypoint in world_map.get_topology():
        xs = [entry_waypoint.transform.location.x, exit_waypoint.transform.location.x]
        ys = [entry_waypoint.transform.location.y, exit_waypoint.transform.location.y]
        ax.plot(xs, ys, color="#d0d4db", linewidth=0.8, alpha=0.8, zorder=1)


def main() -> None:
    args = build_parser().parse_args()
    route_config_path = Path(args.route_config)
    output_path = Path(args.output)
    summary_path = Path(args.summary_output)

    config = load_route_config(route_config_path)
    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)
    world = client.get_world()
    active_town = world.get_map().name.split("/")[-1]
    if active_town != config["town"]:
        world = client.load_world(config["town"])
    world_map = world.get_map()
    spawn_points = world_map.get_spawn_points()
    planner = GlobalRoutePlanner(world_map, config["sampling_resolution_m"])

    anchor_indices: list[int] = config["anchor_spawn_indices"]
    anchor_transforms = [spawn_points[index] for index in anchor_indices]
    segment_pairs = list(zip(anchor_indices[:-1], anchor_indices[1:]))
    if config.get("closed_loop", False):
        segment_pairs.append((anchor_indices[-1], anchor_indices[0]))

    all_route_points: list[tuple[float, float]] = []
    segment_summaries = []
    road_option_counts: Counter[str] = Counter()

    for start_index, end_index in segment_pairs:
        start_transform = spawn_points[start_index]
        end_transform = spawn_points[end_index]
        route_trace = planner.trace_route(start_transform.location, end_transform.location)
        route_points = unique_waypoint_points(route_trace)
        if all_route_points and route_points and all_route_points[-1] == route_points[0]:
            route_points = route_points[1:]
        all_route_points.extend(route_points)
        road_option_counts.update(str(option).split(".")[-1] for _, option in route_trace)
        segment_summaries.append(
            {
                "start_spawn_index": start_index,
                "end_spawn_index": end_index,
                "waypoint_count": len(route_trace),
                "length_m": round(route_length(route_points), 2),
            }
        )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(10, 8))
    plot_topology(ax, world_map)

    route_xs = [point[0] for point in all_route_points]
    route_ys = [point[1] for point in all_route_points]
    ax.plot(route_xs, route_ys, color="#e67e22", linewidth=2.6, zorder=3, label="Loop route")

    for order, (spawn_index, transform) in enumerate(zip(anchor_indices, anchor_transforms), start=1):
        x = transform.location.x
        y = transform.location.y
        ax.scatter([x], [y], color="#c0392b", s=45, zorder=4)
        ax.text(
            x + 4.0,
            y + 4.0,
            f"{order}: spawn {spawn_index}",
            fontsize=8,
            color="#2c3e50",
            zorder=5,
        )

    ax.set_title(f"{config['name']} ({config['town']})")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.invert_yaxis()
    ax.legend(loc="upper right")
    ax.grid(color="#ecf0f1", linewidth=0.5, alpha=0.6)
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)

    summary = {
        "name": config["name"],
        "town": config["town"],
        "description": config["description"],
        "closed_loop": config.get("closed_loop", False),
        "sampling_resolution_m": config["sampling_resolution_m"],
        "anchor_spawn_indices": anchor_indices,
        "anchors": [
            {
                "spawn_index": spawn_index,
                "x": round(transform.location.x, 2),
                "y": round(transform.location.y, 2),
                "yaw_deg": round(transform.rotation.yaw, 1),
            }
            for spawn_index, transform in zip(anchor_indices, anchor_transforms)
        ],
        "segment_summaries": segment_summaries,
        "total_waypoint_count": sum(segment["waypoint_count"] for segment in segment_summaries),
        "total_length_m": round(route_length(all_route_points), 2),
        "road_option_counts": dict(sorted(road_option_counts.items())),
        "plot_path": str(output_path.relative_to(PROJECT_ROOT)),
    }

    with summary_path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)

    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
