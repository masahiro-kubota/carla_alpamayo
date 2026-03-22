#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass
import json
from math import hypot
from pathlib import Path
import sys
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import carla

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CARLA_AGENTS_ROOT = Path("/home/masa/sim/carla-0.9.16/PythonAPI/carla")
if str(DEFAULT_CARLA_AGENTS_ROOT) not in sys.path:
    sys.path.append(str(DEFAULT_CARLA_AGENTS_ROOT))

from agents.navigation.global_route_planner import GlobalRoutePlanner


TURN_OPTIONS = {"LEFT", "RIGHT", "STRAIGHT"}


@dataclass(slots=True)
class CandidateRoute:
    movement_key: str
    movement_label: str
    junction_id: int | None
    start_spawn_index: int
    end_spawn_index: int
    total_length_m: float
    euclidean_distance_m: float
    approach_length_m: float
    turn_length_m: float
    exit_length_m: float
    total_waypoint_count: int
    turn_waypoint_count: int
    center_x: float
    center_y: float
    entry_road_id: int
    entry_lane_id: int
    exit_road_id: int
    exit_lane_id: int
    road_option_counts: dict[str, int]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build a Town01 junction movement inventory and recommended train/eval routes.")
    parser.add_argument("--town", default="Town01")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--sampling-resolution-m", type=float, default=2.0)
    parser.add_argument("--min-euclidean-m", type=float, default=20.0)
    parser.add_argument("--max-euclidean-m", type=float, default=220.0)
    parser.add_argument("--min-approach-m", type=float, default=20.0)
    parser.add_argument("--min-exit-m", type=float, default=20.0)
    parser.add_argument("--inventory-output", default=str(PROJECT_ROOT / "docs" / "assets" / "town01_movement_inventory.json"))
    parser.add_argument("--plot-output", default=str(PROJECT_ROOT / "docs" / "assets" / "town01_movement_inventory.png"))
    parser.add_argument("--train-config-dir", default=str(PROJECT_ROOT / "configs" / "routes" / "town01_movement_train"))
    parser.add_argument("--eval-config-dir", default=str(PROJECT_ROOT / "configs" / "routes" / "town01_movement_eval"))
    return parser


def road_option_name(option: Any) -> str:
    if option is None:
        return "VOID"
    if hasattr(option, "name"):
        return str(option.name).upper()
    return str(option).split(".")[-1].upper()


def route_length(points: list[tuple[float, float]]) -> float:
    return sum(hypot(x2 - x1, y2 - y1) for (x1, y1), (x2, y2) in zip(points[:-1], points[1:]))


def unique_waypoint_points(route_trace: list[tuple[carla.Waypoint, Any]]) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for waypoint, _ in route_trace:
        location = waypoint.transform.location
        point = (location.x, location.y)
        if not points or point != points[-1]:
            points.append(point)
    return points


def run_segments(commands: list[str]) -> list[tuple[str, int, int]]:
    segments: list[tuple[str, int, int]] = []
    if not commands:
        return segments
    current = commands[0]
    start = 0
    for index, command in enumerate(commands[1:], start=1):
        if command != current:
            segments.append((current, start, index - 1))
            current = command
            start = index
    segments.append((current, start, len(commands) - 1))
    return segments


def segment_points(route_trace: list[tuple[carla.Waypoint, Any]], start_index: int, end_index: int) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for waypoint, _ in route_trace[start_index : end_index + 1]:
        location = waypoint.transform.location
        point = (location.x, location.y)
        if not points or point != points[-1]:
            points.append(point)
    return points


def junction_waypoint_for_segment(route_trace: list[tuple[carla.Waypoint, Any]], start_index: int, end_index: int) -> carla.Waypoint | None:
    for waypoint, _ in route_trace[start_index : end_index + 1]:
        if waypoint.is_junction:
            return waypoint
    return None


def movement_key_for_segment(
    movement_label: str,
    route_trace: list[tuple[carla.Waypoint, Any]],
    start_index: int,
    end_index: int,
) -> tuple[str, int | None, float, float, int, int, int, int]:
    junction_waypoint = junction_waypoint_for_segment(route_trace, start_index, end_index)
    if junction_waypoint is not None:
        junction = junction_waypoint.get_junction()
        center = junction.bounding_box.location
        junction_id = junction.id
    else:
        points = segment_points(route_trace, start_index, end_index)
        avg_x = sum(point[0] for point in points) / len(points)
        avg_y = sum(point[1] for point in points) / len(points)
        center = carla.Location(avg_x, avg_y, 0.0)
        junction_id = None

    entry_waypoint = route_trace[start_index][0]
    exit_waypoint = route_trace[end_index][0]
    key = (
        movement_label,
        junction_id,
        round(center.x, 1),
        round(center.y, 1),
        entry_waypoint.road_id,
        entry_waypoint.lane_id,
        exit_waypoint.road_id,
        exit_waypoint.lane_id,
    )
    return key


def candidate_route_from_trace(
    start_index: int,
    end_index: int,
    start_transform: carla.Transform,
    end_transform: carla.Transform,
    route_trace: list[tuple[carla.Waypoint, Any]],
) -> CandidateRoute | None:
    commands = [road_option_name(option) for _, option in route_trace]
    turn_segments = [segment for segment in run_segments(commands) if segment[0] in TURN_OPTIONS]
    if len(turn_segments) != 1:
        return None
    movement_label, turn_start, turn_end = turn_segments[0]

    total_points = unique_waypoint_points(route_trace)
    approach_points = segment_points(route_trace, 0, max(turn_start, 0))
    turn_points = segment_points(route_trace, turn_start, turn_end)
    exit_points = segment_points(route_trace, turn_end, len(route_trace) - 1)
    if len(turn_points) < 2:
        return None

    key_tuple = movement_key_for_segment(movement_label, route_trace, turn_start, turn_end)
    movement_key = "|".join(str(item) for item in key_tuple)
    junction_waypoint = junction_waypoint_for_segment(route_trace, turn_start, turn_end)
    if junction_waypoint is not None:
        junction = junction_waypoint.get_junction()
        center_x = round(junction.bounding_box.location.x, 2)
        center_y = round(junction.bounding_box.location.y, 2)
        junction_id = junction.id
    else:
        center_x = round(sum(point[0] for point in turn_points) / len(turn_points), 2)
        center_y = round(sum(point[1] for point in turn_points) / len(turn_points), 2)
        junction_id = None

    entry_waypoint = route_trace[turn_start][0]
    exit_waypoint = route_trace[turn_end][0]
    option_counts = Counter(commands)
    return CandidateRoute(
        movement_key=movement_key,
        movement_label=movement_label,
        junction_id=junction_id,
        start_spawn_index=start_index,
        end_spawn_index=end_index,
        total_length_m=round(route_length(total_points), 2),
        euclidean_distance_m=round(start_transform.location.distance(end_transform.location), 2),
        approach_length_m=round(route_length(approach_points), 2),
        turn_length_m=round(route_length(turn_points), 2),
        exit_length_m=round(route_length(exit_points), 2),
        total_waypoint_count=len(route_trace),
        turn_waypoint_count=turn_end - turn_start + 1,
        center_x=center_x,
        center_y=center_y,
        entry_road_id=entry_waypoint.road_id,
        entry_lane_id=entry_waypoint.lane_id,
        exit_road_id=exit_waypoint.road_id,
        exit_lane_id=exit_waypoint.lane_id,
        road_option_counts=dict(sorted(option_counts.items())),
    )


def plot_topology(ax: plt.Axes, world_map: carla.Map) -> None:
    for entry_waypoint, exit_waypoint in world_map.get_topology():
        xs = [entry_waypoint.transform.location.x, exit_waypoint.transform.location.x]
        ys = [entry_waypoint.transform.location.y, exit_waypoint.transform.location.y]
        ax.plot(xs, ys, color="#d0d4db", linewidth=0.7, alpha=0.6, zorder=1)


def choose_representatives(
    candidates: list[CandidateRoute],
    *,
    min_approach_m: float,
    min_exit_m: float,
) -> tuple[CandidateRoute | None, CandidateRoute | None]:
    eligible = [
        candidate
        for candidate in sorted(
            candidates,
            key=lambda item: (
                -min(item.approach_length_m, item.exit_length_m),
                item.total_length_m,
                item.start_spawn_index,
                item.end_spawn_index,
            ),
        )
        if candidate.approach_length_m >= min_approach_m and candidate.exit_length_m >= min_exit_m
    ]
    if not eligible:
        return None, None
    train_candidate = eligible[0]
    eval_candidate = None
    for candidate in eligible[1:]:
        if (
            candidate.start_spawn_index != train_candidate.start_spawn_index
            or candidate.end_spawn_index != train_candidate.end_spawn_index
        ):
            eval_candidate = candidate
            break
    return train_candidate, eval_candidate


def route_config_dict(name: str, candidate: CandidateRoute, description: str) -> dict[str, Any]:
    return {
        "name": name,
        "town": "Town01",
        "closed_loop": False,
        "sampling_resolution_m": 2.0,
        "anchor_spawn_indices": [candidate.start_spawn_index, candidate.end_spawn_index],
        "description": description,
    }


def write_route_configs(
    movement_records: list[dict[str, Any]],
    *,
    train_config_dir: Path,
    eval_config_dir: Path,
) -> None:
    train_config_dir.mkdir(parents=True, exist_ok=True)
    eval_config_dir.mkdir(parents=True, exist_ok=True)
    for config_dir in (train_config_dir, eval_config_dir):
        for path in config_dir.glob("town01_movement_*.json"):
            path.unlink()

    for index, record in enumerate(movement_records, start=1):
        movement_label = str(record["movement_label"]).lower()
        key_suffix = f"{index:02d}_{movement_label}"
        if record["junction_id"] is not None:
            key_suffix += f"_j{record['junction_id']}"
        train_candidate = record.get("recommended_train")
        eval_candidate = record.get("recommended_eval")
        if train_candidate:
            train_name = f"town01_movement_{key_suffix}_train"
            train_description = (
                f"Town01 {movement_label} movement train route for junction {record['junction_id']} "
                f"({train_candidate['start_spawn_index']} -> {train_candidate['end_spawn_index']})."
            )
            with (train_config_dir / f"{train_name}.json").open("w", encoding="utf-8") as handle:
                json.dump(route_config_dict(train_name, CandidateRoute(**train_candidate), train_description), handle, indent=2)
                handle.write("\n")
        if eval_candidate:
            eval_name = f"town01_movement_{key_suffix}_eval"
            eval_description = (
                f"Town01 {movement_label} movement eval route for junction {record['junction_id']} "
                f"({eval_candidate['start_spawn_index']} -> {eval_candidate['end_spawn_index']})."
            )
            with (eval_config_dir / f"{eval_name}.json").open("w", encoding="utf-8") as handle:
                json.dump(route_config_dict(eval_name, CandidateRoute(**eval_candidate), eval_description), handle, indent=2)
                handle.write("\n")


def main() -> None:
    args = build_parser().parse_args()
    inventory_output = Path(args.inventory_output).resolve()
    plot_output = Path(args.plot_output).resolve()
    train_config_dir = Path(args.train_config_dir).resolve()
    eval_config_dir = Path(args.eval_config_dir).resolve()

    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)
    world = client.get_world()
    active_town = world.get_map().name.split("/")[-1]
    if active_town != args.town:
        world = client.load_world(args.town)
    world_map = world.get_map()
    spawn_points = world_map.get_spawn_points()
    planner = GlobalRoutePlanner(world_map, args.sampling_resolution_m)

    candidate_groups: dict[str, list[CandidateRoute]] = defaultdict(list)
    total_pairs = 0
    traced_pairs = 0

    for start_index, start_transform in enumerate(spawn_points):
        for end_index, end_transform in enumerate(spawn_points):
            if start_index == end_index:
                continue
            total_pairs += 1
            euclidean_distance = start_transform.location.distance(end_transform.location)
            if euclidean_distance < args.min_euclidean_m or euclidean_distance > args.max_euclidean_m:
                continue
            route_trace = planner.trace_route(start_transform.location, end_transform.location)
            traced_pairs += 1
            if not route_trace:
                continue
            candidate = candidate_route_from_trace(
                start_index,
                end_index,
                start_transform,
                end_transform,
                route_trace,
            )
            if candidate is None:
                continue
            candidate_groups[candidate.movement_key].append(candidate)

    movement_records: list[dict[str, Any]] = []
    label_counts: Counter[str] = Counter()
    eval_ready_counts: Counter[str] = Counter()

    for movement_key, candidates in sorted(
        candidate_groups.items(),
        key=lambda item: (
            item[1][0].movement_label,
            item[1][0].junction_id if item[1][0].junction_id is not None else 10**9,
            item[1][0].center_x,
            item[1][0].center_y,
        ),
    ):
        movement_label = candidates[0].movement_label
        label_counts[movement_label] += 1
        train_candidate, eval_candidate = choose_representatives(
            candidates,
            min_approach_m=args.min_approach_m,
            min_exit_m=args.min_exit_m,
        )
        if eval_candidate is not None:
            eval_ready_counts[movement_label] += 1
        movement_records.append(
            {
                "movement_key": movement_key,
                "movement_label": movement_label,
                "junction_id": candidates[0].junction_id,
                "center_x": candidates[0].center_x,
                "center_y": candidates[0].center_y,
                "entry_road_id": candidates[0].entry_road_id,
                "entry_lane_id": candidates[0].entry_lane_id,
                "exit_road_id": candidates[0].exit_road_id,
                "exit_lane_id": candidates[0].exit_lane_id,
                "candidate_count": len(candidates),
                "recommended_train": asdict(train_candidate) if train_candidate is not None else None,
                "recommended_eval": asdict(eval_candidate) if eval_candidate is not None else None,
                "candidates": [
                    asdict(candidate)
                    for candidate in sorted(candidates, key=lambda item: (item.total_length_m, item.start_spawn_index, item.end_spawn_index))
                ],
            }
        )

    write_route_configs(movement_records, train_config_dir=train_config_dir, eval_config_dir=eval_config_dir)

    inventory_output.parent.mkdir(parents=True, exist_ok=True)
    plot_output.parent.mkdir(parents=True, exist_ok=True)
    summary = {
        "town": args.town,
        "sampling_resolution_m": args.sampling_resolution_m,
        "spawn_point_count": len(spawn_points),
        "total_ordered_spawn_pairs": total_pairs,
        "traced_pair_count": traced_pairs,
        "min_euclidean_m": args.min_euclidean_m,
        "max_euclidean_m": args.max_euclidean_m,
        "min_approach_m": args.min_approach_m,
        "min_exit_m": args.min_exit_m,
        "movement_count": len(movement_records),
        "movement_counts_by_label": dict(sorted(label_counts.items())),
        "eval_ready_counts_by_label": dict(sorted(eval_ready_counts.items())),
        "train_config_dir": str(train_config_dir.relative_to(PROJECT_ROOT)),
        "eval_config_dir": str(eval_config_dir.relative_to(PROJECT_ROOT)),
        "movements": movement_records,
    }
    with inventory_output.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
        handle.write("\n")

    fig, ax = plt.subplots(figsize=(10, 8))
    plot_topology(ax, world_map)
    color_map = {"LEFT": "#d35454", "RIGHT": "#2c7fb8", "STRAIGHT": "#27ae60"}
    for record in movement_records:
        ax.scatter(
            [record["center_x"]],
            [record["center_y"]],
            color=color_map.get(record["movement_label"], "#7f8c8d"),
            s=38,
            alpha=0.9,
            zorder=3,
        )
    ax.set_title("Town01 movement inventory")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.invert_yaxis()
    ax.grid(color="#ecf0f1", linewidth=0.5, alpha=0.6)
    handles = [
        plt.Line2D([0], [0], marker="o", color="w", markerfacecolor=color, markersize=8, label=label.title())
        for label, color in color_map.items()
    ]
    ax.legend(handles=handles, loc="upper right")
    fig.tight_layout()
    fig.savefig(plot_output, dpi=180)
    plt.close(fig)

    stdout_summary = {
        "town": summary["town"],
        "movement_count": summary["movement_count"],
        "movement_counts_by_label": summary["movement_counts_by_label"],
        "eval_ready_counts_by_label": summary["eval_ready_counts_by_label"],
        "train_route_config_count": sum(1 for movement in movement_records if movement["recommended_train"] is not None),
        "eval_route_config_count": sum(1 for movement in movement_records if movement["recommended_eval"] is not None),
        "inventory_output": str(inventory_output.relative_to(PROJECT_ROOT)),
        "plot_output": str(plot_output.relative_to(PROJECT_ROOT)),
        "train_config_dir": summary["train_config_dir"],
        "eval_config_dir": summary["eval_config_dir"],
    }
    print(json.dumps(stdout_summary, indent=2))


if __name__ == "__main__":
    main()
