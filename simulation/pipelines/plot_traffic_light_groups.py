from __future__ import annotations

import argparse
import json
from collections import defaultdict
from dataclasses import asdict, dataclass
from itertools import pairwise
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
from matplotlib import pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D

from libs.carla_utils import ensure_carla_agents_on_path, require_carla
from libs.carla_utils.traffic_light_phasing import TrafficLightApproach, build_opposing_phase_groups
from libs.project import PROJECT_ROOT, relative_to_project

DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "outputs" / "inspect" / "traffic_light_groups"
PHASE_COLORS = [
    "#2563eb",
    "#ea580c",
    "#16a34a",
    "#a21caf",
]


@dataclass(frozen=True, slots=True)
class TrafficLightActorSummary:
    actor_id: int
    raw_group_index: int
    phase_index: int
    x_m: float
    y_m: float
    heading_deg: float
    lane_id: str | None
    road_id: int | None


@dataclass(frozen=True, slots=True)
class TrafficLightGroupSummary:
    raw_group_index: int
    actor_ids: list[int]
    phase_actor_ids: list[list[int]]
    centroid_x_m: float
    centroid_y_m: float


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Plot CARLA traffic light raw groups and derived opposing-direction phases on a town map."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--town", default="Town01")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--lane-sampling-m", type=float, default=5.0)
    parser.add_argument("--overview-dpi", type=int, default=180)
    parser.add_argument("--group-dpi", type=int, default=180)
    parser.add_argument("--group-padding-m", type=float, default=24.0)
    parser.add_argument("--axis-merge-tolerance-deg", type=float, default=20.0)
    return parser


def _load_world_for_town(client: Any, town: str) -> Any:
    world = client.get_world()
    active_town = world.get_map().name.split("/")[-1]
    if active_town != town:
        world = client.load_world(town)
    return world


def _lane_centerlines_all_map(
    world_map: Any, *, lane_sampling_m: float
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


def _segments_from_lines(lines: list[list[tuple[float, float]]]) -> list[list[tuple[float, float]]]:
    segments: list[list[tuple[float, float]]] = []
    for line in lines:
        segments.extend([[start, end] for start, end in pairwise(line)])
    return segments


def _plot_background(ax: Any, lane_centerlines: list[list[tuple[float, float]]]) -> None:
    lane_segments = _segments_from_lines(lane_centerlines)
    if lane_segments:
        ax.add_collection(
            LineCollection(
                lane_segments,
                colors="#e7e5e4",
                linewidths=8.0,
                alpha=0.95,
                zorder=0,
            )
        )
        ax.add_collection(
            LineCollection(
                lane_segments,
                colors="#cbd5e1",
                linewidths=0.75,
                alpha=0.9,
                zorder=1,
            )
        )


def _plot_actor(ax: Any, actor: TrafficLightActorSummary, *, label: bool = True) -> None:
    color = PHASE_COLORS[actor.phase_index % len(PHASE_COLORS)]
    ax.scatter(
        [actor.x_m],
        [actor.y_m],
        s=42,
        c=color,
        edgecolors="white",
        linewidths=0.8,
        zorder=4,
    )
    heading_rad = actor.heading_deg * 3.141592653589793 / 180.0
    dx = 5.0 * __import__("math").cos(heading_rad)
    dy = 5.0 * __import__("math").sin(heading_rad)
    ax.annotate(
        "",
        xy=(actor.x_m + dx, actor.y_m + dy),
        xytext=(actor.x_m, actor.y_m),
        arrowprops={"arrowstyle": "->", "color": color, "lw": 1.2},
        zorder=5,
    )
    if label:
        ax.text(
            actor.x_m + 1.5,
            actor.y_m + 1.5,
            f"{actor.actor_id}\nG{actor.raw_group_index:02d} P{actor.phase_index + 1}",
            fontsize=7,
            color="#111827",
            zorder=6,
            bbox={
                "facecolor": "#ffffffdd",
                "edgecolor": "#d6d3d1",
                "linewidth": 0.5,
                "boxstyle": "round,pad=0.2",
            },
        )


def _collect_group_summaries(
    world: Any,
    *,
    axis_merge_tolerance_deg: float,
) -> tuple[list[TrafficLightActorSummary], list[TrafficLightGroupSummary]]:
    ensure_carla_agents_on_path()
    from agents.tools.misc import get_trafficlight_trigger_location

    world_map = world.get_map()
    lights_by_id = {int(actor.id): actor for actor in world.get_actors().filter("*traffic_light*")}
    raw_group_keys: list[tuple[int, ...]] = []
    visited: set[tuple[int, ...]] = set()
    for traffic_light in lights_by_id.values():
        group = list(traffic_light.get_group_traffic_lights()) or [traffic_light]
        group_key = tuple(sorted(int(actor.id) for actor in group))
        if group_key in visited:
            continue
        visited.add(group_key)
        raw_group_keys.append(group_key)

    actor_summaries: list[TrafficLightActorSummary] = []
    group_summaries: list[TrafficLightGroupSummary] = []
    for raw_group_index, group_key in enumerate(sorted(raw_group_keys), start=1):
        approaches: list[TrafficLightApproach] = []
        actor_payloads: list[tuple[int, float, float, float, str | None, int | None]] = []
        for actor_id in group_key:
            traffic_light = lights_by_id[actor_id]
            trigger_location = get_trafficlight_trigger_location(traffic_light)
            trigger_waypoint = world_map.get_waypoint(trigger_location)
            heading_deg = float(trigger_waypoint.transform.rotation.yaw)
            lane_id = (
                f"{int(trigger_waypoint.road_id)}:{int(trigger_waypoint.lane_id)}"
                if trigger_waypoint is not None
                else None
            )
            road_id = int(trigger_waypoint.road_id) if trigger_waypoint is not None else None
            approaches.append(TrafficLightApproach(actor_id=actor_id, heading_deg=heading_deg))
            actor_payloads.append(
                (
                    actor_id,
                    float(trigger_location.x),
                    float(trigger_location.y),
                    heading_deg,
                    lane_id,
                    road_id,
                )
            )

        phase_actor_ids = build_opposing_phase_groups(
            approaches,
            axis_merge_tolerance_deg=axis_merge_tolerance_deg,
        )
        phase_index_by_actor_id = {
            actor_id: phase_index
            for phase_index, actor_ids in enumerate(phase_actor_ids)
            for actor_id in actor_ids
        }

        centroid_x_m = sum(payload[1] for payload in actor_payloads) / max(1, len(actor_payloads))
        centroid_y_m = sum(payload[2] for payload in actor_payloads) / max(1, len(actor_payloads))
        group_summaries.append(
            TrafficLightGroupSummary(
                raw_group_index=raw_group_index,
                actor_ids=list(group_key),
                phase_actor_ids=phase_actor_ids,
                centroid_x_m=centroid_x_m,
                centroid_y_m=centroid_y_m,
            )
        )

        for actor_id, x_m, y_m, heading_deg, lane_id, road_id in actor_payloads:
            actor_summaries.append(
                TrafficLightActorSummary(
                    actor_id=actor_id,
                    raw_group_index=raw_group_index,
                    phase_index=phase_index_by_actor_id[actor_id],
                    x_m=x_m,
                    y_m=y_m,
                    heading_deg=heading_deg,
                    lane_id=lane_id,
                    road_id=road_id,
                )
            )
    return actor_summaries, group_summaries


def _plot_overview(
    *,
    town: str,
    lane_centerlines: list[list[tuple[float, float]]],
    actors: list[TrafficLightActorSummary],
    groups: list[TrafficLightGroupSummary],
    output_path: Path,
    dpi: int,
) -> None:
    figure, ax = plt.subplots(figsize=(12, 12), dpi=dpi)
    figure.patch.set_facecolor("#f8fafc")
    ax.set_facecolor("#ffffff")
    _plot_background(ax, lane_centerlines)

    for group in groups:
        for actor in [item for item in actors if item.raw_group_index == group.raw_group_index]:
            ax.plot(
                [group.centroid_x_m, actor.x_m],
                [group.centroid_y_m, actor.y_m],
                color="#94a3b8",
                linewidth=0.8,
                linestyle="--",
                zorder=2,
            )
        ax.text(
            group.centroid_x_m,
            group.centroid_y_m,
            f"G{group.raw_group_index:02d}",
            fontsize=9,
            fontweight="bold",
            color="#0f172a",
            ha="center",
            va="center",
            zorder=3,
            bbox={
                "facecolor": "#ffffffee",
                "edgecolor": "#94a3b8",
                "linewidth": 0.6,
                "boxstyle": "round,pad=0.2",
            },
        )

    for actor in actors:
        _plot_actor(ax, actor)

    all_points = [point for line in lane_centerlines for point in line]
    if all_points:
        xs = [point[0] for point in all_points]
        ys = [point[1] for point in all_points]
        ax.set_xlim(min(xs) - 15.0, max(xs) + 15.0)
        ax.set_ylim(min(ys) - 15.0, max(ys) + 15.0)

    legend_handles = [
        Line2D(
            [0],
            [0],
            marker="o",
            color="w",
            markerfacecolor=PHASE_COLORS[index],
            markersize=8,
            label=f"Phase {index + 1}",
        )
        for index in range(min(4, max((actor.phase_index for actor in actors), default=-1) + 1))
    ]
    if legend_handles:
        ax.legend(handles=legend_handles, loc="upper right", frameon=True, fontsize=8)

    ax.set_aspect("equal", adjustable="box")
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)

    ax.set_title(f"{town} Traffic Light Groups", fontsize=15, color="#0f172a", pad=16)
    figure.text(
        0.5,
        0.955,
        "Marker label: actor_id / raw group / derived phase. Dashed lines connect actors belonging to the same CARLA raw group.",
        ha="center",
        va="top",
        fontsize=9,
        color="#475569",
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)


def _plot_group_detail(
    *,
    town: str,
    lane_centerlines: list[list[tuple[float, float]]],
    actors: list[TrafficLightActorSummary],
    group: TrafficLightGroupSummary,
    output_path: Path,
    dpi: int,
    padding_m: float,
) -> None:
    figure, ax = plt.subplots(figsize=(7, 7), dpi=dpi)
    figure.patch.set_facecolor("#f8fafc")
    ax.set_facecolor("#ffffff")
    _plot_background(ax, lane_centerlines)

    group_actors = [actor for actor in actors if actor.raw_group_index == group.raw_group_index]
    for actor in group_actors:
        ax.plot(
            [group.centroid_x_m, actor.x_m],
            [group.centroid_y_m, actor.y_m],
            color="#94a3b8",
            linewidth=0.8,
            linestyle="--",
            zorder=2,
        )
        _plot_actor(ax, actor)

    xs = [actor.x_m for actor in group_actors]
    ys = [actor.y_m for actor in group_actors]
    ax.set_xlim(min(xs) - padding_m, max(xs) + padding_m)
    ax.set_ylim(min(ys) - padding_m, max(ys) + padding_m)

    phase_text = " | ".join(
        f"P{phase_index + 1}: {actor_ids}"
        for phase_index, actor_ids in enumerate(group.phase_actor_ids)
    )
    ax.set_title(
        f"{town} Raw Group G{group.raw_group_index:02d}",
        fontsize=13,
        color="#0f172a",
        pad=14,
    )
    figure.text(
        0.5,
        0.955,
        f"Actors={group.actor_ids}  Derived phases={phase_text}",
        ha="center",
        va="top",
        fontsize=8,
        color="#475569",
    )

    ax.set_aspect("equal", adjustable="box")
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)


def main() -> None:
    args = build_parser().parse_args()

    carla = require_carla()
    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = _load_world_for_town(client, args.town)
    world_map = world.get_map()

    lane_centerlines = _lane_centerlines_all_map(world_map, lane_sampling_m=args.lane_sampling_m)
    actors, groups = _collect_group_summaries(
        world,
        axis_merge_tolerance_deg=args.axis_merge_tolerance_deg,
    )

    output_dir = Path(args.output_dir).resolve() / args.town.lower()
    overview_path = output_dir / f"{args.town.lower()}_overview.png"
    _plot_overview(
        town=args.town,
        lane_centerlines=lane_centerlines,
        actors=actors,
        groups=groups,
        output_path=overview_path,
        dpi=args.overview_dpi,
    )

    for group in groups:
        detail_path = output_dir / f"{args.town.lower()}_group_{group.raw_group_index:02d}.png"
        _plot_group_detail(
            town=args.town,
            lane_centerlines=lane_centerlines,
            actors=actors,
            group=group,
            output_path=detail_path,
            dpi=args.group_dpi,
            padding_m=args.group_padding_m,
        )

    summary_path = output_dir / f"{args.town.lower()}_groups.json"
    summary = {
        "town": args.town,
        "group_count": len(groups),
        "actor_count": len(actors),
        "groups": [asdict(group) for group in groups],
        "actors": [asdict(actor) for actor in actors],
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"overview: {relative_to_project(overview_path)}")
    print(f"summary:  {relative_to_project(summary_path)}")
    for group in groups:
        detail_path = output_dir / f"{args.town.lower()}_group_{group.raw_group_index:02d}.png"
        print(
            f"group {group.raw_group_index:02d}: actors={group.actor_ids} phases={group.phase_actor_ids} "
            f"-> {relative_to_project(detail_path)}"
        )


if __name__ == "__main__":
    main()
