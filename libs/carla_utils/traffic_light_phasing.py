from __future__ import annotations

from dataclasses import dataclass
from math import inf


@dataclass(frozen=True, slots=True)
class TrafficLightApproach:
    actor_id: int
    heading_deg: float


def _normalize_heading_deg(heading_deg: float) -> float:
    wrapped = heading_deg % 360.0
    if wrapped < 0.0:
        wrapped += 360.0
    return wrapped


def _heading_axis_deg(heading_deg: float) -> float:
    normalized = _normalize_heading_deg(heading_deg)
    return normalized if normalized < 180.0 else normalized - 180.0


def build_opposing_phase_groups(
    approaches: list[TrafficLightApproach],
    *,
    axis_merge_tolerance_deg: float = 20.0,
) -> list[list[int]]:
    if axis_merge_tolerance_deg < 0.0:
        raise ValueError("axis_merge_tolerance_deg must be non-negative.")

    ordered = sorted(approaches, key=lambda approach: (approach.actor_id, _normalize_heading_deg(approach.heading_deg)))
    axis_clusters: list[tuple[list[float], list[int]]] = []
    for approach in ordered:
        axis_deg = _heading_axis_deg(approach.heading_deg)
        best_index = -1
        best_distance = inf
        for cluster_index, (cluster_axes_deg, _actor_ids) in enumerate(axis_clusters):
            axis_distance = min(
                min(
                    abs(axis_deg - cluster_axis_deg),
                    180.0 - abs(axis_deg - cluster_axis_deg),
                )
                for cluster_axis_deg in cluster_axes_deg
            )
            if axis_distance <= axis_merge_tolerance_deg and axis_distance < best_distance:
                best_distance = axis_distance
                best_index = cluster_index
        if best_index >= 0:
            cluster_axes_deg, actor_ids = axis_clusters[best_index]
            cluster_axes_deg.append(axis_deg)
            actor_ids.append(approach.actor_id)
        else:
            axis_clusters.append(([axis_deg], [approach.actor_id]))

    axis_clusters.sort(key=lambda cluster: min(cluster[0]))
    return [sorted(actor_ids) for _axes_deg, actor_ids in axis_clusters]
