from __future__ import annotations

from dataclasses import dataclass
from math import inf


@dataclass(frozen=True, slots=True)
class TrafficLightApproach:
    actor_id: int
    heading_deg: float


@dataclass(frozen=True, slots=True)
class TrafficLightPhaseCycle:
    green_seconds: float
    yellow_seconds: float
    all_red_seconds: float
    initial_offset_seconds: float = 0.0


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

    ordered = sorted(
        approaches,
        key=lambda approach: (approach.actor_id, _normalize_heading_deg(approach.heading_deg)),
    )
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


def compute_phase_states(
    phase_actor_ids: list[list[int]],
    *,
    elapsed_seconds: float,
    cycle: TrafficLightPhaseCycle,
) -> dict[int, str]:
    if not phase_actor_ids:
        return {}
    if cycle.green_seconds < 0.0 or cycle.yellow_seconds < 0.0 or cycle.all_red_seconds < 0.0:
        raise ValueError("TrafficLightPhaseCycle durations must be non-negative.")

    phase_window_seconds = cycle.green_seconds + cycle.yellow_seconds
    full_cycle_seconds = (len(phase_actor_ids) * phase_window_seconds) + cycle.all_red_seconds
    if full_cycle_seconds <= 0.0:
        raise ValueError("TrafficLightPhaseCycle must have positive total duration.")

    cycle_elapsed_seconds = (elapsed_seconds + cycle.initial_offset_seconds) % full_cycle_seconds
    all_actor_ids = [actor_id for actor_ids in phase_actor_ids for actor_id in actor_ids]

    if cycle_elapsed_seconds >= len(phase_actor_ids) * phase_window_seconds:
        return {actor_id: "red" for actor_id in all_actor_ids}

    active_phase_index = (
        int(cycle_elapsed_seconds // phase_window_seconds) if phase_window_seconds > 0.0 else 0
    )
    active_phase_elapsed = cycle_elapsed_seconds - (active_phase_index * phase_window_seconds)
    active_state = "green" if active_phase_elapsed < cycle.green_seconds else "yellow"

    states = {actor_id: "red" for actor_id in all_actor_ids}
    for actor_id in phase_actor_ids[active_phase_index]:
        states[actor_id] = active_state
    return states
