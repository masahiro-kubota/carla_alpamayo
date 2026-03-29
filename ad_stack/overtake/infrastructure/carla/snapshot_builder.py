from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from ad_stack.overtake.domain import (
    AdjacentLaneGapSnapshot,
    OvertakeContext,
    OvertakeLeadSnapshot,
    OvertakeTargetSnapshot,
)
from ad_stack.overtake.infrastructure.carla.route_alignment import lane_id
from ad_stack.overtake.policies import build_stopped_obstacle_targets


@dataclass(slots=True)
class OvertakeSceneSnapshot:
    lead_vehicle: Any | None
    active_target: OvertakeTargetSnapshot | None
    decision_context: OvertakeContext
    lead_distance_m: float | None
    lead_speed_mps: float
    same_lane_lead_distance_m: float | None
    same_lane_lead_speed_mps: float
    closing_speed_mps: float
    min_ttc: float
    same_lane_min_ttc: float
    follow_target_speed_kmh: float
    left_front_gap_m: float
    left_rear_gap_m: float
    right_front_gap_m: float
    right_rear_gap_m: float


def nearest_lead(
    tracked_objects: tuple[Any, ...],
    *,
    relation: str = "same_lane",
) -> Any | None:
    candidates = [
        actor
        for actor in tracked_objects
        if actor.relation == relation
        and actor.is_ahead
        and actor.longitudinal_distance_m is not None
        and actor.longitudinal_distance_m > 0.0
    ]
    if not candidates:
        return None
    return min(candidates, key=lambda actor: float(actor.longitudinal_distance_m))


def lane_gaps(
    tracked_objects: tuple[Any, ...],
    relation: str,
) -> tuple[float, float]:
    front_gap_m = float("inf")
    rear_gap_m = float("inf")
    for actor in tracked_objects:
        if actor.relation != relation or actor.longitudinal_distance_m is None:
            continue
        longitudinal_distance = float(actor.longitudinal_distance_m)
        if actor.is_ahead and longitudinal_distance >= 0.0:
            front_gap_m = min(front_gap_m, longitudinal_distance)
        elif longitudinal_distance < 0.0:
            rear_gap_m = min(rear_gap_m, abs(longitudinal_distance))
    return front_gap_m, rear_gap_m


def lane_gaps_for_lane_id(
    tracked_objects: tuple[Any, ...],
    lane_id_value: str | None,
) -> tuple[float, float]:
    if lane_id_value is None:
        return float("inf"), float("inf")
    front_gap_m = float("inf")
    rear_gap_m = float("inf")
    for actor in tracked_objects:
        if actor.lane_id != lane_id_value or actor.longitudinal_distance_m is None:
            continue
        longitudinal_distance = float(actor.longitudinal_distance_m)
        if longitudinal_distance >= 0.0:
            front_gap_m = min(front_gap_m, longitudinal_distance)
        else:
            rear_gap_m = min(rear_gap_m, abs(longitudinal_distance))
    return front_gap_m, rear_gap_m


def visible_overtake_target_actors(
    tracked_objects: tuple[Any, ...],
    *,
    target_actor_id: int | None,
    target_member_actor_ids: tuple[int, ...],
) -> tuple[Any | None, tuple[Any, ...]]:
    if target_actor_id is None:
        return None, ()
    member_ids = set(target_member_actor_ids) or {target_actor_id}
    visible_members = tuple(actor for actor in tracked_objects if actor.actor_id in member_ids)
    primary_actor = next(
        (actor for actor in visible_members if actor.actor_id == target_actor_id),
        None,
    )
    return primary_actor, visible_members


def route_relative_progress_to_actor(
    *,
    actor: Any,
    reference_route_index: int,
    base_trace: list[tuple[Any, Any]],
    route_point_to_trace_index: list[int],
    route_point_progress_m: list[float],
    search_back_points: int = 24,
    search_forward_points: int = 120,
) -> tuple[float | None, float, str | None]:
    if not route_point_to_trace_index:
        return None, float("inf"), None
    bounded_reference_index = max(0, min(reference_route_index, len(route_point_to_trace_index) - 1))
    start_index = max(0, bounded_reference_index - search_back_points)
    end_index = min(len(route_point_to_trace_index), bounded_reference_index + search_forward_points + 1)
    best_route_index: int | None = None
    best_distance_m = float("inf")
    for route_point_index in range(start_index, end_index):
        trace_index = route_point_to_trace_index[route_point_index]
        route_location = base_trace[trace_index][0].transform.location
        distance_m = math.hypot(route_location.x - actor.x_m, route_location.y - actor.y_m)
        if distance_m < best_distance_m:
            best_distance_m = distance_m
            best_route_index = route_point_index
    if best_route_index is None:
        return None, best_distance_m, None
    trace_index = route_point_to_trace_index[best_route_index]
    route_lane_id = lane_id(base_trace[trace_index][0])
    relative_progress_m = (
        route_point_progress_m[best_route_index] - route_point_progress_m[bounded_reference_index]
    )
    return float(relative_progress_m), best_distance_m, route_lane_id


def build_same_lane_stopped_targets(
    tracked_objects: tuple[Any, ...],
    *,
    stopped_speed_threshold_mps: float,
    cluster_merge_gap_m: float,
    cluster_max_member_speed_mps: float,
) -> list[OvertakeTargetSnapshot]:
    leads = [
        OvertakeLeadSnapshot(
            actor_id=actor.actor_id,
            lane_id=actor.lane_id,
            distance_m=float(actor.longitudinal_distance_m),
            speed_mps=float(actor.speed_mps),
            relative_speed_mps=0.0,
            is_stopped=float(actor.speed_mps) <= stopped_speed_threshold_mps,
        )
        for actor in tracked_objects
        if (
            actor.relation == "same_lane"
            and actor.is_ahead
            and actor.longitudinal_distance_m is not None
            and float(actor.longitudinal_distance_m) > 0.0
        )
    ]
    return build_stopped_obstacle_targets(
        leads,
        cluster_merge_gap_m=cluster_merge_gap_m,
        cluster_max_member_speed_mps=cluster_max_member_speed_mps,
    )


def build_route_aligned_stopped_targets(
    tracked_objects: tuple[Any, ...],
    *,
    route_index: int | None,
    base_trace: list[tuple[Any, Any]],
    route_point_to_trace_index: list[int],
    route_point_progress_m: list[float],
    stopped_speed_threshold_mps: float,
    cluster_merge_gap_m: float,
    cluster_max_member_speed_mps: float,
) -> list[OvertakeTargetSnapshot]:
    if route_index is None or not base_trace or not route_point_to_trace_index or not route_point_progress_m:
        return []

    bounded_route_index = max(0, min(route_index, len(route_point_to_trace_index) - 1))
    leads: list[OvertakeLeadSnapshot] = []
    for actor in tracked_objects:
        if actor.actor_id is None or actor.lane_id is None:
            continue
        if float(actor.speed_mps) > stopped_speed_threshold_mps:
            continue
        route_distance_m, route_distance_to_centerline_m, route_lane_id = route_relative_progress_to_actor(
            actor=actor,
            reference_route_index=bounded_route_index,
            base_trace=base_trace,
            route_point_to_trace_index=route_point_to_trace_index,
            route_point_progress_m=route_point_progress_m,
            search_back_points=0,
        )
        if route_distance_m is None or route_distance_to_centerline_m > 4.0:
            continue
        if route_lane_id != actor.lane_id:
            continue
        if route_distance_m <= 0.0:
            continue
        leads.append(
            OvertakeLeadSnapshot(
                actor_id=actor.actor_id,
                lane_id=actor.lane_id,
                distance_m=float(route_distance_m),
                speed_mps=float(actor.speed_mps),
                relative_speed_mps=0.0,
                is_stopped=True,
            )
        )
    return build_stopped_obstacle_targets(
        leads,
        cluster_merge_gap_m=cluster_merge_gap_m,
        cluster_max_member_speed_mps=cluster_max_member_speed_mps,
    )


def enrich_targets_with_adjacent_lane_availability(
    targets: list[OvertakeTargetSnapshot],
    *,
    tracked_objects: tuple[Any, ...],
    world_map: Any,
    carla_module: Any,
) -> list[OvertakeTargetSnapshot]:
    enriched_targets: list[OvertakeTargetSnapshot] = []
    for target in targets:
        matching_actor = next(
            (actor for actor in tracked_objects if actor.actor_id == target.primary_actor_id),
            None,
        )
        adjacent_lane_available = True
        if matching_actor is not None:
            target_waypoint = world_map.get_waypoint(
                carla_module.Location(x=matching_actor.x_m, y=matching_actor.y_m, z=0.0),
                lane_type=carla_module.LaneType.Driving,
            )
            target_left_waypoint = target_waypoint.get_left_lane() if target_waypoint is not None else None
            target_right_waypoint = target_waypoint.get_right_lane() if target_waypoint is not None else None
            adjacent_lane_available = bool(
                (
                    target_left_waypoint is not None
                    and target_left_waypoint.lane_type == carla_module.LaneType.Driving
                )
                or (
                    target_right_waypoint is not None
                    and target_right_waypoint.lane_type == carla_module.LaneType.Driving
                )
            )
        enriched_targets.append(
            OvertakeTargetSnapshot(
                kind=target.kind,
                primary_actor_id=target.primary_actor_id,
                member_actor_ids=target.member_actor_ids,
                lane_id=target.lane_id,
                entry_distance_m=target.entry_distance_m,
                exit_distance_m=target.exit_distance_m,
                speed_mps=target.speed_mps,
                is_stopped=target.is_stopped,
                adjacent_lane_available=adjacent_lane_available,
            )
        )
    return enriched_targets


def build_overtake_scene_snapshot(
    *,
    tracked_objects: tuple[Any, ...],
    timestamp_s: float,
    current_speed_mps: float,
    current_lane_id: str | None,
    route_target_lane_id: str | None,
    route_index: int | None,
    ego_waypoint: Any,
    adjacent_lanes_open: dict[str, bool],
    target_speed_kmh: float,
    follow_headway_seconds: float,
    stopped_speed_threshold_mps: float,
    cluster_merge_gap_m: float,
    cluster_max_member_speed_mps: float,
    active_signal_state: str | None,
    signal_stop_distance_m: float | None,
    allow_overtake: bool,
    preferred_direction: str,
    world_map: Any,
    carla_module: Any,
    base_trace: list[tuple[Any, Any]],
    route_point_to_trace_index: list[int],
    route_point_progress_m: list[float],
) -> OvertakeSceneSnapshot:
    lead_vehicle = nearest_lead(tracked_objects, relation="same_lane")
    same_lane_stopped_targets = build_same_lane_stopped_targets(
        tracked_objects,
        stopped_speed_threshold_mps=stopped_speed_threshold_mps,
        cluster_merge_gap_m=cluster_merge_gap_m,
        cluster_max_member_speed_mps=cluster_max_member_speed_mps,
    )
    route_aligned_stopped_targets = build_route_aligned_stopped_targets(
        tracked_objects,
        route_index=route_index,
        base_trace=base_trace,
        route_point_to_trace_index=route_point_to_trace_index,
        route_point_progress_m=route_point_progress_m,
        stopped_speed_threshold_mps=stopped_speed_threshold_mps,
        cluster_merge_gap_m=cluster_merge_gap_m,
        cluster_max_member_speed_mps=cluster_max_member_speed_mps,
    )
    route_aligned_stopped_targets = enrich_targets_with_adjacent_lane_availability(
        route_aligned_stopped_targets,
        tracked_objects=tracked_objects,
        world_map=world_map,
        carla_module=carla_module,
    )
    stopped_targets = (
        same_lane_stopped_targets if same_lane_stopped_targets else route_aligned_stopped_targets
    )
    active_target = stopped_targets[0] if stopped_targets else None

    same_lane_lead_distance_m = (
        float(lead_vehicle.longitudinal_distance_m)
        if lead_vehicle is not None and lead_vehicle.longitudinal_distance_m is not None
        else None
    )
    same_lane_lead_speed_mps = float(lead_vehicle.speed_mps) if lead_vehicle is not None else 0.0
    lead_distance_m = (
        same_lane_lead_distance_m
        if same_lane_lead_distance_m is not None
        else (float(active_target.entry_distance_m) if active_target is not None else None)
    )
    lead_speed_mps = (
        same_lane_lead_speed_mps
        if same_lane_lead_distance_m is not None
        else (float(active_target.speed_mps) if active_target is not None else 0.0)
    )
    closing_speed_mps = current_speed_mps - lead_speed_mps if lead_distance_m is not None else 0.0

    min_ttc = float("inf")
    if lead_distance_m is not None and closing_speed_mps > 1e-3:
        min_ttc = lead_distance_m / closing_speed_mps

    same_lane_min_ttc = float("inf")
    same_lane_closing_speed_mps = (
        current_speed_mps - same_lane_lead_speed_mps
        if same_lane_lead_distance_m is not None
        else 0.0
    )
    if same_lane_lead_distance_m is not None and same_lane_closing_speed_mps > 1e-3:
        same_lane_min_ttc = same_lane_lead_distance_m / same_lane_closing_speed_mps

    follow_target_speed_kmh = target_speed_kmh
    if lead_distance_m is not None:
        distance_limited_speed_kmh = max(
            0.0,
            (lead_distance_m / max(follow_headway_seconds, 0.25)) * 3.6,
        )
        follow_target_speed_kmh = min(
            target_speed_kmh,
            (lead_speed_mps * 3.6) + 2.0,
            distance_limited_speed_kmh,
        )

    left_front_gap_m, left_rear_gap_m = lane_gaps(tracked_objects, "left_lane")
    right_front_gap_m, right_rear_gap_m = lane_gaps(tracked_objects, "right_lane")
    left_lane_waypoint = ego_waypoint.get_left_lane() if ego_waypoint is not None else None
    right_lane_waypoint = ego_waypoint.get_right_lane() if ego_waypoint is not None else None
    left_lane_snapshot = AdjacentLaneGapSnapshot(
        lane_id=lane_id(left_lane_waypoint),
        front_gap_m=None if not math.isfinite(left_front_gap_m) else float(left_front_gap_m),
        rear_gap_m=None if not math.isfinite(left_rear_gap_m) else float(left_rear_gap_m),
        lane_open=bool(adjacent_lanes_open.get("left", False)),
    )
    right_lane_snapshot = AdjacentLaneGapSnapshot(
        lane_id=lane_id(right_lane_waypoint),
        front_gap_m=None if not math.isfinite(right_front_gap_m) else float(right_front_gap_m),
        rear_gap_m=None if not math.isfinite(right_rear_gap_m) else float(right_rear_gap_m),
        lane_open=bool(adjacent_lanes_open.get("right", False)),
    )
    decision_context = OvertakeContext(
        timestamp_s=timestamp_s,
        current_lane_id=current_lane_id,
        origin_lane_id=current_lane_id,
        route_target_lane_id=route_target_lane_id,
        target_speed_kmh=target_speed_kmh,
        stopped_speed_threshold_mps=stopped_speed_threshold_mps,
        lead=OvertakeLeadSnapshot(
            actor_id=(
                lead_vehicle.actor_id
                if lead_vehicle is not None
                else active_target.primary_actor_id
                if active_target is not None
                else None
            ),
            lane_id=(
                lead_vehicle.lane_id
                if lead_vehicle is not None
                else active_target.lane_id
                if active_target is not None
                else None
            ),
            distance_m=lead_distance_m,
            speed_mps=lead_speed_mps,
            relative_speed_mps=closing_speed_mps,
            is_stopped=lead_speed_mps <= stopped_speed_threshold_mps,
        ),
        left_lane=left_lane_snapshot,
        right_lane=right_lane_snapshot,
        active_signal_state=active_signal_state,
        signal_stop_distance_m=signal_stop_distance_m,
        allow_overtake=allow_overtake,
        preferred_direction=preferred_direction,  # type: ignore[arg-type]
        active_target=active_target,
    )
    return OvertakeSceneSnapshot(
        lead_vehicle=lead_vehicle,
        active_target=active_target,
        decision_context=decision_context,
        lead_distance_m=lead_distance_m,
        lead_speed_mps=lead_speed_mps,
        same_lane_lead_distance_m=same_lane_lead_distance_m,
        same_lane_lead_speed_mps=same_lane_lead_speed_mps,
        closing_speed_mps=closing_speed_mps,
        min_ttc=min_ttc,
        same_lane_min_ttc=same_lane_min_ttc,
        follow_target_speed_kmh=follow_target_speed_kmh,
        left_front_gap_m=left_front_gap_m,
        left_rear_gap_m=left_rear_gap_m,
        right_front_gap_m=right_front_gap_m,
        right_rear_gap_m=right_rear_gap_m,
    )
