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
from ad_stack.overtake.policies import TargetPolicy

from .candidate_extractor import TargetCandidateBuilder, nearest_lead
from .motion_profile import classify_motion_profile
from .route_alignment import lane_id
from .route_projection import route_relative_progress_to_actor


@dataclass(slots=True)
class OvertakeSceneSnapshot:
    follow_actor: Any | None
    follow_lead: OvertakeLeadSnapshot | None
    active_target: OvertakeTargetSnapshot | None
    decision_context: OvertakeContext
    follow_distance_m: float | None
    follow_speed_mps: float
    same_lane_follow_distance_m: float | None
    same_lane_follow_speed_mps: float
    closing_speed_mps: float
    min_ttc: float
    same_lane_min_ttc: float
    follow_target_speed_kmh: float
    left_front_gap_m: float
    left_rear_gap_m: float
    right_front_gap_m: float
    right_rear_gap_m: float


@dataclass(slots=True)
class OvertakePassSnapshot:
    target_actor: Any | None
    visible_target_actors: tuple[Any, ...]
    target_actor_visible: bool
    target_longitudinal_distance_m: float | None
    target_exit_longitudinal_distance_m: float | None


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
                motion_profile=target.motion_profile,
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
    candidate_builder: TargetCandidateBuilder,
    target_policy: TargetPolicy,
    active_signal_state: str | None,
    signal_stop_distance_m: float | None,
    allow_overtake: bool,
    preferred_direction: str,
    world_map: Any,
    carla_module: Any,
    base_trace: list[tuple[Any, Any]],
    route_point_to_trace_index: list[int],
    route_point_progress_m: list[int] | list[float],
) -> OvertakeSceneSnapshot:
    follow_actor = nearest_lead(tracked_objects, relation="same_lane")
    target_candidates = candidate_builder(
        tracked_objects=tracked_objects,
        route_index=route_index,
        base_trace=base_trace,
        route_point_to_trace_index=route_point_to_trace_index,
        route_point_progress_m=route_point_progress_m,
        stopped_speed_threshold_mps=stopped_speed_threshold_mps,
    )
    same_lane_targets = target_policy(
        target_candidates.same_lane,
        cluster_merge_gap_m=cluster_merge_gap_m,
        cluster_max_member_speed_mps=cluster_max_member_speed_mps,
    )
    route_aligned_targets = target_policy(
        target_candidates.route_aligned,
        cluster_merge_gap_m=cluster_merge_gap_m,
        cluster_max_member_speed_mps=cluster_max_member_speed_mps,
    )
    route_aligned_targets = enrich_targets_with_adjacent_lane_availability(
        route_aligned_targets,
        tracked_objects=tracked_objects,
        world_map=world_map,
        carla_module=carla_module,
    )
    resolved_targets = same_lane_targets if same_lane_targets else route_aligned_targets
    active_target = resolved_targets[0] if resolved_targets else None

    same_lane_follow_distance_m = (
        float(follow_actor.longitudinal_distance_m)
        if follow_actor is not None and follow_actor.longitudinal_distance_m is not None
        else None
    )
    same_lane_follow_speed_mps = float(follow_actor.speed_mps) if follow_actor is not None else 0.0
    follow_distance_m = (
        same_lane_follow_distance_m
        if same_lane_follow_distance_m is not None
        else (float(active_target.entry_distance_m) if active_target is not None else None)
    )
    follow_speed_mps = (
        same_lane_follow_speed_mps
        if same_lane_follow_distance_m is not None
        else (float(active_target.speed_mps) if active_target is not None else 0.0)
    )
    closing_speed_mps = current_speed_mps - follow_speed_mps if follow_distance_m is not None else 0.0

    min_ttc = float("inf")
    if follow_distance_m is not None and closing_speed_mps > 1e-3:
        min_ttc = follow_distance_m / closing_speed_mps

    same_lane_min_ttc = float("inf")
    same_lane_closing_speed_mps = (
        current_speed_mps - same_lane_follow_speed_mps
        if same_lane_follow_distance_m is not None
        else 0.0
    )
    if same_lane_follow_distance_m is not None and same_lane_closing_speed_mps > 1e-3:
        same_lane_min_ttc = same_lane_follow_distance_m / same_lane_closing_speed_mps

    follow_target_speed_kmh = target_speed_kmh
    if follow_distance_m is not None:
        follow_target_speed_kmh = min(
            target_speed_kmh,
            max(0.0, follow_speed_mps * 3.6),
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
    follow_lead = (
        OvertakeLeadSnapshot(
            actor_id=(
                follow_actor.actor_id
                if follow_actor is not None
                else active_target.primary_actor_id
                if active_target is not None
                else None
            ),
            lane_id=(
                follow_actor.lane_id
                if follow_actor is not None
                else active_target.lane_id
                if active_target is not None
                else None
            ),
            distance_m=follow_distance_m,
            speed_mps=follow_speed_mps,
            relative_speed_mps=closing_speed_mps,
            motion_profile=(
                classify_motion_profile(
                    speed_mps=follow_speed_mps,
                    stopped_speed_threshold_mps=stopped_speed_threshold_mps,
                )
                if active_target is None
                else active_target.motion_profile
            ),
        )
        if follow_actor is not None or active_target is not None
        else None
    )
    decision_context = OvertakeContext(
        timestamp_s=timestamp_s,
        current_lane_id=current_lane_id,
        origin_lane_id=current_lane_id,
        route_target_lane_id=route_target_lane_id,
        target_speed_kmh=target_speed_kmh,
        lead=follow_lead,
        left_lane=left_lane_snapshot,
        right_lane=right_lane_snapshot,
        active_signal_state=active_signal_state,
        signal_stop_distance_m=signal_stop_distance_m,
        allow_overtake=allow_overtake,
        preferred_direction=preferred_direction,  # type: ignore[arg-type]
        active_target=active_target,
    )
    return OvertakeSceneSnapshot(
        follow_actor=follow_actor,
        follow_lead=follow_lead,
        active_target=active_target,
        decision_context=decision_context,
        follow_distance_m=follow_distance_m,
        follow_speed_mps=follow_speed_mps,
        same_lane_follow_distance_m=same_lane_follow_distance_m,
        same_lane_follow_speed_mps=same_lane_follow_speed_mps,
        closing_speed_mps=closing_speed_mps,
        min_ttc=min_ttc,
        same_lane_min_ttc=same_lane_min_ttc,
        follow_target_speed_kmh=follow_target_speed_kmh,
        left_front_gap_m=left_front_gap_m,
        left_rear_gap_m=left_rear_gap_m,
        right_front_gap_m=right_front_gap_m,
        right_rear_gap_m=right_rear_gap_m,
    )


def build_overtake_pass_snapshot(
    *,
    tracked_objects: tuple[Any, ...],
    target_actor_id: int | None,
    target_member_actor_ids: tuple[int, ...],
    route_index: int | None,
    base_trace: list[tuple[Any, Any]],
    route_point_to_trace_index: list[int],
    route_point_progress_m: list[float],
) -> OvertakePassSnapshot:
    if target_actor_id is None:
        return OvertakePassSnapshot(
            target_actor=None,
            visible_target_actors=(),
            target_actor_visible=False,
            target_longitudinal_distance_m=None,
            target_exit_longitudinal_distance_m=None,
        )

    target_actor, visible_target_actors = visible_overtake_target_actors(
        tracked_objects,
        target_actor_id=target_actor_id,
        target_member_actor_ids=target_member_actor_ids,
    )
    target_actor_visible = bool(visible_target_actors)
    target_longitudinal_distance_m = None
    target_exit_longitudinal_distance_m = None
    visible_route_relative_progress_m: list[float] = []
    if route_index is not None:
        for visible_actor in visible_target_actors:
            route_relative_progress_m, _distance_to_centerline_m, _route_lane_id = (
                route_relative_progress_to_actor(
                    actor=visible_actor,
                    reference_route_index=route_index,
                    base_trace=base_trace,
                    route_point_to_trace_index=route_point_to_trace_index,
                    route_point_progress_m=route_point_progress_m,
                    search_back_points=48,
                    search_forward_points=96,
                )
            )
            if route_relative_progress_m is not None:
                visible_route_relative_progress_m.append(route_relative_progress_m)
            if (
                target_actor is not None
                and visible_actor.actor_id == target_actor.actor_id
                and route_relative_progress_m is not None
            ):
                target_longitudinal_distance_m = route_relative_progress_m
    if visible_route_relative_progress_m:
        target_exit_longitudinal_distance_m = max(visible_route_relative_progress_m)
    return OvertakePassSnapshot(
        target_actor=target_actor,
        visible_target_actors=visible_target_actors,
        target_actor_visible=target_actor_visible,
        target_longitudinal_distance_m=target_longitudinal_distance_m,
        target_exit_longitudinal_distance_m=target_exit_longitudinal_distance_m,
    )
