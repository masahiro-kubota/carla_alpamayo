from __future__ import annotations

from typing import Any, Protocol

from ad_stack.overtake.domain import OvertakeTargetCandidates, OvertakeLeadSnapshot

from .motion_profile import classify_motion_profile
from .route_projection import build_route_aligned_target_candidates


class TargetCandidateBuilder(Protocol):
    def __call__(
        self,
        *,
        tracked_objects: tuple[Any, ...],
        route_index: int | None,
        base_trace: list[tuple[Any, Any]],
        route_point_to_trace_index: list[int],
        route_point_progress_m: list[float],
        stopped_speed_threshold_mps: float,
    ) -> OvertakeTargetCandidates: ...


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


def build_same_lane_target_candidates(
    tracked_objects: tuple[Any, ...],
    *,
    stopped_speed_threshold_mps: float,
) -> list[OvertakeLeadSnapshot]:
    return [
        OvertakeLeadSnapshot(
            actor_id=actor.actor_id,
            lane_id=actor.lane_id,
            distance_m=float(actor.longitudinal_distance_m),
            speed_mps=float(actor.speed_mps),
            relative_speed_mps=0.0,
            motion_profile=classify_motion_profile(
                speed_mps=float(actor.speed_mps),
                stopped_speed_threshold_mps=stopped_speed_threshold_mps,
            ),
        )
        for actor in tracked_objects
        if (
            actor.relation == "same_lane"
            and actor.is_ahead
            and actor.longitudinal_distance_m is not None
            and float(actor.longitudinal_distance_m) > 0.0
        )
    ]


def build_target_candidates(
    *,
    tracked_objects: tuple[Any, ...],
    route_index: int | None,
    base_trace: list[tuple[Any, Any]],
    route_point_to_trace_index: list[int],
    route_point_progress_m: list[float],
    stopped_speed_threshold_mps: float,
) -> OvertakeTargetCandidates:
    return OvertakeTargetCandidates(
        same_lane=build_same_lane_target_candidates(
            tracked_objects,
            stopped_speed_threshold_mps=stopped_speed_threshold_mps,
        ),
        route_aligned=build_route_aligned_target_candidates(
            tracked_objects,
            route_index=route_index,
            base_trace=base_trace,
            route_point_to_trace_index=route_point_to_trace_index,
            route_point_progress_m=route_point_progress_m,
            stopped_speed_threshold_mps=stopped_speed_threshold_mps,
        ),
    )
