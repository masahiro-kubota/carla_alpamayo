from __future__ import annotations

from dataclasses import dataclass

from ad_stack.overtake.domain import ScenarioKind


@dataclass(slots=True)
class PreflightValidationInput:
    scenario_kind: ScenarioKind
    ego_lane_id: str | None
    obstacle_lane_id: str | None
    obstacle_route_lane_id: str | None = None
    blocker_lane_id: str | None = None
    ego_to_obstacle_longitudinal_distance_m: float | None = None
    ego_to_blocker_longitudinal_distance_m: float | None = None
    left_lane_is_driving: bool = False
    right_lane_is_driving: bool = False
    route_target_lane_id: str | None = None
    nearest_signal_distance_m: float | None = None
    nearest_junction_distance_m: float | None = None
    route_aligned_adjacent_lane_available: bool = False


@dataclass(slots=True)
class ScenarioValidationResult:
    scenario_kind: ScenarioKind
    errors: list[str]
    warnings: list[str]

    @property
    def is_valid(self) -> bool:
        return not self.errors


def validate_preflight(snapshot: PreflightValidationInput) -> ScenarioValidationResult:
    errors: list[str] = []
    warnings: list[str] = []

    obstacle_aligned_with_route = (
        snapshot.obstacle_route_lane_id is not None
        and snapshot.obstacle_lane_id == snapshot.obstacle_route_lane_id
    )
    obstacle_expected_in_current_or_future_lane = snapshot.obstacle_lane_id == snapshot.ego_lane_id
    if snapshot.scenario_kind in {"curve_clear", "adjacent_lane_closed"}:
        obstacle_expected_in_current_or_future_lane = (
            obstacle_expected_in_current_or_future_lane or obstacle_aligned_with_route
        )
    if not obstacle_expected_in_current_or_future_lane:
        errors.append("obstacle_not_in_ego_lane")
    if (
        snapshot.ego_to_obstacle_longitudinal_distance_m is None
        or snapshot.ego_to_obstacle_longitudinal_distance_m <= 0.0
    ):
        errors.append("obstacle_not_ahead")
    if snapshot.scenario_kind != "adjacent_lane_closed":
        if not (snapshot.left_lane_is_driving or snapshot.right_lane_is_driving):
            errors.append("no_adjacent_driving_lane")
        if not snapshot.route_aligned_adjacent_lane_available:
            errors.append("route_aligned_adjacent_lane_unavailable")
    if (
        snapshot.route_target_lane_id is not None
        and snapshot.ego_lane_id is not None
        and snapshot.route_target_lane_id != snapshot.ego_lane_id
    ):
        errors.append("route_target_lane_conflict")
    signal_nearby = (
        snapshot.nearest_signal_distance_m is not None
        and snapshot.nearest_signal_distance_m < 25.0
    )
    junction_nearby = (
        snapshot.nearest_junction_distance_m is not None
        and snapshot.nearest_junction_distance_m < 30.0
    )
    if signal_nearby:
        warnings.append("signal_nearby")
    if junction_nearby:
        warnings.append("junction_nearby")

    if snapshot.scenario_kind == "blocked_static":
        if snapshot.blocker_lane_id is None:
            errors.append("blocker_missing")
        elif snapshot.blocker_lane_id == snapshot.ego_lane_id:
            errors.append("blocker_not_in_opposite_lane")

    if snapshot.scenario_kind == "blocked_oncoming":
        if snapshot.blocker_lane_id is None:
            errors.append("oncoming_blocker_missing")

    if snapshot.scenario_kind in {"double_stopped_separated", "double_stopped_clustered"}:
        if snapshot.obstacle_lane_id is None:
            errors.append("obstacle_actor_missing")

    if snapshot.scenario_kind == "adjacent_lane_closed":
        if snapshot.route_aligned_adjacent_lane_available or (
            snapshot.left_lane_is_driving or snapshot.right_lane_is_driving
        ):
            errors.append("adjacent_lane_not_closed")

    if snapshot.scenario_kind == "near_junction_preflight_reject":
        if signal_nearby:
            errors.append("signal_nearby")
        if junction_nearby:
            errors.append("junction_nearby")

    return ScenarioValidationResult(
        scenario_kind=snapshot.scenario_kind,
        errors=errors,
        warnings=warnings,
    )
