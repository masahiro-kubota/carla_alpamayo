from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Literal, Sequence

from simulation.environment_config import EnvironmentConfigSpec

ScenarioKind = Literal[
    "clear",
    "blocked_static",
    "blocked_oncoming",
    "signal_suppressed",
    "rejoin_blocked_then_release",
    "adjacent_lane_closed",
    "double_stopped_separated",
    "double_stopped_clustered",
    "curve_clear",
    "near_junction_preflight_reject",
]


@dataclass(slots=True)
class OvertakeScenarioConfig:
    scenario_kind: ScenarioKind
    obstacle_npc_index: int = 0
    blocker_npc_index: int | None = None
    route_aligned_adjacent_lane_available: bool | None = None
    nearest_signal_distance_m: float | None = None
    nearest_junction_distance_m: float | None = None


def parse_overtake_scenario_config(
    raw: dict[str, Any] | None,
) -> OvertakeScenarioConfig | None:
    if raw is None:
        return None
    return OvertakeScenarioConfig(
        scenario_kind=str(raw["scenario_kind"]),  # type: ignore[arg-type]
        obstacle_npc_index=int(raw.get("obstacle_npc_index", 0)),
        blocker_npc_index=(
            int(raw["blocker_npc_index"]) if raw.get("blocker_npc_index") is not None else None
        ),
        route_aligned_adjacent_lane_available=(
            bool(raw["route_aligned_adjacent_lane_available"])
            if raw.get("route_aligned_adjacent_lane_available") is not None
            else None
        ),
        nearest_signal_distance_m=(
            float(raw["nearest_signal_distance_m"])
            if raw.get("nearest_signal_distance_m") is not None
            else None
        ),
        nearest_junction_distance_m=(
            float(raw["nearest_junction_distance_m"])
            if raw.get("nearest_junction_distance_m") is not None
            else None
        ),
    )


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


def _lane_id(waypoint: Any | None) -> str | None:
    if waypoint is None:
        return None
    return f"{waypoint.road_id}:{waypoint.lane_id}"


def _distance_ahead_m(*, origin_waypoint: Any | None, target_waypoint: Any | None) -> float | None:
    if origin_waypoint is None or target_waypoint is None:
        return None
    if (
        int(origin_waypoint.road_id) == int(target_waypoint.road_id)
        and int(origin_waypoint.lane_id) == int(target_waypoint.lane_id)
    ):
        origin_s = getattr(origin_waypoint, "s", None)
        target_s = getattr(target_waypoint, "s", None)
        if origin_s is not None and target_s is not None:
            longitudinal_distance_m = float(target_s) - float(origin_s)
            if abs(longitudinal_distance_m) > 1e-3:
                return longitudinal_distance_m
    origin_rotation = getattr(origin_waypoint.transform, "rotation", None)
    yaw_deg = getattr(origin_rotation, "yaw", None)
    if yaw_deg is not None:
        heading_rad = math.radians(float(yaw_deg))
        forward_x = math.cos(heading_rad)
        forward_y = math.sin(heading_rad)
        delta_x = float(target_waypoint.transform.location.x) - float(origin_waypoint.transform.location.x)
        delta_y = float(target_waypoint.transform.location.y) - float(origin_waypoint.transform.location.y)
        forward_projection_m = (delta_x * forward_x) + (delta_y * forward_y)
        if abs(forward_projection_m) > 1e-3:
            return float(forward_projection_m)
    return float(origin_waypoint.transform.location.distance(target_waypoint.transform.location))


def _nearest_route_waypoint(route_trace: list[tuple[Any, Any]], *, location: Any) -> Any | None:
    if not route_trace:
        return None
    nearest_index = min(
        range(len(route_trace)),
        key=lambda index: float(route_trace[index][0].transform.location.distance(location)),
    )
    return route_trace[nearest_index][0]


def _nearest_junction_distance_along_route(
    route_trace: list[tuple[Any, Any]],
    *,
    location: Any,
) -> float | None:
    if not route_trace:
        return None
    nearest_index = min(
        range(len(route_trace)),
        key=lambda index: float(route_trace[index][0].transform.location.distance(location)),
    )
    accumulated_distance_m = 0.0
    previous_location = route_trace[nearest_index][0].transform.location
    if bool(route_trace[nearest_index][0].is_junction):
        return 0.0
    for waypoint, _road_option in route_trace[nearest_index + 1 :]:
        current_location = waypoint.transform.location
        accumulated_distance_m += float(previous_location.distance(current_location))
        if bool(waypoint.is_junction):
            return accumulated_distance_m
        previous_location = current_location
    return None


def build_overtake_scenario_validation(
    *,
    environment_config: EnvironmentConfigSpec | None,
    world_map: Any,
    route_trace: list[tuple[Any, Any]],
    ego_vehicle: Any,
    npc_actor_refs: Sequence[Any],
    driving_lane_type: Any,
) -> dict[str, Any] | None:
    raw_contract = environment_config.overtake_scenario if environment_config is not None else None
    scenario_config = parse_overtake_scenario_config(raw_contract)
    if scenario_config is None:
        return None

    if not npc_actor_refs:
        return {
            "scenario_kind": scenario_config.scenario_kind,
            "valid": False,
            "errors": ["obstacle_actor_missing"],
            "warnings": [],
            "snapshot": None,
        }

    if (
        scenario_config.obstacle_npc_index < 0
        or scenario_config.obstacle_npc_index >= len(npc_actor_refs)
    ):
        return {
            "scenario_kind": scenario_config.scenario_kind,
            "valid": False,
            "errors": ["obstacle_actor_missing"],
            "warnings": [],
            "snapshot": None,
        }

    ego_waypoint = world_map.get_waypoint(ego_vehicle.get_location())
    obstacle_actor = npc_actor_refs[scenario_config.obstacle_npc_index]
    obstacle_waypoint = world_map.get_waypoint(obstacle_actor.get_location())
    blocker_waypoint = None
    blocker_actor_id = None
    if (
        scenario_config.blocker_npc_index is not None
        and 0 <= scenario_config.blocker_npc_index < len(npc_actor_refs)
    ):
        blocker_actor = npc_actor_refs[scenario_config.blocker_npc_index]
        blocker_actor_id = int(blocker_actor.id)
        blocker_waypoint = world_map.get_waypoint(blocker_actor.get_location())

    left_waypoint = obstacle_waypoint.get_left_lane() if obstacle_waypoint is not None else None
    right_waypoint = obstacle_waypoint.get_right_lane() if obstacle_waypoint is not None else None
    left_lane_is_driving = left_waypoint is not None and left_waypoint.lane_type == driving_lane_type
    right_lane_is_driving = (
        right_waypoint is not None and right_waypoint.lane_type == driving_lane_type
    )
    route_aligned_adjacent_lane_available = (
        bool(scenario_config.route_aligned_adjacent_lane_available)
        if scenario_config.route_aligned_adjacent_lane_available is not None
        else bool(left_lane_is_driving or right_lane_is_driving)
    )
    nearest_junction_distance_m = (
        float(scenario_config.nearest_junction_distance_m)
        if scenario_config.nearest_junction_distance_m is not None
        else _nearest_junction_distance_along_route(route_trace, location=obstacle_actor.get_location())
    )
    nearest_signal_distance_m = (
        float(scenario_config.nearest_signal_distance_m)
        if scenario_config.nearest_signal_distance_m is not None
        else None
    )

    validation_input = PreflightValidationInput(
        scenario_kind=scenario_config.scenario_kind,
        ego_lane_id=_lane_id(ego_waypoint),
        obstacle_lane_id=_lane_id(obstacle_waypoint),
        obstacle_route_lane_id=_lane_id(
            _nearest_route_waypoint(route_trace, location=obstacle_actor.get_location())
        ),
        blocker_lane_id=_lane_id(blocker_waypoint),
        ego_to_obstacle_longitudinal_distance_m=_distance_ahead_m(
            origin_waypoint=ego_waypoint,
            target_waypoint=obstacle_waypoint,
        ),
        ego_to_blocker_longitudinal_distance_m=_distance_ahead_m(
            origin_waypoint=ego_waypoint,
            target_waypoint=blocker_waypoint,
        ),
        left_lane_is_driving=bool(left_lane_is_driving),
        right_lane_is_driving=bool(right_lane_is_driving),
        route_target_lane_id=_lane_id(
            _nearest_route_waypoint(route_trace, location=ego_vehicle.get_location())
        ),
        nearest_signal_distance_m=nearest_signal_distance_m,
        nearest_junction_distance_m=nearest_junction_distance_m,
        route_aligned_adjacent_lane_available=route_aligned_adjacent_lane_available,
    )
    validation_result = validate_preflight(validation_input)
    return {
        "scenario_kind": scenario_config.scenario_kind,
        "valid": bool(validation_result.is_valid),
        "errors": list(validation_result.errors),
        "warnings": list(validation_result.warnings),
        "snapshot": {
            "ego_lane_id": validation_input.ego_lane_id,
            "obstacle_actor_id": int(obstacle_actor.id),
            "obstacle_lane_id": validation_input.obstacle_lane_id,
            "obstacle_route_lane_id": validation_input.obstacle_route_lane_id,
            "blocker_actor_id": blocker_actor_id,
            "blocker_lane_id": validation_input.blocker_lane_id,
            "ego_to_obstacle_longitudinal_distance_m": (
                round(validation_input.ego_to_obstacle_longitudinal_distance_m, 3)
                if validation_input.ego_to_obstacle_longitudinal_distance_m is not None
                else None
            ),
            "ego_to_blocker_longitudinal_distance_m": (
                round(validation_input.ego_to_blocker_longitudinal_distance_m, 3)
                if validation_input.ego_to_blocker_longitudinal_distance_m is not None
                else None
            ),
            "left_lane_is_driving": bool(validation_input.left_lane_is_driving),
            "right_lane_is_driving": bool(validation_input.right_lane_is_driving),
            "route_target_lane_id": validation_input.route_target_lane_id,
            "route_aligned_adjacent_lane_available": bool(
                validation_input.route_aligned_adjacent_lane_available
            ),
            "nearest_signal_distance_m": (
                round(validation_input.nearest_signal_distance_m, 3)
                if validation_input.nearest_signal_distance_m is not None
                else None
            ),
            "nearest_junction_distance_m": (
                round(validation_input.nearest_junction_distance_m, 3)
                if validation_input.nearest_junction_distance_m is not None
                else None
            ),
        },
    }


def warm_up_and_build_overtake_scenario_validation(
    *,
    world: Any,
    environment_config: EnvironmentConfigSpec | None,
    route_trace: list[tuple[Any, Any]],
    ego_vehicle: Any,
    npc_actor_refs: Sequence[Any],
    driving_lane_type: Any,
    warmup_ticks: int = 3,
) -> dict[str, Any] | None:
    for _ in range(max(0, int(warmup_ticks))):
        world.tick()
    return build_overtake_scenario_validation(
        environment_config=environment_config,
        world_map=world.get_map(),
        route_trace=route_trace,
        ego_vehicle=ego_vehicle,
        npc_actor_refs=npc_actor_refs,
        driving_lane_type=driving_lane_type,
    )
