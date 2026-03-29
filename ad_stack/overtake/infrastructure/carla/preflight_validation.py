from __future__ import annotations

import math
from typing import Any, Sequence

from ad_stack.overtake.validation import PreflightValidationInput, validate_preflight
from simulation.environment_config import EnvironmentConfigSpec


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


def build_stopped_obstacle_scenario_validation(
    *,
    environment_config: EnvironmentConfigSpec | None,
    world_map: Any,
    route_trace: list[tuple[Any, Any]],
    ego_vehicle: Any,
    npc_actor_refs: Sequence[Any],
    driving_lane_type: Any,
) -> dict[str, Any] | None:
    scenario_config = (
        environment_config.stopped_obstacle_scenario if environment_config is not None else None
    )
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


def warm_up_and_build_stopped_obstacle_scenario_validation(
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
    return build_stopped_obstacle_scenario_validation(
        environment_config=environment_config,
        world_map=world.get_map(),
        route_trace=route_trace,
        ego_vehicle=ego_vehicle,
        npc_actor_refs=npc_actor_refs,
        driving_lane_type=driving_lane_type,
    )
