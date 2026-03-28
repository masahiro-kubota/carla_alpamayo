from __future__ import annotations

import json
import queue
import random
import tempfile
from collections.abc import Callable
from dataclasses import dataclass, field
from math import inf
from pathlib import Path
from typing import Any, Literal

from ad_stack.api import (
    create_expert_collector_stack,
    create_interactive_pilotnet_controller,
    create_pilotnet_eval_stack,
    to_carla_control,
)
from ad_stack.expert_config import bind_runtime_overrides, expert_config_to_dict, load_expert_config
from libs.carla_utils import (
    FrameEventTracker,
    attach_sensor,
    build_planned_route,
    destroy_actors,
    ensure_carla_agents_on_path,
    load_default_topdown_map_asset,
    load_route_config,
    relative_to_project,
    require_blueprint,
    route_geometry_from_planned_route,
    setup_world,
    speed_mps,
    wait_for_image,
)
from libs.carla_utils.traffic_light_phasing import (
    TrafficLightApproach,
    TrafficLightPhaseCycle,
    build_opposing_phase_groups,
    compute_phase_states,
)
from libs.project import PROJECT_ROOT, build_versioned_run_id, ensure_clean_git_worktree
from libs.schemas import (
    EgoStateSample,
    EpisodeRecord,
    NPCVehicleStateSample,
    RotatingRouteLoopMcapWriter,
    TrackedVehicleStateSample,
    TrafficLightObservationSample,
    append_jsonl,
)
from libs.utils import render_png_sequence_to_mp4

RunMode = Literal["evaluate", "interactive"]
PolicyKind = Literal["expert", "learned", "interactive"]
McapMapScope = Literal["full", "near_route"]
InteractiveCommandProvider = Callable[[str], tuple[str, bool]]
InteractiveStatusSink = Callable[[str], None]
InteractivePreviewSink = Callable[[Any, str], bool | None]


@dataclass(slots=True)
class RouteLoopScenarioSpec:
    route_config_path: Path
    weather: str = "ClearNoon"
    goal_tolerance_m: float = 10.0
    max_stop_seconds: float = 10.0
    stationary_speed_threshold_mps: float = 0.5
    max_seconds: float = 600.0
    environment_config_path: Path | None = None


@dataclass(slots=True)
class NPCProfileSpec:
    name: str
    default_target_speed_kmh: float = 20.0
    speed_jitter_kmh: float = 0.0
    enable_autopilot: bool = True


@dataclass(slots=True)
class NPCVehicleSpec:
    spawn_index: int
    npc_profile_id: str | None = None
    target_speed_kmh: float | None = None
    lane_behavior: str = "keep_lane"
    vehicle_filter: str = "vehicle.*"


@dataclass(slots=True)
class TrafficLightOverrideSpec:
    actor_id: int
    state: str
    freeze: bool = True


@dataclass(slots=True)
class TrafficLightSchedulePhaseSpec:
    at_seconds: float
    state: str
    freeze: bool = True


@dataclass(slots=True)
class TrafficLightScheduleSpec:
    actor_id: int
    phases: list[TrafficLightSchedulePhaseSpec] = field(default_factory=list)


@dataclass(slots=True)
class TrafficLightPhaseCycleSpec:
    green_seconds: float
    yellow_seconds: float
    all_red_seconds: float
    reset_groups: bool = True
    initial_offset_seconds: float = 0.0


@dataclass(slots=True)
class TrafficLightPhaseRuntimeGroup:
    actor_ids: list[int]
    phase_actor_ids: list[list[int]]


@dataclass(slots=True)
class EnvironmentConfigSpec:
    name: str
    town: str
    weather: str | None = None
    goal_tolerance_m: float | None = None
    max_stop_seconds: float | None = None
    stationary_speed_threshold_mps: float | None = None
    max_seconds: float | None = None
    npc_vehicles: list[NPCVehicleSpec] = field(default_factory=list)
    traffic_light_phase_cycle: TrafficLightPhaseCycleSpec | None = None
    traffic_light_overrides: list[TrafficLightOverrideSpec] = field(default_factory=list)
    traffic_light_schedules: list[TrafficLightScheduleSpec] = field(default_factory=list)
    description: str = ""


@dataclass(slots=True)
class InteractiveScenarioSpec:
    town: str = "Town01"
    spawn_index: int = 70
    weather: str = "ClearNoon"
    max_seconds: float = 1800.0
    spectator_follow_distance_m: float = 7.0
    spectator_height_m: float = 3.0


@dataclass(slots=True)
class RuntimeSpec:
    host: str = "127.0.0.1"
    port: int = 2000
    vehicle_filter: str = "vehicle.tesla.model3"
    fixed_delta_seconds: float = 0.05
    sensor_timeout: float = 2.0
    camera_width: int = 1280
    camera_height: int = 720
    camera_fov: int = 90
    target_speed_kmh: float | None = None
    seed: int = 7


@dataclass(slots=True)
class ArtifactSpec:
    record_video: bool = True
    video_crf: int = 23
    video_fps: float | None = None
    record_mcap: bool = True
    mcap_jpeg_quality: int = 85
    mcap_segment_seconds: float = 600.0
    record_hz: float = 10.0
    mcap_map_scope: McapMapScope = "full"


@dataclass(slots=True)
class PolicySpec:
    kind: PolicyKind
    checkpoint_path: Path | None = None
    expert_config_path: Path | None = None
    device: str | None = None
    steer_smoothing: float = 1.0
    max_steer_delta: float | None = None
    ignore_traffic_lights: bool = False
    ignore_stop_signs: bool = True
    ignore_vehicles: bool = False
    initial_command: str = "lanefollow"
    command_provider: InteractiveCommandProvider | None = None
    status_sink: InteractiveStatusSink | None = None
    preview_sink: InteractivePreviewSink | None = None


@dataclass(slots=True)
class RunRequest:
    mode: RunMode
    scenario: RouteLoopScenarioSpec | InteractiveScenarioSpec
    runtime: RuntimeSpec
    policy: PolicySpec
    artifacts: ArtifactSpec = field(default_factory=ArtifactSpec)


@dataclass(slots=True)
class RunResult:
    mode: RunMode
    success: bool
    summary: dict[str, Any]
    episode_id: str | None = None
    output_dir: Path | None = None
    summary_path: Path | None = None
    manifest_path: Path | None = None
    video_path: Path | None = None
    mcap_path: Path | None = None
    frame_count: int = 0
    elapsed_seconds: float = 0.0


def _require_carla():
    try:
        import carla
    except ModuleNotFoundError as exc:  # pragma: no cover - exercised in runtime env
        raise SystemExit(
            "The 'carla' Python package is not installed. "
            "Run 'uv sync' after confirming the CARLA wheel path in pyproject.toml."
        ) from exc
    return carla


def _resolve_weather(carla_module: Any, weather_name: str):
    weather = getattr(carla_module.WeatherParameters, weather_name, None)
    if weather is None:
        raise ValueError(f"Unknown CARLA weather preset: {weather_name}")
    return weather


def _carla_image_to_rgb_array(image) -> Any:
    import numpy as np

    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))
    return array[:, :, :3][:, :, ::-1].copy()


def _relative_or_none(path: Path | None) -> str | None:
    if path is None:
        return None
    return relative_to_project(path)


def _lane_id(waypoint: Any | None) -> str | None:
    if waypoint is None:
        return None
    return f"{waypoint.road_id}:{waypoint.lane_id}"


def _load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _load_npc_profile(profile_id: str) -> NPCProfileSpec:
    profile_path = PROJECT_ROOT / "scenarios" / "npc_profiles" / f"{profile_id}.json"
    raw = _load_json(profile_path)
    return NPCProfileSpec(
        name=str(raw["name"]),
        default_target_speed_kmh=float(raw.get("default_target_speed_kmh", 20.0)),
        speed_jitter_kmh=float(raw.get("speed_jitter_kmh", 0.0)),
        enable_autopilot=bool(raw.get("enable_autopilot", True)),
    )


def _load_environment_config(path: Path) -> EnvironmentConfigSpec:
    raw = _load_json(path)
    raw_cycle = raw.get("traffic_light_phase_cycle")
    if raw_cycle is None:
        raw_cycle = raw.get("traffic_light_group_cycle")
    all_red_seconds = None
    if raw_cycle is not None:
        all_red_seconds = raw_cycle.get("all_red_seconds")
        if all_red_seconds is None:
            all_red_seconds = raw_cycle["red_seconds"]
    return EnvironmentConfigSpec(
        name=str(raw["name"]),
        town=str(raw["town"]),
        weather=str(raw["weather"]) if raw.get("weather") is not None else None,
        goal_tolerance_m=float(raw["goal_tolerance_m"])
        if raw.get("goal_tolerance_m") is not None
        else None,
        max_stop_seconds=float(raw["max_stop_seconds"])
        if raw.get("max_stop_seconds") is not None
        else None,
        stationary_speed_threshold_mps=(
            float(raw["stationary_speed_threshold_mps"])
            if raw.get("stationary_speed_threshold_mps") is not None
            else None
        ),
        max_seconds=float(raw["max_seconds"]) if raw.get("max_seconds") is not None else None,
        npc_vehicles=[
            NPCVehicleSpec(
                spawn_index=int(item["spawn_index"]),
                npc_profile_id=str(item["npc_profile_id"]) if item.get("npc_profile_id") else None,
                target_speed_kmh=float(item["target_speed_kmh"])
                if item.get("target_speed_kmh") is not None
                else None,
                lane_behavior=str(item.get("lane_behavior", "keep_lane")),
                vehicle_filter=str(item.get("vehicle_filter", "vehicle.*")),
            )
            for item in raw.get("npc_vehicles", [])
        ],
        traffic_light_phase_cycle=(
            TrafficLightPhaseCycleSpec(
                green_seconds=float(raw_cycle["green_seconds"]),
                yellow_seconds=float(raw_cycle["yellow_seconds"]),
                all_red_seconds=float(all_red_seconds),
                reset_groups=bool(raw_cycle.get("reset_groups", True)),
                initial_offset_seconds=float(raw_cycle.get("initial_offset_seconds", 0.0)),
            )
            if raw_cycle is not None
            else None
        ),
        traffic_light_overrides=[
            TrafficLightOverrideSpec(
                actor_id=int(item["actor_id"]),
                state=str(item["state"]),
                freeze=bool(item.get("freeze", True)),
            )
            for item in raw.get("traffic_light_overrides", [])
        ],
        traffic_light_schedules=[
            TrafficLightScheduleSpec(
                actor_id=int(item["actor_id"]),
                phases=[
                    TrafficLightSchedulePhaseSpec(
                        at_seconds=float(phase["at_seconds"]),
                        state=str(phase["state"]),
                        freeze=bool(phase.get("freeze", True)),
                    )
                    for phase in sorted(
                        item.get("phases", []),
                        key=lambda phase: float(phase["at_seconds"]),
                    )
                ],
            )
            for item in raw.get("traffic_light_schedules", [])
        ],
        description=str(raw.get("description", "")),
    )


def _resolved_npc_speed_kmh(
    spec: NPCVehicleSpec, profile: NPCProfileSpec | None, rng: random.Random
) -> float:
    if spec.target_speed_kmh is not None:
        return float(spec.target_speed_kmh)
    if profile is None:
        return 20.0
    jitter = profile.speed_jitter_kmh
    return max(0.0, profile.default_target_speed_kmh + rng.uniform(-jitter, jitter))


def _update_spectator(
    carla_module: Any,
    spectator: Any,
    vehicle_transform: Any,
    *,
    follow_distance_m: float,
    height_m: float,
) -> None:
    import math

    yaw_rad = math.radians(vehicle_transform.rotation.yaw)
    vehicle_location = vehicle_transform.location
    spectator_location = carla_module.Location(
        x=vehicle_location.x - (follow_distance_m * math.cos(yaw_rad)),
        y=vehicle_location.y - (follow_distance_m * math.sin(yaw_rad)),
        z=vehicle_location.z + height_m,
    )
    spectator_rotation = carla_module.Rotation(
        pitch=-15.0,
        yaw=vehicle_transform.rotation.yaw,
        roll=0.0,
    )
    spectator.set_transform(carla_module.Transform(spectator_location, spectator_rotation))


def _traffic_lights_by_id(world: Any) -> dict[int, Any]:
    return {int(actor.id): actor for actor in world.get_actors().filter("*traffic_light*")}


def _set_traffic_light_state(
    carla_module: Any, traffic_light: Any, *, state_name: str, freeze: bool
) -> None:
    state_map = {
        "red": carla_module.TrafficLightState.Red,
        "yellow": carla_module.TrafficLightState.Yellow,
        "green": carla_module.TrafficLightState.Green,
    }
    state = state_map.get(state_name.lower())
    if state is None:
        raise ValueError(f"Unsupported traffic light override state: {state_name}")
    traffic_light.set_state(state)
    traffic_light.freeze(bool(freeze))


def _apply_traffic_light_overrides(
    carla_module: Any,
    world: Any,
    overrides: list[TrafficLightOverrideSpec],
) -> dict[int, Any]:
    lights_by_id = _traffic_lights_by_id(world)
    if not overrides:
        return lights_by_id
    for override in overrides:
        traffic_light = lights_by_id.get(override.actor_id)
        if traffic_light is None:
            raise RuntimeError(
                f"Traffic light actor_id={override.actor_id} not found for override."
            )
        _set_traffic_light_state(
            carla_module,
            traffic_light,
            state_name=override.state,
            freeze=override.freeze,
        )
    return lights_by_id


def _apply_traffic_light_group_cycle(
    world: Any,
    cycle: TrafficLightPhaseCycleSpec | None,
) -> tuple[dict[int, Any], list[TrafficLightPhaseRuntimeGroup]]:
    lights_by_id = _traffic_lights_by_id(world)
    if cycle is None:
        return lights_by_id, []

    ensure_carla_agents_on_path()
    from agents.tools.misc import get_trafficlight_trigger_location

    world_map = world.get_map()
    runtime_groups: list[TrafficLightPhaseRuntimeGroup] = []
    visited_groups: set[tuple[int, ...]] = set()
    for traffic_light in lights_by_id.values():
        group = list(traffic_light.get_group_traffic_lights())
        if not group:
            group = [traffic_light]
        group_key = tuple(sorted(int(actor.id) for actor in group))
        if group_key in visited_groups:
            continue
        visited_groups.add(group_key)

        approaches: list[TrafficLightApproach] = []
        for actor in group:
            actor.freeze(False)
            trigger_location = get_trafficlight_trigger_location(actor)
            trigger_waypoint = world_map.get_waypoint(trigger_location)
            approaches.append(
                TrafficLightApproach(
                    actor_id=int(actor.id),
                    heading_deg=float(trigger_waypoint.transform.rotation.yaw),
                )
            )
        runtime_groups.append(
            TrafficLightPhaseRuntimeGroup(
                actor_ids=list(group_key),
                phase_actor_ids=build_opposing_phase_groups(approaches),
            )
        )
    return lights_by_id, runtime_groups


def _apply_derived_traffic_light_group_cycle(
    carla_module: Any,
    lights_by_id: dict[int, Any],
    runtime_groups: list[TrafficLightPhaseRuntimeGroup],
    cycle: TrafficLightPhaseCycleSpec | None,
    *,
    elapsed_seconds: float,
    applied_states: dict[int, str],
    excluded_actor_ids: set[int],
) -> None:
    if cycle is None or not runtime_groups:
        return

    phase_cycle = TrafficLightPhaseCycle(
        green_seconds=float(cycle.green_seconds),
        yellow_seconds=float(cycle.yellow_seconds),
        all_red_seconds=float(cycle.all_red_seconds),
        initial_offset_seconds=float(cycle.initial_offset_seconds),
    )
    for runtime_group in runtime_groups:
        actor_states = compute_phase_states(
            runtime_group.phase_actor_ids,
            elapsed_seconds=elapsed_seconds,
            cycle=phase_cycle,
        )
        for actor_id in runtime_group.actor_ids:
            if actor_id in excluded_actor_ids:
                continue
            state_name = actor_states.get(actor_id, "red")
            if applied_states.get(actor_id) == state_name:
                continue
            traffic_light = lights_by_id.get(actor_id)
            if traffic_light is None:
                continue
            _set_traffic_light_state(
                carla_module,
                traffic_light,
                state_name=state_name,
                freeze=True,
            )
            applied_states[actor_id] = state_name


def _apply_traffic_light_schedules(
    carla_module: Any,
    lights_by_id: dict[int, Any],
    schedules: list[TrafficLightScheduleSpec],
    *,
    elapsed_seconds: float,
    applied_phase_indices: dict[int, int],
) -> None:
    if not schedules:
        return
    for schedule in schedules:
        traffic_light = lights_by_id.get(schedule.actor_id)
        if traffic_light is None:
            raise RuntimeError(
                f"Traffic light actor_id={schedule.actor_id} not found for schedule."
            )
        latest_index = -1
        for phase_index, phase in enumerate(schedule.phases):
            if phase.at_seconds <= elapsed_seconds:
                latest_index = phase_index
            else:
                break
        if latest_index < 0:
            continue
        if applied_phase_indices.get(schedule.actor_id) == latest_index:
            continue
        phase = schedule.phases[latest_index]
        _set_traffic_light_state(
            carla_module,
            traffic_light,
            state_name=phase.state,
            freeze=phase.freeze,
        )
        applied_phase_indices[schedule.actor_id] = latest_index


def _spawn_npc_vehicles(
    *,
    carla_module: Any,
    client: Any,
    world: Any,
    runtime: RuntimeSpec,
    environment_config: EnvironmentConfigSpec | None,
    rng: random.Random,
    actors: list[Any],
    spawned_actor_refs: list[Any] | None = None,
) -> list[dict[str, Any]]:
    if environment_config is None or not environment_config.npc_vehicles:
        return []

    traffic_manager = client.get_trafficmanager(8000)
    traffic_manager.set_synchronous_mode(True)
    traffic_manager.set_global_distance_to_leading_vehicle(1.5)
    spawn_points = world.get_map().get_spawn_points()
    spawned: list[dict[str, Any]] = []
    for npc_spec in environment_config.npc_vehicles:
        if npc_spec.spawn_index < 0 or npc_spec.spawn_index >= len(spawn_points):
            raise RuntimeError(f"NPC spawn_index out of range: {npc_spec.spawn_index}")
        profile = _load_npc_profile(npc_spec.npc_profile_id) if npc_spec.npc_profile_id else None
        desired_speed_kmh = _resolved_npc_speed_kmh(npc_spec, profile, rng)
        blueprint = require_blueprint(world, npc_spec.vehicle_filter, rng=rng)
        blueprint.set_attribute("role_name", "npc")
        actor = world.try_spawn_actor(blueprint, spawn_points[npc_spec.spawn_index])
        if actor is None:
            raise RuntimeError(
                f"Failed to spawn NPC vehicle at spawn index {npc_spec.spawn_index}."
            )
        actors.append(actor)
        if spawned_actor_refs is not None:
            spawned_actor_refs.append(actor)

        if profile is None or profile.enable_autopilot:
            actor.set_autopilot(True, traffic_manager.get_port())
            traffic_manager.auto_lane_change(actor, npc_spec.lane_behavior != "keep_lane")
            traffic_manager.ignore_lights_percentage(actor, 0.0)
            traffic_manager.ignore_vehicles_percentage(actor, 0.0)
            traffic_manager.set_desired_speed(actor, float(desired_speed_kmh))
        else:
            actor.apply_control(
                carla_module.VehicleControl(
                    throttle=0.0,
                    brake=1.0,
                    hand_brake=True,
                )
            )
        spawned.append(
            {
                "actor_id": int(actor.id),
                "spawn_index": npc_spec.spawn_index,
                "target_speed_kmh": round(desired_speed_kmh, 2),
                "lane_behavior": npc_spec.lane_behavior,
                "npc_profile_id": npc_spec.npc_profile_id,
                "autopilot_enabled": bool(profile is None or profile.enable_autopilot),
            }
        )
    return spawned


def _collect_npc_vehicle_states(
    *,
    npc_actor_refs: list[Any],
    npc_actor_metadata_by_id: dict[int, dict[str, Any]],
    world_map: Any,
) -> list[NPCVehicleStateSample]:
    states: list[NPCVehicleStateSample] = []
    for actor in npc_actor_refs:
        try:
            actor_id = int(actor.id)
            transform = actor.get_transform()
            location = transform.location
            rotation = transform.rotation
            actor_type_id = str(actor.type_id)
            current_speed_mps = float(speed_mps(actor))
            waypoint = world_map.get_waypoint(location)
            bounding_box = actor.bounding_box
        except RuntimeError as exc:
            if "destroyed actor" in str(exc):
                continue
            raise
        metadata = npc_actor_metadata_by_id.get(actor_id, {})
        states.append(
            NPCVehicleStateSample(
                actor_id=actor_id,
                type_id=actor_type_id,
                spawn_index=(
                    int(metadata["spawn_index"])
                    if metadata.get("spawn_index") is not None
                    else None
                ),
                target_speed_kmh=(
                    float(metadata["target_speed_kmh"])
                    if metadata.get("target_speed_kmh") is not None
                    else None
                ),
                npc_profile_id=str(metadata["npc_profile_id"])
                if metadata.get("npc_profile_id") is not None
                else None,
                lane_behavior=str(metadata["lane_behavior"])
                if metadata.get("lane_behavior") is not None
                else None,
                autopilot_enabled=(
                    bool(metadata["autopilot_enabled"])
                    if metadata.get("autopilot_enabled") is not None
                    else None
                ),
                speed_mps=current_speed_mps,
                lane_id=_lane_id(waypoint),
                pose={
                    "x": float(location.x),
                    "y": float(location.y),
                    "z": float(location.z),
                    "yaw_deg": float(rotation.yaw),
                    "pitch_deg": float(rotation.pitch),
                    "roll_deg": float(rotation.roll),
                },
                size={
                    "x": float(bounding_box.extent.x * 2.0),
                    "y": float(bounding_box.extent.y * 2.0),
                    "z": float(bounding_box.extent.z * 2.0),
                },
            )
        )
    return states


def _collect_tracked_vehicle_states(*, scene_state: Any | None) -> list[TrackedVehicleStateSample]:
    if scene_state is None:
        return []
    tracked_states: list[TrackedVehicleStateSample] = []
    for actor in getattr(scene_state, "tracked_objects", ()) or ():
        tracked_states.append(
            TrackedVehicleStateSample(
                actor_id=int(actor.actor_id),
                relation=str(actor.relation),
                lane_id=str(actor.lane_id) if actor.lane_id is not None else None,
                speed_mps=float(actor.speed_mps),
                longitudinal_distance_m=(
                    float(actor.longitudinal_distance_m)
                    if actor.longitudinal_distance_m is not None
                    else None
                ),
                lateral_distance_m=(
                    float(actor.lateral_distance_m) if actor.lateral_distance_m is not None else None
                ),
                is_ahead=bool(actor.is_ahead),
                pose={
                    "x": float(actor.x_m),
                    "y": float(actor.y_m),
                    "yaw_deg": float(actor.yaw_deg),
                },
            )
        )
    return tracked_states


def _collect_traffic_light_states(
    *, scene_state: Any | None
) -> list[TrafficLightObservationSample]:
    if scene_state is None:
        return []
    traffic_light_states: list[TrafficLightObservationSample] = []
    for light in getattr(scene_state, "traffic_lights", ()) or ():
        traffic_light_states.append(
            TrafficLightObservationSample(
                actor_id=int(light.actor_id),
                state=str(light.state),
                affects_ego=bool(light.affects_ego),
                distance_m=float(light.distance_m),
                stop_line_distance_m=(
                    float(light.stop_line_distance_m)
                    if light.stop_line_distance_m is not None
                    else None
                ),
            )
        )
    return traffic_light_states


def _apply_npc_target_speeds(
    *,
    traffic_manager: Any,
    npc_actor_refs: list[Any],
    npc_actor_metadata_by_id: dict[int, dict[str, Any]],
) -> None:
    for actor in npc_actor_refs:
        try:
            actor_id = int(actor.id)
        except RuntimeError as exc:
            if "destroyed actor" in str(exc):
                continue
            raise
        metadata = npc_actor_metadata_by_id.get(actor_id)
        if not metadata:
            continue
        target_speed_kmh = metadata.get("target_speed_kmh")
        if target_speed_kmh is None:
            continue
        try:
            traffic_manager.set_desired_speed(actor, float(target_speed_kmh))
        except RuntimeError as exc:
            if "destroyed actor" in str(exc):
                continue
            raise


def _route_success_criteria(scenario: RouteLoopScenarioSpec) -> dict[str, float | int]:
    return {
        "route_completion_ratio_min": 0.99,
        "collision_count_max": 0,
        "traffic_light_violation_count_max": 0,
        "max_stationary_seconds_max": scenario.max_stop_seconds,
        "goal_tolerance_m_max": scenario.goal_tolerance_m,
        "manual_interventions_max": 0,
    }


def _resolve_route_loop_scenario(
    scenario: RouteLoopScenarioSpec,
    environment_config: EnvironmentConfigSpec | None,
) -> RouteLoopScenarioSpec:
    if environment_config is None:
        return scenario
    return RouteLoopScenarioSpec(
        route_config_path=scenario.route_config_path,
        weather=environment_config.weather or scenario.weather,
        goal_tolerance_m=(
            float(environment_config.goal_tolerance_m)
            if environment_config.goal_tolerance_m is not None
            else scenario.goal_tolerance_m
        ),
        max_stop_seconds=(
            float(environment_config.max_stop_seconds)
            if environment_config.max_stop_seconds is not None
            else scenario.max_stop_seconds
        ),
        stationary_speed_threshold_mps=(
            float(environment_config.stationary_speed_threshold_mps)
            if environment_config.stationary_speed_threshold_mps is not None
            else scenario.stationary_speed_threshold_mps
        ),
        max_seconds=(
            float(environment_config.max_seconds)
            if environment_config.max_seconds is not None
            else scenario.max_seconds
        ),
        environment_config_path=scenario.environment_config_path,
    )


def _route_trace_xyz(
    route_trace: list[tuple[Any, Any]], *, z_offset_m: float = 0.2
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for waypoint, _road_option in route_trace:
        location = waypoint.transform.location
        point = (
            round(float(location.x), 3),
            round(float(location.y), 3),
            round(float(location.z + z_offset_m), 3),
        )
        if not points or point != points[-1]:
            points.append(point)
    return points


def _lane_centerlines_near_route(
    world_map: Any,
    route_trace: list[tuple[Any, Any]],
    *,
    lane_sampling_m: float = 5.0,
    corridor_margin_m: float = 35.0,
    z_offset_m: float = 0.05,
) -> list[list[tuple[float, float, float]]]:
    route_locations = [waypoint.transform.location for waypoint, _ in route_trace]
    if not route_locations:
        return []

    margin = float(corridor_margin_m)
    min_x = min(location.x for location in route_locations) - margin
    max_x = max(location.x for location in route_locations) + margin
    min_y = min(location.y for location in route_locations) - margin
    max_y = max(location.y for location in route_locations) + margin
    max_distance_sq = margin * margin

    grouped: dict[tuple[int, int, int], list[tuple[float, float, float, float]]] = {}
    for waypoint in world_map.generate_waypoints(lane_sampling_m):
        location = waypoint.transform.location
        if location.x < min_x or location.x > max_x or location.y < min_y or location.y > max_y:
            continue
        min_route_distance_sq = inf
        for route_location in route_locations:
            dx = float(location.x - route_location.x)
            dy = float(location.y - route_location.y)
            distance_sq = dx * dx + dy * dy
            if distance_sq < min_route_distance_sq:
                min_route_distance_sq = distance_sq
        if min_route_distance_sq > max_distance_sq:
            continue

        lane_key = (int(waypoint.road_id), int(waypoint.section_id), int(waypoint.lane_id))
        grouped.setdefault(lane_key, []).append(
            (
                float(getattr(waypoint, "s", 0.0)),
                round(float(location.x), 3),
                round(float(location.y), 3),
                round(float(location.z + z_offset_m), 3),
            )
        )

    centerlines: list[list[tuple[float, float, float]]] = []
    for _, samples in sorted(grouped.items()):
        samples.sort(key=lambda item: item[0])
        points: list[tuple[float, float, float]] = []
        for _s, x, y, z in samples:
            point = (x, y, z)
            if not points or point != points[-1]:
                points.append(point)
        if len(points) >= 2:
            centerlines.append(points)
    return centerlines


def _lane_centerlines_all_map(
    world_map: Any,
    *,
    lane_sampling_m: float = 5.0,
    z_offset_m: float = 0.05,
) -> list[list[tuple[float, float, float]]]:
    grouped: dict[tuple[int, int, int], list[tuple[float, float, float, float]]] = {}
    for waypoint in world_map.generate_waypoints(lane_sampling_m):
        location = waypoint.transform.location
        lane_key = (int(waypoint.road_id), int(waypoint.section_id), int(waypoint.lane_id))
        grouped.setdefault(lane_key, []).append(
            (
                float(getattr(waypoint, "s", 0.0)),
                round(float(location.x), 3),
                round(float(location.y), 3),
                round(float(location.z + z_offset_m), 3),
            )
        )

    centerlines: list[list[tuple[float, float, float]]] = []
    for _, samples in sorted(grouped.items()):
        samples.sort(key=lambda item: item[0])
        points: list[tuple[float, float, float]] = []
        for _s, x, y, z in samples:
            point = (x, y, z)
            if not points or point != points[-1]:
                points.append(point)
        if len(points) >= 2:
            centerlines.append(points)
    return centerlines


def _validate_route_loop_request(request: RunRequest) -> RouteLoopScenarioSpec:
    if not isinstance(request.scenario, RouteLoopScenarioSpec):
        raise TypeError(f"Mode {request.mode!r} requires RouteLoopScenarioSpec.")
    return request.scenario


def _validate_interactive_request(request: RunRequest) -> InteractiveScenarioSpec:
    if not isinstance(request.scenario, InteractiveScenarioSpec):
        raise TypeError("Mode 'interactive' requires InteractiveScenarioSpec.")
    return request.scenario


def run(request: RunRequest) -> RunResult:
    if request.mode == "interactive":
        return _run_interactive(request)
    return _run_route_loop(request)


def _run_route_loop(request: RunRequest) -> RunResult:
    carla = _require_carla()
    scenario = _validate_route_loop_request(request)
    runtime = request.runtime
    policy = request.policy
    artifacts = request.artifacts

    if policy.kind == "learned" and policy.checkpoint_path is None:
        raise ValueError("Learned policy requires checkpoint_path.")

    random_seed = random.Random(runtime.seed)
    route_config_path = Path(scenario.route_config_path).resolve()
    route_config = load_route_config(route_config_path)
    environment_config = (
        _load_environment_config(Path(scenario.environment_config_path).resolve())
        if scenario.environment_config_path is not None
        else None
    )
    if environment_config is not None and environment_config.town != route_config.town:
        raise ValueError(
            f"environment_config town mismatch: route={route_config.town} environment={environment_config.town}"
        )
    scenario = _resolve_route_loop_scenario(scenario, environment_config)
    expert_config, expert_config_path = load_expert_config(policy.expert_config_path)
    effective_expert_config = bind_runtime_overrides(
        expert_config,
        target_speed_kmh=runtime.target_speed_kmh,
        ignore_traffic_lights=policy.ignore_traffic_lights,
        ignore_stop_signs=policy.ignore_stop_signs,
        ignore_vehicles=policy.ignore_vehicles,
        sampling_resolution_m=route_config.sampling_resolution_m,
    )
    allow_overtake = bool(effective_expert_config.allow_overtake)

    git_commit_id = ensure_clean_git_worktree(action_label="Evaluate route loop")
    eval_suffix = "pilotnet_eval" if policy.kind == "learned" else "expert_eval"
    episode_id = build_versioned_run_id(
        f"{route_config.name}_{eval_suffix}", commit_id=git_commit_id
    )
    episode_dir = PROJECT_ROOT / "outputs" / "evaluate" / episode_id
    manifest_path = episode_dir / "manifest.jsonl"
    summary_path = episode_dir / "summary.json"
    mcap_dir = episode_dir / "telemetry"
    mcap_index_path = mcap_dir / "index.json"
    episode_dir.mkdir(parents=True, exist_ok=True)
    temp_image_dir_handle: tempfile.TemporaryDirectory[str] | None = None
    image_dir: Path | None = None
    if artifacts.record_video:
        temp_image_dir_handle = tempfile.TemporaryDirectory(prefix="carla_alpamayo_front_rgb_")
        image_dir = Path(temp_image_dir_handle.name)

    client = carla.Client(runtime.host, runtime.port)
    client.set_timeout(30.0)
    world, original_settings = setup_world(client, route_config.town, runtime.fixed_delta_seconds)
    world.set_weather(_resolve_weather(carla, scenario.weather))

    frame_events = FrameEventTracker()
    image_queue: queue.Queue[Any] = queue.Queue()
    actors: list[Any] = []

    elapsed_seconds = 0.0
    frame_index = 0
    recorded_frame_index = 0
    max_stationary_seconds = 0.0
    current_stationary_seconds = 0.0
    average_speed_mps = 0.0
    speed_sum = 0.0
    max_completion_ratio = 0.0
    current_completion_ratio = 0.0
    success = False
    failure_reason = ""
    distance_to_goal_m = 0.0
    video_path: Path | None = None
    video_error: str | None = None
    mcap_error: str | None = None
    mcap_writer: RotatingRouteLoopMcapWriter | None = None
    traffic_light_stop_count = 0
    traffic_light_resume_count = 0
    traffic_light_violation_count = 0
    car_follow_event_count = 0
    overtake_attempt_count = 0
    overtake_success_count = 0
    overtake_abort_count = 0
    unsafe_lane_change_reject_count = 0
    min_ttc = float("inf")
    min_lead_distance_m = float("inf")
    _traffic_light_violation_active = False
    npc_actors_summary: list[dict[str, Any]] = []
    npc_actor_refs: list[Any] = []
    traffic_light_phase_indices: dict[int, int] = {}
    traffic_light_runtime_groups: list[TrafficLightPhaseRuntimeGroup] = []
    controlled_cycle_actor_ids: set[int] = set()
    explicit_traffic_light_actor_ids = {
        override.actor_id
        for override in (
            environment_config.traffic_light_overrides if environment_config is not None else []
        )
    } | {
        schedule.actor_id
        for schedule in (
            environment_config.traffic_light_schedules if environment_config is not None else []
        )
    }
    traffic_light_cycle_states: dict[int, str] = {}

    planned_route = build_planned_route(world.get_map(), route_config)
    topdown_map_asset = load_default_topdown_map_asset(route_config.town)
    route_geometry = route_geometry_from_planned_route(planned_route)
    goal_location = planned_route.trace[-1][0].transform.location
    success_criteria = _route_success_criteria(scenario)
    record_period_s = (
        runtime.fixed_delta_seconds
        if artifacts.record_hz <= 0
        else max(runtime.fixed_delta_seconds, 1.0 / artifacts.record_hz)
    )
    next_record_elapsed_s = runtime.fixed_delta_seconds

    try:
        if artifacts.record_mcap:
            mcap_writer = RotatingRouteLoopMcapWriter(
                root_dir=mcap_dir,
                episode_id=episode_id,
                route_name=route_config.name,
                town=route_config.town,
                weather=scenario.weather,
                camera_width=runtime.camera_width,
                camera_height=runtime.camera_height,
                jpeg_quality=artifacts.mcap_jpeg_quality,
                segment_seconds=artifacts.mcap_segment_seconds,
                topdown_map_asset=topdown_map_asset,
            )

        vehicle_blueprint = require_blueprint(world, runtime.vehicle_filter, rng=random_seed)
        vehicle_blueprint.set_attribute("role_name", "hero")
        vehicle = world.try_spawn_actor(vehicle_blueprint, planned_route.anchor_transforms[0])
        if vehicle is None:
            raise RuntimeError("Failed to spawn ego vehicle at the first route anchor.")
        actors.append(vehicle)

        camera_blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
        camera_blueprint.set_attribute("image_size_x", str(runtime.camera_width))
        camera_blueprint.set_attribute("image_size_y", str(runtime.camera_height))
        camera_blueprint.set_attribute("fov", str(runtime.camera_fov))
        camera_transform = carla.Transform(carla.Location(x=1.5, z=1.5))
        camera = world.spawn_actor(camera_blueprint, camera_transform, attach_to=vehicle)
        actors.append(camera)
        camera.listen(image_queue.put)

        collision_sensor = attach_sensor(
            world, "sensor.other.collision", carla.Transform(), vehicle
        )
        actors.append(collision_sensor)

        def _handle_collision_event(event: Any) -> None:
            try:
                other_actor = getattr(event, "other_actor", None)
            except RuntimeError as exc:
                if "destroyed actor" in str(exc):
                    other_actor = None
                else:
                    raise
            frame_events.mark_collision(other_actor)

        collision_sensor.listen(_handle_collision_event)

        lane_sensor = attach_sensor(world, "sensor.other.lane_invasion", carla.Transform(), vehicle)
        actors.append(lane_sensor)
        lane_sensor.listen(lambda _event: frame_events.mark_lane_invasion())

        lights_by_id, traffic_light_runtime_groups = _apply_traffic_light_group_cycle(
            world,
            environment_config.traffic_light_phase_cycle
            if environment_config is not None
            else None,
        )
        controlled_cycle_actor_ids = {
            actor_id
            for runtime_group in traffic_light_runtime_groups
            for actor_id in runtime_group.actor_ids
        }
        _apply_derived_traffic_light_group_cycle(
            carla,
            lights_by_id,
            traffic_light_runtime_groups,
            environment_config.traffic_light_phase_cycle
            if environment_config is not None
            else None,
            elapsed_seconds=0.0,
            applied_states=traffic_light_cycle_states,
            excluded_actor_ids=explicit_traffic_light_actor_ids,
        )
        lights_by_id = _apply_traffic_light_overrides(
            carla,
            world,
            environment_config.traffic_light_overrides if environment_config is not None else [],
        )
        _apply_traffic_light_schedules(
            carla,
            lights_by_id,
            environment_config.traffic_light_schedules if environment_config is not None else [],
            elapsed_seconds=0.0,
            applied_phase_indices=traffic_light_phase_indices,
        )
        npc_actors_summary = _spawn_npc_vehicles(
            carla_module=carla,
            client=client,
            world=world,
            runtime=runtime,
            environment_config=environment_config,
            rng=random_seed,
            actors=actors,
            spawned_actor_refs=npc_actor_refs,
        )
        npc_actor_metadata_by_id = {int(item["actor_id"]): item for item in npc_actors_summary}
        traffic_manager = client.get_trafficmanager(8000)

        if policy.kind == "expert":
            stack = create_expert_collector_stack(
                vehicle=vehicle,
                world_map=world.get_map(),
                planned_trace=planned_route.trace,
                route_id=route_config.name,
                town_id=route_config.town,
                target_speed_kmh=effective_expert_config.target_speed_kmh,
                ignore_traffic_lights=policy.ignore_traffic_lights,
                ignore_stop_signs=policy.ignore_stop_signs,
                ignore_vehicles=policy.ignore_vehicles,
                sampling_resolution_m=route_config.sampling_resolution_m,
                route_geometry=route_geometry,
                expert_config=effective_expert_config,
            )
            stack_description = stack.describe()
        else:
            stack = create_pilotnet_eval_stack(
                vehicle=vehicle,
                world_map=world.get_map(),
                planned_trace=planned_route.trace,
                route_id=route_config.name,
                town_id=route_config.town,
                target_speed_kmh=effective_expert_config.target_speed_kmh,
                checkpoint_path=Path(policy.checkpoint_path).resolve(),
                device=policy.device,
                ignore_traffic_lights=policy.ignore_traffic_lights,
                ignore_stop_signs=policy.ignore_stop_signs,
                ignore_vehicles=policy.ignore_vehicles,
                sampling_resolution_m=route_config.sampling_resolution_m,
                route_geometry=route_geometry,
                expert_config=effective_expert_config,
            )
            stack_description = stack.describe()

        world_frame = world.tick()
        current_image = wait_for_image(image_queue, world_frame, runtime.sensor_timeout)
        if mcap_writer is not None:
            lane_centerlines = (
                _lane_centerlines_all_map(world.get_map())
                if artifacts.mcap_map_scope == "full"
                else _lane_centerlines_near_route(world.get_map(), planned_route.trace)
            )
            mcap_writer.write_static_scene(
                timestamp_s=current_image.timestamp,
                route_name=route_config.name,
                route_points=_route_trace_xyz(planned_route.trace),
                lane_centerlines=lane_centerlines,
            )
        preview_sink = policy.preview_sink
        while True:
            _apply_npc_target_speeds(
                traffic_manager=traffic_manager,
                npc_actor_refs=npc_actor_refs,
                npc_actor_metadata_by_id=npc_actor_metadata_by_id,
            )
            _apply_derived_traffic_light_group_cycle(
                carla,
                lights_by_id,
                traffic_light_runtime_groups,
                environment_config.traffic_light_phase_cycle
                if environment_config is not None
                else None,
                elapsed_seconds=elapsed_seconds,
                applied_states=traffic_light_cycle_states,
                excluded_actor_ids=explicit_traffic_light_actor_ids,
            )
            _apply_traffic_light_schedules(
                carla,
                lights_by_id,
                environment_config.traffic_light_schedules
                if environment_config is not None
                else [],
                elapsed_seconds=elapsed_seconds,
                applied_phase_indices=traffic_light_phase_indices,
            )
            current_speed = speed_mps(vehicle)
            vehicle_transform = vehicle.get_transform()
            vehicle_location = vehicle_transform.location
            current_rgb = (
                _carla_image_to_rgb_array(current_image) if policy.kind == "learned" else None
            )

            if policy.kind == "learned":
                step_result = stack.run_step(
                    timestamp_s=elapsed_seconds,
                    vehicle_transform=vehicle_transform,
                    speed_mps=current_speed,
                    current_rgb=current_rgb,
                    steering_smoothing=policy.steer_smoothing,
                    max_steer_delta=policy.max_steer_delta,
                )
            else:
                step_result = stack.run_step(
                    timestamp_s=elapsed_seconds,
                    vehicle_transform=vehicle_transform,
                    speed_mps=current_speed,
                )

            decision = step_result.decision
            control = to_carla_control(carla, decision.command)
            vehicle.apply_control(control)

            collision, lane_invasion = frame_events.consume_frame_flags()
            speed_sum += current_speed
            frame_index += 1
            elapsed_seconds = frame_index * runtime.fixed_delta_seconds
            average_speed_mps = speed_sum / frame_index

            if current_speed < scenario.stationary_speed_threshold_mps:
                current_stationary_seconds += runtime.fixed_delta_seconds
            else:
                current_stationary_seconds = 0.0
            max_stationary_seconds = max(max_stationary_seconds, current_stationary_seconds)

            current_completion_ratio = step_result.progress_ratio
            max_completion_ratio = max(max_completion_ratio, current_completion_ratio)
            distance_to_goal_m = vehicle_location.distance(goal_location)
            route_point = step_result.route_point
            behavior = step_result.behavior or decision.behavior
            planning_decision = step_result.expert_decision or decision
            planning_debug = dict(planning_decision.debug or {})
            should_record = elapsed_seconds + 1e-9 >= next_record_elapsed_s

            traffic_light_stop_count += int(bool(planning_debug.get("event_traffic_light_stop")))
            traffic_light_resume_count += int(
                bool(planning_debug.get("event_traffic_light_resume"))
            )
            car_follow_event_count += int(bool(planning_debug.get("event_car_follow_start")))
            overtake_attempt_count += int(bool(planning_debug.get("event_overtake_attempt")))
            overtake_success_count += int(bool(planning_debug.get("event_overtake_success")))
            overtake_abort_count += int(bool(planning_debug.get("event_overtake_abort")))
            unsafe_lane_change_reject_count += int(
                bool(planning_debug.get("event_unsafe_lane_change_reject"))
            )
            if (
                bool(planning_debug.get("traffic_light_violation"))
                and not _traffic_light_violation_active
            ):
                traffic_light_violation_count += 1
                _traffic_light_violation_active = True
            elif not bool(planning_debug.get("traffic_light_violation")):
                _traffic_light_violation_active = False
            if planning_debug.get("min_ttc") is not None:
                min_ttc = min(min_ttc, float(planning_debug["min_ttc"]))
            if planning_debug.get("lead_vehicle_distance_m") is not None:
                min_lead_distance_m = min(
                    min_lead_distance_m, float(planning_debug["lead_vehicle_distance_m"])
                )

            if should_record:
                if current_rgb is None:
                    current_rgb = _carla_image_to_rgb_array(current_image)

                image_path: Path | None = None
                if artifacts.record_video and image_dir is not None:
                    image_path = image_dir / f"{recorded_frame_index:06d}.png"
                    current_image.save_to_disk(str(image_path))

                mcap_segment_index: int | None = None
                mcap_segment_path: str | None = None

                if mcap_writer is not None:
                    npc_vehicle_states = _collect_npc_vehicle_states(
                        npc_actor_refs=npc_actor_refs,
                        npc_actor_metadata_by_id=npc_actor_metadata_by_id,
                        world_map=world.get_map(),
                    )
                    tracked_vehicle_states = _collect_tracked_vehicle_states(
                        scene_state=step_result.scene_state
                    )
                    traffic_light_states = _collect_traffic_light_states(
                        scene_state=step_result.scene_state
                    )
                    planning_debug_mcap = {
                        key: value
                        for key, value in planning_debug.items()
                        if key
                        not in {
                            "planner_state",
                            "traffic_light_state",
                            "overtake_state",
                            "target_lane_id",
                            "min_ttc",
                        }
                    }
                    mcap_segment = mcap_writer.write_frame(
                        current_rgb=current_rgb,
                        ego_state=EgoStateSample(
                            episode_id=episode_id,
                            frame_id=frame_index - 1,
                            timestamp_s=current_image.timestamp,
                            elapsed_seconds=elapsed_seconds,
                            speed_mps=current_speed,
                            behavior=behavior,
                            route_completion_ratio=current_completion_ratio,
                            distance_to_goal_m=distance_to_goal_m,
                            planner_state=planning_decision.planner_state,
                            traffic_light_state=planning_debug.get("traffic_light_state"),
                            lead_vehicle_distance_m=planning_debug.get("lead_vehicle_distance_m"),
                            overtake_state=planning_debug.get("overtake_state"),
                            target_lane_id=planning_debug.get("target_lane_id"),
                            min_ttc=planning_debug.get("min_ttc"),
                            pose={
                                "x": float(vehicle_location.x),
                                "y": float(vehicle_location.y),
                                "z": float(vehicle_location.z),
                                "yaw_deg": float(vehicle_transform.rotation.yaw),
                                "pitch_deg": float(vehicle_transform.rotation.pitch),
                                "roll_deg": float(vehicle_transform.rotation.roll),
                            },
                            control={
                                "steer": float(decision.command.steer),
                                "throttle": float(decision.command.throttle),
                                "brake": float(decision.command.brake),
                            },
                            planning_debug=planning_debug_mcap,
                        ),
                        npc_vehicle_states=npc_vehicle_states,
                        tracked_vehicle_states=tracked_vehicle_states,
                        traffic_light_states=traffic_light_states,
                    )
                    mcap_segment_index = mcap_segment.segment_index
                    mcap_segment_path = relative_to_project(mcap_segment.path)

                record = EpisodeRecord(
                    episode_id=episode_id,
                    frame_id=frame_index - 1,
                    town_id=route_config.town,
                    route_id=route_config.name,
                    weather_id=scenario.weather,
                    timestamp=current_image.timestamp,
                    front_rgb_path=None,
                    speed=current_speed,
                    command=behavior,
                    steer=decision.command.steer,
                    throttle=decision.command.throttle,
                    brake=decision.command.brake,
                    collision=collision,
                    lane_invasion=lane_invasion,
                    success=not collision and not failure_reason,
                    vehicle_x=vehicle_location.x,
                    vehicle_y=vehicle_location.y,
                    vehicle_z=vehicle_location.z,
                    vehicle_yaw_deg=vehicle_transform.rotation.yaw,
                    route_completion_ratio=current_completion_ratio,
                    distance_to_goal_m=distance_to_goal_m,
                    expert_steer=(
                        step_result.expert_decision.command.steer
                        if step_result.expert_decision is not None
                        else None
                    ),
                    route_target_x=route_point[0] if route_point is not None else None,
                    route_target_y=route_point[1] if route_point is not None else None,
                    planner_state=planning_decision.planner_state,
                    traffic_light_state=planning_debug.get("traffic_light_state"),
                    traffic_light_actor_id=planning_debug.get("traffic_light_actor_id"),
                    traffic_light_distance_m=planning_debug.get("traffic_light_distance_m"),
                    traffic_light_stop_line_distance_m=planning_debug.get(
                        "traffic_light_stop_line_distance_m"
                    ),
                    traffic_light_violation=planning_debug.get("traffic_light_violation"),
                    lead_vehicle_distance_m=planning_debug.get("lead_vehicle_distance_m"),
                    lead_vehicle_id=planning_debug.get("lead_vehicle_id"),
                    lead_vehicle_speed_mps=planning_debug.get("lead_vehicle_speed_mps"),
                    lead_vehicle_relative_speed_mps=planning_debug.get(
                        "lead_vehicle_relative_speed_mps"
                    ),
                    lead_vehicle_lane_id=planning_debug.get("lead_vehicle_lane_id"),
                    left_lane_front_gap_m=planning_debug.get("left_lane_front_gap_m"),
                    left_lane_rear_gap_m=planning_debug.get("left_lane_rear_gap_m"),
                    right_lane_front_gap_m=planning_debug.get("right_lane_front_gap_m"),
                    right_lane_rear_gap_m=planning_debug.get("right_lane_rear_gap_m"),
                    overtake_state=planning_debug.get("overtake_state"),
                    overtake_considered=planning_debug.get("overtake_considered"),
                    overtake_direction=planning_debug.get("overtake_direction"),
                    overtake_reject_reason=planning_debug.get("overtake_reject_reason"),
                    overtake_target_lane_id=planning_debug.get("overtake_target_lane_id"),
                    current_lane_id=planning_debug.get("current_lane_id"),
                    route_target_lane_id=planning_debug.get("route_target_lane_id"),
                    target_lane_id=planning_debug.get("target_lane_id"),
                    target_speed_kmh=planning_debug.get("target_speed_kmh"),
                    emergency_stop=planning_debug.get("emergency_stop"),
                    min_ttc=planning_debug.get("min_ttc"),
                    mcap_segment_index=mcap_segment_index,
                    mcap_segment_path=mcap_segment_path,
                )
                append_jsonl(manifest_path, record)

                recorded_frame_index += 1
                while next_record_elapsed_s <= elapsed_seconds + 1e-9:
                    next_record_elapsed_s += record_period_s

            if preview_sink is not None:
                if current_rgb is None:
                    current_rgb = _carla_image_to_rgb_array(current_image)
                behavior_label = behavior or "unknown"
                status_core = (
                    f"policy={policy.kind:<7} behavior={behavior_label:<12} "
                    f"speed={current_speed * 3.6:5.1f} km/h progress={current_completion_ratio:5.3f} "
                    f"collisions={frame_events.collision_count} lane={frame_events.lane_invasion_count}"
                )
                keep_preview = preview_sink(current_rgb, status_core)
                if keep_preview is False:
                    preview_sink = None

            if collision:
                failure_reason = "collision"
                break

            reached_success_goal = (
                current_completion_ratio >= success_criteria["route_completion_ratio_min"]
                and distance_to_goal_m <= success_criteria["goal_tolerance_m_max"]
            )
            if reached_success_goal:
                break

            if max_stationary_seconds >= scenario.max_stop_seconds:
                failure_reason = "stalled"
                break

            if step_result.done:
                break

            if elapsed_seconds >= scenario.max_seconds:
                failure_reason = "max_seconds_exceeded"
                break

            world_frame = world.tick()
            current_image = wait_for_image(image_queue, world_frame, runtime.sensor_timeout)

        max_completion_ratio = max(max_completion_ratio, current_completion_ratio)
        distance_to_goal_m = vehicle.get_location().distance(goal_location)
        success = (
            not failure_reason
            and max_completion_ratio >= success_criteria["route_completion_ratio_min"]
            and frame_events.collision_count <= success_criteria["collision_count_max"]
            and traffic_light_violation_count
            <= success_criteria["traffic_light_violation_count_max"]
            and max_stationary_seconds < success_criteria["max_stationary_seconds_max"]
            and distance_to_goal_m <= success_criteria["goal_tolerance_m_max"]
        )
        if not success and not failure_reason:
            if max_completion_ratio < success_criteria["route_completion_ratio_min"]:
                failure_reason = "route_incomplete"
            elif (
                traffic_light_violation_count
                > success_criteria["traffic_light_violation_count_max"]
            ):
                failure_reason = "traffic_light_violation"
            elif distance_to_goal_m > success_criteria["goal_tolerance_m_max"]:
                failure_reason = "goal_tolerance_exceeded"
            else:
                failure_reason = "success_criteria_failed"
        if mcap_writer is not None:
            mcap_writer.close()
    finally:
        if mcap_writer is not None:
            try:
                mcap_writer.close()
            except Exception as exc:  # pragma: no cover - depends on runtime environment
                mcap_error = str(exc)
        if environment_config is not None:
            managed_traffic_light_actor_ids = (
                controlled_cycle_actor_ids | explicit_traffic_light_actor_ids
            )
            for actor in world.get_actors().filter("*traffic_light*"):
                if int(actor.id) in managed_traffic_light_actor_ids:
                    actor.freeze(False)
        client.get_trafficmanager(8000).set_synchronous_mode(False)
        world.apply_settings(original_settings)
        destroy_actors(reversed(actors))

    video_fps = artifacts.video_fps or artifacts.record_hz
    if artifacts.record_video and image_dir is not None:
        try:
            video_path = render_png_sequence_to_mp4(
                image_dir=image_dir,
                output_path=episode_dir / "front_rgb.mp4",
                fps=video_fps,
                crf=artifacts.video_crf,
            )
        except Exception as exc:  # pragma: no cover - depends on ffmpeg/runtime env
            video_error = str(exc)
        finally:
            if temp_image_dir_handle is not None:
                temp_image_dir_handle.cleanup()
                temp_image_dir_handle = None

    mcap_segments = []
    if artifacts.record_mcap and mcap_writer is not None:
        mcap_segments = [
            {
                "segment_index": segment.segment_index,
                "path": relative_to_project(segment.path),
                "start_elapsed_seconds": round(segment.start_elapsed_seconds, 3),
                "end_elapsed_seconds": (
                    round(segment.end_elapsed_seconds, 3)
                    if segment.end_elapsed_seconds is not None
                    else None
                ),
                "frame_count": segment.frame_count,
            }
            for segment in mcap_writer.segments
        ]
    first_mcap_segment_path = mcap_segments[0]["path"] if mcap_segments else None

    base_summary = {
        "episode_id": episode_id,
        "route_name": route_config.name,
        "route_config_path": relative_to_project(route_config_path),
        "environment_config_path": _relative_or_none(
            Path(scenario.environment_config_path).resolve()
            if scenario.environment_config_path
            else None
        ),
        "environment_config_name": environment_config.name
        if environment_config is not None
        else None,
        "expert_config_path": relative_to_project(expert_config_path),
        "expert_config": expert_config_to_dict(effective_expert_config),
        "npc_vehicle_count": len(npc_actors_summary),
        "npc_vehicles": npc_actors_summary,
        "town": route_config.town,
        "weather": scenario.weather,
        "frame_count": frame_index,
        "recorded_frame_count": recorded_frame_index,
        "elapsed_seconds": round(elapsed_seconds, 2),
        "lap_time_seconds": round(elapsed_seconds, 2),
        "target_speed_kmh": effective_expert_config.target_speed_kmh,
        "camera_width": runtime.camera_width,
        "camera_height": runtime.camera_height,
        "record_hz": artifacts.record_hz,
        "average_speed_kmh": round(average_speed_mps * 3.6, 2),
        "collision_count": frame_events.collision_count,
        "last_collision_actor_id": frame_events.last_collision_actor_id,
        "last_collision_actor_type_id": frame_events.last_collision_actor_type_id,
        "lane_invasion_count": frame_events.lane_invasion_count,
        "traffic_light_stop_count": traffic_light_stop_count,
        "traffic_light_resume_count": traffic_light_resume_count,
        "traffic_light_violation_count": traffic_light_violation_count,
        "car_follow_event_count": car_follow_event_count,
        "overtake_attempt_count": overtake_attempt_count,
        "overtake_success_count": overtake_success_count,
        "overtake_abort_count": overtake_abort_count,
        "unsafe_lane_change_reject_count": unsafe_lane_change_reject_count,
        "min_ttc": None if not min_ttc < float("inf") else round(min_ttc, 3),
        "min_lead_distance_m": None
        if not min_lead_distance_m < float("inf")
        else round(min_lead_distance_m, 3),
        "allow_overtake": allow_overtake,
        "max_stationary_seconds": round(max_stationary_seconds, 2),
        "route_completion_ratio": round(max_completion_ratio, 4),
        "distance_to_goal_m": round(distance_to_goal_m, 2),
        "goal_tolerance_m": scenario.goal_tolerance_m,
        "manual_interventions": 0,
        "success": success,
        "failure_reason": failure_reason or None,
        "success_criteria": success_criteria,
        "segment_summaries": planned_route.segment_summaries,
        "road_option_counts": planned_route.road_option_counts,
        "front_rgb_dir": None,
        "video_path": _relative_or_none(video_path),
        "video_fps": round(video_fps, 3) if video_path is not None else None,
        "video_crf": artifacts.video_crf if video_path is not None else None,
        "video_error": video_error,
        "mcap_path": first_mcap_segment_path,
        "mcap_dir": _relative_or_none(mcap_dir if artifacts.record_mcap else None),
        "mcap_index_path": _relative_or_none(mcap_index_path if artifacts.record_mcap else None),
        "mcap_segment_seconds": artifacts.mcap_segment_seconds if artifacts.record_mcap else None,
        "topdown_map_asset_path": (
            _relative_or_none(topdown_map_asset.image_path)
            if artifacts.record_mcap and topdown_map_asset is not None
            else None
        ),
        "topdown_map_metadata_path": (
            _relative_or_none(topdown_map_asset.metadata_path)
            if artifacts.record_mcap and topdown_map_asset is not None
            else None
        ),
        "mcap_segments": mcap_segments,
        "mcap_error": mcap_error,
        "manifest_path": relative_to_project(manifest_path),
    }

    if policy.kind == "expert":
        summary = {
            **base_summary,
            "policy_type": "expert_route_policy",
            "control_source": "ad_stack.run(expert_evaluate)",
            "route_intent_source": "global_route_planner",
            "lateral_control_source": "carla_local_planner_pid_via_ad_stack_expert_route_policy",
            "is_camera_e2e_policy": False,
            "git_commit_id": git_commit_id,
        }
    else:
        summary = {
            **base_summary,
            "policy_type": "learned_lateral_policy",
            "control_source": "ad_stack.run(learned_evaluate)",
            "route_intent_source": "global_route_planner",
            "lateral_control_source": "pilotnet_learning_runtime_via_ad_stack",
            "longitudinal_control_source": "ad_stack_expert_route_policy_longitudinal",
            "is_camera_e2e_policy": True,
            "e2e_scope": "front_rgb_plus_speed_to_steer",
            "model_checkpoint_path": _relative_or_none(
                Path(policy.checkpoint_path).resolve() if policy.checkpoint_path else None
            ),
            "model_name": stack_description.model_name,
            "git_commit_id": git_commit_id,
            "steer_smoothing": policy.steer_smoothing,
            "max_steer_delta": policy.max_steer_delta,
            "camera_width": runtime.camera_width,
            "camera_height": runtime.camera_height,
            "camera_fov": runtime.camera_fov,
        }

    with summary_path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
        handle.write("\n")

    return RunResult(
        mode=request.mode,
        success=success,
        summary=summary,
        episode_id=episode_id,
        output_dir=episode_dir,
        summary_path=summary_path,
        manifest_path=manifest_path,
        video_path=video_path,
        mcap_path=(
            Path(PROJECT_ROOT / first_mcap_segment_path).resolve()
            if first_mcap_segment_path is not None
            else None
        ),
        frame_count=frame_index,
        elapsed_seconds=elapsed_seconds,
    )


def _run_interactive(request: RunRequest) -> RunResult:
    carla = _require_carla()
    scenario = _validate_interactive_request(request)
    runtime = request.runtime
    policy = request.policy

    if policy.kind != "interactive":
        raise ValueError("Interactive mode currently requires policy.kind='interactive'.")
    if policy.checkpoint_path is None:
        raise ValueError("Interactive mode requires checkpoint_path.")

    client = carla.Client(runtime.host, runtime.port)
    client.set_timeout(30.0)
    world, original_settings = setup_world(client, scenario.town, runtime.fixed_delta_seconds)
    world.set_weather(_resolve_weather(carla, scenario.weather))

    frame_events = FrameEventTracker()
    image_queue: queue.Queue[Any] = queue.Queue()
    actors: list[Any] = []
    frame_index = 0
    elapsed_seconds = 0.0
    current_command = policy.initial_command

    controller = create_interactive_pilotnet_controller(
        checkpoint_path=Path(policy.checkpoint_path).resolve(),
        device=policy.device,
        target_speed_kmh=runtime.target_speed_kmh,
    )
    description = controller.describe()

    try:
        spawn_points = world.get_map().get_spawn_points()
        if scenario.spawn_index < 0 or scenario.spawn_index >= len(spawn_points):
            raise SystemExit(f"--spawn-index must be within [0, {len(spawn_points) - 1}]")

        vehicle_blueprint = require_blueprint(world, runtime.vehicle_filter)
        vehicle_blueprint.set_attribute("role_name", "hero")
        vehicle = world.try_spawn_actor(vehicle_blueprint, spawn_points[scenario.spawn_index])
        if vehicle is None:
            raise RuntimeError(
                f"Failed to spawn ego vehicle at spawn index {scenario.spawn_index}."
            )
        actors.append(vehicle)

        camera_blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
        camera_blueprint.set_attribute("image_size_x", str(runtime.camera_width))
        camera_blueprint.set_attribute("image_size_y", str(runtime.camera_height))
        camera_blueprint.set_attribute("fov", str(runtime.camera_fov))
        camera_transform = carla.Transform(carla.Location(x=1.5, z=1.5))
        camera = world.spawn_actor(camera_blueprint, camera_transform, attach_to=vehicle)
        actors.append(camera)
        camera.listen(image_queue.put)

        collision_sensor = attach_sensor(
            world, "sensor.other.collision", carla.Transform(), vehicle
        )
        actors.append(collision_sensor)
        collision_sensor.listen(lambda _event: frame_events.mark_collision())

        lane_sensor = attach_sensor(world, "sensor.other.lane_invasion", carla.Transform(), vehicle)
        actors.append(lane_sensor)
        lane_sensor.listen(lambda _event: frame_events.mark_lane_invasion())

        world_frame = world.tick()
        current_image = wait_for_image(image_queue, world_frame, runtime.sensor_timeout)
        spectator = world.get_spectator()

        while elapsed_seconds < scenario.max_seconds:
            if policy.command_provider is not None:
                current_command, should_quit = policy.command_provider(current_command)
                if should_quit:
                    break

            current_rgb = _carla_image_to_rgb_array(current_image)
            current_speed = speed_mps(vehicle)
            step_result = controller.run_step(
                current_rgb=current_rgb,
                speed_mps=current_speed,
                command=current_command,
                steering_smoothing=policy.steer_smoothing,
                max_steer_delta=policy.max_steer_delta,
            )
            predicted_steer = (
                step_result.applied_steer
                if step_result.applied_steer is not None
                else step_result.decision.command.steer
            )
            control = to_carla_control(carla, step_result.decision.command)
            vehicle.apply_control(control)
            _update_spectator(
                carla,
                spectator,
                vehicle.get_transform(),
                follow_distance_m=scenario.spectator_follow_distance_m,
                height_m=scenario.spectator_height_m,
            )

            collision, lane_invasion = frame_events.consume_frame_flags()
            frame_index += 1
            elapsed_seconds = frame_index * runtime.fixed_delta_seconds
            status_core = (
                f"cmd={current_command:<10} speed={current_speed * 3.6:5.1f} km/h "
                f"steer={predicted_steer:+.3f} throttle={control.throttle:.2f} brake={control.brake:.2f} "
                f"collisions={frame_events.collision_count} lane={frame_events.lane_invasion_count}"
            )
            if policy.status_sink is not None:
                policy.status_sink(status_core)
            if policy.preview_sink is not None:
                keep_running = policy.preview_sink(current_rgb, status_core)
                if keep_running is False:
                    break

            world_frame = world.tick()
            current_image = wait_for_image(image_queue, world_frame, runtime.sensor_timeout)
    finally:
        world.apply_settings(original_settings)
        destroy_actors(reversed(actors))

    summary = {
        "mode": "interactive",
        "town": scenario.town,
        "spawn_index": scenario.spawn_index,
        "weather": scenario.weather,
        "checkpoint_path": _relative_or_none(Path(policy.checkpoint_path).resolve()),
        "model_name": description.model_name,
        "target_speed_kmh": runtime.target_speed_kmh,
        "frame_count": frame_index,
        "elapsed_seconds": round(elapsed_seconds, 2),
        "collision_count": frame_events.collision_count,
        "lane_invasion_count": frame_events.lane_invasion_count,
        "success": frame_events.collision_count == 0,
    }
    return RunResult(
        mode="interactive",
        success=bool(summary["success"]),
        summary=summary,
        frame_count=frame_index,
        elapsed_seconds=elapsed_seconds,
    )


__all__ = [
    "ArtifactSpec",
    "InteractiveScenarioSpec",
    "PolicySpec",
    "RouteLoopScenarioSpec",
    "RunRequest",
    "RunResult",
    "RuntimeSpec",
    "run",
]
