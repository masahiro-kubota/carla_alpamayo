from __future__ import annotations

from dataclasses import dataclass, field
import json
from pathlib import Path
import queue
import random
from typing import Any, Callable, Literal

from ad_stack.api import (
    create_expert_collector_stack,
    create_interactive_pilotnet_controller,
    create_pilotnet_eval_stack,
    to_carla_control,
)
from libs.carla_utils import (
    FrameEventTracker,
    attach_sensor,
    build_episode_id,
    build_planned_route,
    destroy_actors,
    load_route_config,
    relative_to_project,
    require_blueprint,
    route_geometry_from_planned_route,
    setup_world,
    speed_mps,
    wait_for_image,
)
from libs.project import PROJECT_ROOT, build_versioned_run_id, ensure_clean_git_worktree
from libs.schemas import EpisodeRecord, append_jsonl
from libs.utils import render_png_sequence_to_mp4

RunMode = Literal["collect", "evaluate", "interactive"]
PolicyKind = Literal["expert", "learned", "interactive"]
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
    traffic_setup_path: Path | None = None


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
class TrafficSetupSpec:
    name: str
    town: str
    npc_vehicles: list[NPCVehicleSpec] = field(default_factory=list)
    traffic_light_overrides: list[TrafficLightOverrideSpec] = field(default_factory=list)
    traffic_light_schedules: list[TrafficLightScheduleSpec] = field(default_factory=list)
    expert_overrides: dict[str, Any] = field(default_factory=dict)
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
    camera_width: int = 320
    camera_height: int = 180
    camera_fov: int = 90
    target_speed_kmh: float = 30.0
    seed: int = 7


@dataclass(slots=True)
class ArtifactSpec:
    record_video: bool = True
    video_crf: int = 23
    video_fps: float | None = None


@dataclass(slots=True)
class PolicySpec:
    kind: PolicyKind
    checkpoint_path: Path | None = None
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


def _load_traffic_setup(path: Path) -> TrafficSetupSpec:
    raw = _load_json(path)
    return TrafficSetupSpec(
        name=str(raw["name"]),
        town=str(raw["town"]),
        npc_vehicles=[
            NPCVehicleSpec(
                spawn_index=int(item["spawn_index"]),
                npc_profile_id=str(item["npc_profile_id"]) if item.get("npc_profile_id") else None,
                target_speed_kmh=float(item["target_speed_kmh"]) if item.get("target_speed_kmh") is not None else None,
                lane_behavior=str(item.get("lane_behavior", "keep_lane")),
                vehicle_filter=str(item.get("vehicle_filter", "vehicle.*")),
            )
            for item in raw.get("npc_vehicles", [])
        ],
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
        expert_overrides=dict(raw.get("expert_overrides", {})),
        description=str(raw.get("description", "")),
    )


def _resolved_npc_speed_kmh(spec: NPCVehicleSpec, profile: NPCProfileSpec | None, rng: random.Random) -> float:
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


def _set_traffic_light_state(carla_module: Any, traffic_light: Any, *, state_name: str, freeze: bool) -> None:
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
            raise RuntimeError(f"Traffic light actor_id={override.actor_id} not found for override.")
        _set_traffic_light_state(
            carla_module,
            traffic_light,
            state_name=override.state,
            freeze=override.freeze,
        )
    return lights_by_id


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
            raise RuntimeError(f"Traffic light actor_id={schedule.actor_id} not found for schedule.")
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
    client: Any,
    world: Any,
    runtime: RuntimeSpec,
    traffic_setup: TrafficSetupSpec | None,
    rng: random.Random,
    actors: list[Any],
) -> list[dict[str, Any]]:
    if traffic_setup is None or not traffic_setup.npc_vehicles:
        return []

    traffic_manager = client.get_trafficmanager(8000)
    traffic_manager.set_synchronous_mode(True)
    traffic_manager.set_global_distance_to_leading_vehicle(1.5)
    spawn_points = world.get_map().get_spawn_points()
    spawned: list[dict[str, Any]] = []
    for npc_spec in traffic_setup.npc_vehicles:
        if npc_spec.spawn_index < 0 or npc_spec.spawn_index >= len(spawn_points):
            raise RuntimeError(f"NPC spawn_index out of range: {npc_spec.spawn_index}")
        profile = _load_npc_profile(npc_spec.npc_profile_id) if npc_spec.npc_profile_id else None
        desired_speed_kmh = _resolved_npc_speed_kmh(npc_spec, profile, rng)
        blueprint = require_blueprint(world, npc_spec.vehicle_filter, rng=rng)
        blueprint.set_attribute("role_name", "autopilot")
        actor = world.try_spawn_actor(blueprint, spawn_points[npc_spec.spawn_index])
        if actor is None:
            raise RuntimeError(f"Failed to spawn NPC vehicle at spawn index {npc_spec.spawn_index}.")
        actors.append(actor)

        if profile is None or profile.enable_autopilot:
            actor.set_autopilot(True, traffic_manager.get_port())
            traffic_manager.auto_lane_change(actor, npc_spec.lane_behavior != "keep_lane")
            traffic_manager.ignore_lights_percentage(actor, 0.0)
            traffic_manager.ignore_vehicles_percentage(actor, 0.0)
            speed_limit_kmh = max(1.0, float(actor.get_speed_limit()))
            speed_diff_percent = max(-100.0, min(95.0, ((speed_limit_kmh - desired_speed_kmh) / speed_limit_kmh) * 100.0))
            traffic_manager.vehicle_percentage_speed_difference(actor, speed_diff_percent)
        spawned.append(
            {
                "actor_id": int(actor.id),
                "spawn_index": npc_spec.spawn_index,
                "target_speed_kmh": round(desired_speed_kmh, 2),
                "lane_behavior": npc_spec.lane_behavior,
                "npc_profile_id": npc_spec.npc_profile_id,
            }
        )
    return spawned


def _route_success_criteria(scenario: RouteLoopScenarioSpec) -> dict[str, float | int]:
    return {
        "route_completion_ratio_min": 0.99,
        "collision_count_max": 0,
        "traffic_light_violation_count_max": 0,
        "max_stationary_seconds_max": scenario.max_stop_seconds,
        "goal_tolerance_m_max": scenario.goal_tolerance_m,
        "manual_interventions_max": 0,
    }


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

    if request.mode == "collect" and policy.kind != "expert":
        raise ValueError("Collect mode currently requires policy.kind='expert'.")
    if policy.kind == "learned" and policy.checkpoint_path is None:
        raise ValueError("Learned policy requires checkpoint_path.")

    random_seed = random.Random(runtime.seed)
    route_config_path = Path(scenario.route_config_path).resolve()
    route_config = load_route_config(route_config_path)
    traffic_setup = (
        _load_traffic_setup(Path(scenario.traffic_setup_path).resolve())
        if scenario.traffic_setup_path is not None
        else None
    )
    if traffic_setup is not None and traffic_setup.town != route_config.town:
        raise ValueError(
            f"traffic_setup town mismatch: route={route_config.town} traffic_setup={traffic_setup.town}"
        )
    expert_config_overrides = dict(traffic_setup.expert_overrides) if traffic_setup is not None else {}
    allow_overtake = bool(expert_config_overrides.get("allow_overtake", True))

    git_commit_id = ensure_clean_git_worktree(action_label=f"{request.mode.capitalize()} route loop")
    if request.mode == "evaluate":
        eval_suffix = "pilotnet_eval" if policy.kind == "learned" else "expert_eval"
        episode_id = build_versioned_run_id(f"{route_config.name}_{eval_suffix}", commit_id=git_commit_id)
        episode_dir = PROJECT_ROOT / "outputs" / "evaluate" / episode_id
        manifest_path = episode_dir / "manifest.jsonl"
    else:
        episode_id = build_episode_id(route_config.name)
        episode_dir = PROJECT_ROOT / "outputs" / "collect" / episode_id
        manifest_path = PROJECT_ROOT / "data" / "manifests" / "episodes" / f"{episode_id}.jsonl"
    image_dir = episode_dir / "front_rgb"
    summary_path = episode_dir / "summary.json"
    episode_dir.mkdir(parents=True, exist_ok=True)
    image_dir.mkdir(parents=True, exist_ok=True)

    client = carla.Client(runtime.host, runtime.port)
    client.set_timeout(30.0)

    world, original_settings = setup_world(client, route_config.town, runtime.fixed_delta_seconds)
    world.set_weather(_resolve_weather(carla, scenario.weather))

    frame_events = FrameEventTracker()
    image_queue: queue.Queue[Any] = queue.Queue()
    actors: list[Any] = []

    elapsed_seconds = 0.0
    frame_index = 0
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
    traffic_light_phase_indices: dict[int, int] = {}

    planned_route = build_planned_route(world.get_map(), route_config)
    route_geometry = route_geometry_from_planned_route(planned_route)
    goal_location = planned_route.trace[-1][0].transform.location
    success_criteria = _route_success_criteria(scenario)

    try:
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
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_blueprint, camera_transform, attach_to=vehicle)
        actors.append(camera)
        camera.listen(image_queue.put)

        collision_sensor = attach_sensor(world, "sensor.other.collision", carla.Transform(), vehicle)
        actors.append(collision_sensor)
        collision_sensor.listen(lambda event: frame_events.mark_collision(getattr(event, "other_actor", None)))

        lane_sensor = attach_sensor(world, "sensor.other.lane_invasion", carla.Transform(), vehicle)
        actors.append(lane_sensor)
        lane_sensor.listen(lambda _event: frame_events.mark_lane_invasion())

        lights_by_id = _apply_traffic_light_overrides(
            carla,
            world,
            traffic_setup.traffic_light_overrides if traffic_setup is not None else [],
        )
        _apply_traffic_light_schedules(
            carla,
            lights_by_id,
            traffic_setup.traffic_light_schedules if traffic_setup is not None else [],
            elapsed_seconds=0.0,
            applied_phase_indices=traffic_light_phase_indices,
        )
        npc_actors_summary = _spawn_npc_vehicles(
            client=client,
            world=world,
            runtime=runtime,
            traffic_setup=traffic_setup,
            rng=random_seed,
            actors=actors,
        )

        if policy.kind == "expert":
            stack = create_expert_collector_stack(
                vehicle=vehicle,
                world_map=world.get_map(),
                planned_trace=planned_route.trace,
                route_id=route_config.name,
                town_id=route_config.town,
                target_speed_kmh=runtime.target_speed_kmh,
                ignore_traffic_lights=policy.ignore_traffic_lights,
                ignore_stop_signs=policy.ignore_stop_signs,
                ignore_vehicles=policy.ignore_vehicles,
                sampling_resolution_m=route_config.sampling_resolution_m,
                route_geometry=route_geometry,
                expert_config_overrides=expert_config_overrides,
            )
            stack_description = stack.describe()
        else:
            stack = create_pilotnet_eval_stack(
                vehicle=vehicle,
                world_map=world.get_map(),
                planned_trace=planned_route.trace,
                route_id=route_config.name,
                town_id=route_config.town,
                target_speed_kmh=runtime.target_speed_kmh,
                checkpoint_path=Path(policy.checkpoint_path).resolve(),
                device=policy.device,
                ignore_traffic_lights=policy.ignore_traffic_lights,
                ignore_stop_signs=policy.ignore_stop_signs,
                ignore_vehicles=policy.ignore_vehicles,
                sampling_resolution_m=route_config.sampling_resolution_m,
                route_geometry=route_geometry,
                expert_config_overrides=expert_config_overrides,
            )
            stack_description = stack.describe()

        world_frame = world.tick()
        current_image = wait_for_image(image_queue, world_frame, runtime.sensor_timeout)
        while True:
            _apply_traffic_light_schedules(
                carla,
                lights_by_id,
                traffic_setup.traffic_light_schedules if traffic_setup is not None else [],
                elapsed_seconds=elapsed_seconds,
                applied_phase_indices=traffic_light_phase_indices,
            )
            current_speed = speed_mps(vehicle)
            vehicle_transform = vehicle.get_transform()
            vehicle_location = vehicle_transform.location

            if policy.kind == "learned":
                step_result = stack.run_step(
                    timestamp_s=elapsed_seconds,
                    vehicle_transform=vehicle_transform,
                    speed_mps=current_speed,
                    current_rgb=_carla_image_to_rgb_array(current_image),
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

            image_path = image_dir / f"{frame_index:06d}.png"
            current_image.save_to_disk(str(image_path))

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

            traffic_light_stop_count += int(bool(planning_debug.get("event_traffic_light_stop")))
            traffic_light_resume_count += int(bool(planning_debug.get("event_traffic_light_resume")))
            car_follow_event_count += int(bool(planning_debug.get("event_car_follow_start")))
            overtake_attempt_count += int(bool(planning_debug.get("event_overtake_attempt")))
            overtake_success_count += int(bool(planning_debug.get("event_overtake_success")))
            overtake_abort_count += int(bool(planning_debug.get("event_overtake_abort")))
            unsafe_lane_change_reject_count += int(bool(planning_debug.get("event_unsafe_lane_change_reject")))
            if bool(planning_debug.get("traffic_light_violation")) and not _traffic_light_violation_active:
                traffic_light_violation_count += 1
                _traffic_light_violation_active = True
            elif not bool(planning_debug.get("traffic_light_violation")):
                _traffic_light_violation_active = False
            if planning_debug.get("min_ttc") is not None:
                min_ttc = min(min_ttc, float(planning_debug["min_ttc"]))
            if planning_debug.get("lead_vehicle_distance_m") is not None:
                min_lead_distance_m = min(min_lead_distance_m, float(planning_debug["lead_vehicle_distance_m"]))

            record = EpisodeRecord(
                episode_id=episode_id,
                frame_id=frame_index - 1,
                town_id=route_config.town,
                route_id=route_config.name,
                weather_id=scenario.weather,
                timestamp=current_image.timestamp,
                front_rgb_path=relative_to_project(image_path),
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
                lead_vehicle_distance_m=planning_debug.get("lead_vehicle_distance_m"),
                overtake_state=planning_debug.get("overtake_state"),
                target_lane_id=planning_debug.get("target_lane_id"),
                min_ttc=planning_debug.get("min_ttc"),
            )
            append_jsonl(manifest_path, record)

            if collision:
                failure_reason = "collision"
                break

            if request.mode == "evaluate":
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
            and traffic_light_violation_count <= success_criteria["traffic_light_violation_count_max"]
            and max_stationary_seconds < success_criteria["max_stationary_seconds_max"]
            and distance_to_goal_m <= success_criteria["goal_tolerance_m_max"]
        )
        if not success and not failure_reason:
            if max_completion_ratio < success_criteria["route_completion_ratio_min"]:
                failure_reason = "route_incomplete"
            elif traffic_light_violation_count > success_criteria["traffic_light_violation_count_max"]:
                failure_reason = "traffic_light_violation"
            elif distance_to_goal_m > success_criteria["goal_tolerance_m_max"]:
                failure_reason = "goal_tolerance_exceeded"
            else:
                failure_reason = "success_criteria_failed"
    finally:
        if traffic_setup is not None:
            scheduled_or_overridden_ids = {
                override.actor_id for override in traffic_setup.traffic_light_overrides
            } | {
                schedule.actor_id for schedule in traffic_setup.traffic_light_schedules
            }
            for actor in world.get_actors().filter("*traffic_light*"):
                if int(actor.id) in scheduled_or_overridden_ids:
                    actor.freeze(False)
        client.get_trafficmanager(8000).set_synchronous_mode(False)
        world.apply_settings(original_settings)
        destroy_actors(reversed(actors))

    video_fps = artifacts.video_fps or (1.0 / runtime.fixed_delta_seconds)
    if artifacts.record_video:
        try:
            video_path = render_png_sequence_to_mp4(
                image_dir=image_dir,
                output_path=episode_dir / "front_rgb.mp4",
                fps=video_fps,
                crf=artifacts.video_crf,
            )
        except Exception as exc:  # pragma: no cover - depends on ffmpeg/runtime env
            video_error = str(exc)

    base_summary = {
        "episode_id": episode_id,
        "route_name": route_config.name,
        "route_config_path": relative_to_project(route_config_path),
        "traffic_setup_path": _relative_or_none(Path(scenario.traffic_setup_path).resolve() if scenario.traffic_setup_path else None),
        "traffic_setup_name": traffic_setup.name if traffic_setup is not None else None,
        "expert_config_overrides": expert_config_overrides or None,
        "npc_vehicle_count": len(npc_actors_summary),
        "npc_vehicles": npc_actors_summary,
        "town": route_config.town,
        "weather": scenario.weather,
        "frame_count": frame_index,
        "elapsed_seconds": round(elapsed_seconds, 2),
        "lap_time_seconds": round(elapsed_seconds, 2),
        "target_speed_kmh": runtime.target_speed_kmh,
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
        "min_lead_distance_m": None if not min_lead_distance_m < float("inf") else round(min_lead_distance_m, 3),
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
        "front_rgb_dir": relative_to_project(image_dir),
        "video_path": _relative_or_none(video_path),
        "video_fps": round(video_fps, 3) if video_path is not None else None,
        "video_crf": artifacts.video_crf if video_path is not None else None,
        "video_error": video_error,
        "manifest_path": relative_to_project(manifest_path),
    }

    if request.mode == "collect":
        summary = {
            **base_summary,
            "policy_type": "expert_demonstration",
            "control_source": "ad_stack.run(expert_collect)",
            "route_intent_source": "global_route_planner",
            "lateral_control_source": "carla_local_planner_pid_via_ad_stack_expert_route_policy",
            "is_camera_e2e_policy": False,
        }
    elif policy.kind == "expert":
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
            "model_checkpoint_path": _relative_or_none(Path(policy.checkpoint_path).resolve() if policy.checkpoint_path else None),
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
            raise RuntimeError(f"Failed to spawn ego vehicle at spawn index {scenario.spawn_index}.")
        actors.append(vehicle)

        camera_blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
        camera_blueprint.set_attribute("image_size_x", str(runtime.camera_width))
        camera_blueprint.set_attribute("image_size_y", str(runtime.camera_height))
        camera_blueprint.set_attribute("fov", str(runtime.camera_fov))
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_blueprint, camera_transform, attach_to=vehicle)
        actors.append(camera)
        camera.listen(image_queue.put)

        collision_sensor = attach_sensor(world, "sensor.other.collision", carla.Transform(), vehicle)
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
