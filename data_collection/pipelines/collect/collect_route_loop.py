from __future__ import annotations

import argparse
import json
from pathlib import Path
import queue
import random

try:
    import carla
except ModuleNotFoundError:
    carla = None  # type: ignore[assignment]

from libs.project import PROJECT_ROOT
from libs.carla_utils import (
    DEFAULT_ROUTE_CONFIG_PATH,
    FrameEventTracker,
    attach_sensor,
    build_episode_id,
    build_planned_route,
    compute_local_target_point,
    destroy_actors,
    ensure_carla_agents_on_path,
    load_route_config,
    relative_to_project,
    require_blueprint,
    road_option_name,
    route_geometry_from_planned_route,
    setup_world,
    speed_mps,
    wait_for_image,
)
from libs.schemas import EpisodeRecord, append_jsonl
from libs.utils import render_png_sequence_to_mp4


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Collect a fixed-loop Town01 episode using CARLA's planner as an expert."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--route-config", default=str(DEFAULT_ROUTE_CONFIG_PATH))
    parser.add_argument("--vehicle-filter", default="vehicle.tesla.model3")
    parser.add_argument("--fixed-delta-seconds", type=float, default=0.05)
    parser.add_argument("--sensor-timeout", type=float, default=2.0)
    parser.add_argument("--image-width", type=int, default=320)
    parser.add_argument("--image-height", type=int, default=180)
    parser.add_argument("--image-fov", type=int, default=90)
    parser.add_argument("--target-speed-kmh", type=float, default=30.0)
    parser.add_argument("--goal-tolerance-m", type=float, default=10.0)
    parser.add_argument("--max-stop-seconds", type=float, default=10.0)
    parser.add_argument("--stationary-speed-threshold-mps", type=float, default=0.5)
    parser.add_argument("--max-seconds", type=float, default=600.0)
    parser.add_argument("--weather", default="ClearNoon")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument(
        "--record-video",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--video-fps", type=float, default=None)
    parser.add_argument("--video-crf", type=int, default=23)
    parser.add_argument(
        "--ignore-traffic-lights",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--ignore-stop-signs",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--ignore-vehicles",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    return parser


def resolve_weather(carla_module, weather_name: str):
    weather = getattr(carla_module.WeatherParameters, weather_name, None)
    if weather is None:
        raise ValueError(f"Unknown CARLA weather preset: {weather_name}")
    return weather


def remaining_waypoints(agent) -> int:
    return len(agent.get_local_planner().get_plan())


def completion_ratio(initial_waypoint_count: int, remaining_count: int) -> float:
    if initial_waypoint_count <= 0:
        return 0.0
    progress = 1.0 - (remaining_count / initial_waypoint_count)
    return max(0.0, min(1.0, progress))


def main() -> None:
    args = build_parser().parse_args()
    if carla is None:
        raise SystemExit(
            "The 'carla' Python package is not installed. "
            "Run 'uv sync' after confirming the CARLA wheel path in pyproject.toml."
        )

    ensure_carla_agents_on_path()
    from agents.navigation.basic_agent import BasicAgent

    random_seed = random.Random(args.seed)
    route_config_path = Path(args.route_config).resolve()
    route_config = load_route_config(route_config_path)

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)

    world, original_settings = setup_world(client, route_config.town, args.fixed_delta_seconds)
    world.set_weather(resolve_weather(carla, args.weather))

    episode_id = build_episode_id(route_config.name)
    episode_dir = PROJECT_ROOT / "outputs" / "collect" / episode_id
    image_dir = episode_dir / "front_rgb"
    manifest_path = PROJECT_ROOT / "data" / "manifests" / "episodes" / f"{episode_id}.jsonl"
    summary_path = episode_dir / "summary.json"
    episode_dir.mkdir(parents=True, exist_ok=True)
    image_dir.mkdir(parents=True, exist_ok=True)

    frame_events = FrameEventTracker()
    image_queue: queue.Queue[carla.Image] = queue.Queue()
    actors: list[carla.Actor] = []

    elapsed_seconds = 0.0
    frame_index = 0
    max_stationary_seconds = 0.0
    current_stationary_seconds = 0.0
    average_speed_mps = 0.0
    speed_sum = 0.0
    max_completion_ratio = 0.0
    success = False
    failure_reason = ""
    distance_to_goal_m = 0.0
    video_path: str | None = None
    video_error: str | None = None
    route_index_cursor: int | None = None

    planned_route = build_planned_route(world.get_map(), route_config)
    route_geometry = route_geometry_from_planned_route(planned_route)
    goal_location = planned_route.trace[-1][0].transform.location
    success_criteria = {
        "route_completion_ratio_min": 0.99,
        "collision_count_max": 0,
        "max_stationary_seconds_max": args.max_stop_seconds,
        "goal_tolerance_m_max": args.goal_tolerance_m,
        "manual_interventions_max": 0,
    }

    try:
        vehicle_blueprint = require_blueprint(world, args.vehicle_filter, rng=random_seed)
        vehicle_blueprint.set_attribute("role_name", "hero")
        vehicle = world.try_spawn_actor(vehicle_blueprint, planned_route.anchor_transforms[0])
        if vehicle is None:
            raise RuntimeError("Failed to spawn ego vehicle at the first route anchor.")
        actors.append(vehicle)

        camera_blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
        camera_blueprint.set_attribute("image_size_x", str(args.image_width))
        camera_blueprint.set_attribute("image_size_y", str(args.image_height))
        camera_blueprint.set_attribute("fov", str(args.image_fov))
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

        agent = BasicAgent(
            vehicle,
            target_speed=args.target_speed_kmh,
            opt_dict={
                "ignore_traffic_lights": args.ignore_traffic_lights,
                "ignore_stop_signs": args.ignore_stop_signs,
                "ignore_vehicles": args.ignore_vehicles,
                "sampling_resolution": route_config.sampling_resolution_m,
            },
            map_inst=world.get_map(),
        )
        agent.set_global_plan(planned_route.trace, stop_waypoint_creation=True, clean_queue=True)
        initial_waypoint_count = remaining_waypoints(agent)
        if initial_waypoint_count <= 0:
            raise RuntimeError("Planner produced an empty local plan.")

        world.tick()
        while True:
            control = agent.run_step()
            vehicle.apply_control(control)
            current_completion_ratio = completion_ratio(initial_waypoint_count, remaining_waypoints(agent))
            max_completion_ratio = max(max_completion_ratio, current_completion_ratio)

            world_frame = world.tick()
            image = wait_for_image(image_queue, world_frame, args.sensor_timeout)

            image_path = image_dir / f"{frame_index:06d}.png"
            image.save_to_disk(str(image_path))

            collision, lane_invasion = frame_events.consume_frame_flags()
            speed = speed_mps(vehicle)
            speed_sum += speed
            frame_index += 1
            elapsed_seconds = frame_index * args.fixed_delta_seconds
            average_speed_mps = speed_sum / frame_index

            if speed < args.stationary_speed_threshold_mps:
                current_stationary_seconds += args.fixed_delta_seconds
            else:
                current_stationary_seconds = 0.0
            max_stationary_seconds = max(max_stationary_seconds, current_stationary_seconds)

            vehicle_transform = vehicle.get_transform()
            vehicle_location = vehicle_transform.location
            distance_to_goal_m = vehicle_location.distance(goal_location)
            command = road_option_name(agent.get_local_planner().target_road_option)
            route_point, route_index_cursor = compute_local_target_point(
                route_geometry,
                vehicle_x=vehicle_location.x,
                vehicle_y=vehicle_location.y,
                vehicle_yaw_deg=vehicle_transform.rotation.yaw,
                lookahead_m=8.0,
                target_normalization_m=20.0,
                previous_route_index=route_index_cursor,
            )

            episode_success_so_far = not collision and not failure_reason
            record = EpisodeRecord(
                episode_id=episode_id,
                frame_id=frame_index - 1,
                town_id=route_config.town,
                route_id=route_config.name,
                weather_id=args.weather,
                timestamp=image.timestamp,
                front_rgb_path=relative_to_project(image_path),
                speed=speed,
                command=command,
                steer=control.steer,
                throttle=control.throttle,
                brake=control.brake,
                collision=collision,
                lane_invasion=lane_invasion,
                success=episode_success_so_far,
                vehicle_x=vehicle_location.x,
                vehicle_y=vehicle_location.y,
                vehicle_z=vehicle_location.z,
                vehicle_yaw_deg=vehicle_transform.rotation.yaw,
                route_completion_ratio=current_completion_ratio,
                distance_to_goal_m=distance_to_goal_m,
                route_target_x=route_point[0],
                route_target_y=route_point[1],
            )
            append_jsonl(manifest_path, record)

            if collision:
                failure_reason = "collision"
                break

            if max_stationary_seconds >= args.max_stop_seconds:
                failure_reason = "stalled"
                break

            if agent.done():
                break

            if elapsed_seconds >= args.max_seconds:
                failure_reason = "max_seconds_exceeded"
                break

        max_completion_ratio = max(max_completion_ratio, completion_ratio(initial_waypoint_count, remaining_waypoints(agent)))
        distance_to_goal_m = vehicle.get_location().distance(goal_location)
        success = (
            not failure_reason
            and max_completion_ratio >= success_criteria["route_completion_ratio_min"]
            and frame_events.collision_count <= success_criteria["collision_count_max"]
            and max_stationary_seconds < success_criteria["max_stationary_seconds_max"]
            and distance_to_goal_m <= success_criteria["goal_tolerance_m_max"]
        )
        if not success and not failure_reason:
            if max_completion_ratio < success_criteria["route_completion_ratio_min"]:
                failure_reason = "route_incomplete"
            elif distance_to_goal_m > success_criteria["goal_tolerance_m_max"]:
                failure_reason = "goal_tolerance_exceeded"
            else:
                failure_reason = "success_criteria_failed"
    finally:
        world.apply_settings(original_settings)
        destroy_actors(reversed(actors))

    video_fps = args.video_fps or (1.0 / args.fixed_delta_seconds)
    if args.record_video:
        try:
            rendered_video = render_png_sequence_to_mp4(
                image_dir=image_dir,
                output_path=episode_dir / "front_rgb.mp4",
                fps=video_fps,
                crf=args.video_crf,
            )
            video_path = relative_to_project(rendered_video)
        except Exception as exc:
            video_error = str(exc)

    summary = {
        "episode_id": episode_id,
        "policy_type": "expert_demonstration",
        "control_source": "carla_basic_agent",
        "route_intent_source": "global_route_planner",
        "lateral_control_source": "basic_agent_local_planner_pid",
        "is_camera_e2e_policy": False,
        "route_name": route_config.name,
        "route_config_path": relative_to_project(route_config_path),
        "town": route_config.town,
        "weather": args.weather,
        "frame_count": frame_index,
        "elapsed_seconds": round(elapsed_seconds, 2),
        "lap_time_seconds": round(elapsed_seconds, 2),
        "target_speed_kmh": args.target_speed_kmh,
        "average_speed_kmh": round(average_speed_mps * 3.6, 2),
        "collision_count": frame_events.collision_count,
        "lane_invasion_count": frame_events.lane_invasion_count,
        "max_stationary_seconds": round(max_stationary_seconds, 2),
        "route_completion_ratio": round(max_completion_ratio, 4),
        "distance_to_goal_m": round(distance_to_goal_m, 2),
        "goal_tolerance_m": args.goal_tolerance_m,
        "manual_interventions": 0,
        "success": success,
        "failure_reason": failure_reason or None,
        "success_criteria": success_criteria,
        "segment_summaries": planned_route.segment_summaries,
        "road_option_counts": planned_route.road_option_counts,
        "front_rgb_dir": relative_to_project(image_dir),
        "video_path": video_path,
        "video_fps": round(video_fps, 3) if video_path else None,
        "video_crf": args.video_crf if video_path else None,
        "video_error": video_error,
        "manifest_path": relative_to_project(manifest_path),
    }
    with summary_path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
        handle.write("\n")

    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
