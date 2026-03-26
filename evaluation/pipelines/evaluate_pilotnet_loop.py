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

from ad_stack import (
    create_pilotnet_eval_stack,
    to_carla_control,
)
from libs.carla_utils import (
    DEFAULT_ROUTE_CONFIG_PATH,
    FrameEventTracker,
    attach_sensor,
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
from libs.project import PROJECT_ROOT, build_versioned_run_id, ensure_clean_git_worktree_for_evaluation
from libs.schemas import EpisodeRecord, append_jsonl
from libs.utils import render_png_sequence_to_mp4
from evaluation.pipelines.common import carla_image_to_rgb_array, resolve_weather


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Evaluate a learned PilotNet steer policy on the Town01 fixed loop.")
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--route-config", default=str(DEFAULT_ROUTE_CONFIG_PATH))
    parser.add_argument("--vehicle-filter", default="vehicle.tesla.model3")
    parser.add_argument("--fixed-delta-seconds", type=float, default=0.05)
    parser.add_argument("--sensor-timeout", type=float, default=2.0)
    parser.add_argument("--target-speed-kmh", type=float, default=30.0)
    parser.add_argument("--camera-width", type=int, default=320)
    parser.add_argument("--camera-height", type=int, default=180)
    parser.add_argument("--camera-fov", type=int, default=90)
    parser.add_argument("--goal-tolerance-m", type=float, default=10.0)
    parser.add_argument("--max-stop-seconds", type=float, default=10.0)
    parser.add_argument("--stationary-speed-threshold-mps", type=float, default=0.5)
    parser.add_argument("--max-seconds", type=float, default=600.0)
    parser.add_argument("--weather", default="ClearNoon")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--device", default=None)
    parser.add_argument("--steer-smoothing", type=float, default=1.0)
    parser.add_argument("--max-steer-delta", type=float, default=None)
    parser.add_argument(
        "--record-video",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
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


def main() -> None:
    args = build_parser().parse_args()
    if carla is None:
        raise SystemExit(
            "The 'carla' Python package is not installed. "
            "Run 'uv sync' after confirming the CARLA wheel path in pyproject.toml."
        )

    git_commit_id = ensure_clean_git_worktree_for_evaluation()
    checkpoint_path = Path(args.checkpoint).resolve()

    random_seed = random.Random(args.seed)
    route_config_path = Path(args.route_config).resolve()
    route_config = load_route_config(route_config_path)

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)

    world, original_settings = setup_world(client, route_config.town, args.fixed_delta_seconds)
    world.set_weather(resolve_weather(carla, args.weather))

    episode_id = build_versioned_run_id(f"{route_config.name}_pilotnet_eval", commit_id=git_commit_id)
    episode_dir = PROJECT_ROOT / "outputs" / "evaluate" / episode_id
    image_dir = episode_dir / "front_rgb"
    manifest_path = PROJECT_ROOT / "outputs" / "evaluate" / episode_id / "manifest.jsonl"
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
        camera_blueprint.set_attribute("image_size_x", str(args.camera_width))
        camera_blueprint.set_attribute("image_size_y", str(args.camera_height))
        camera_blueprint.set_attribute("fov", str(args.camera_fov))
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

        stack = create_pilotnet_eval_stack(
            vehicle=vehicle,
            world_map=world.get_map(),
            planned_trace=planned_route.trace,
            route_id=route_config.name,
            town_id=route_config.town,
            target_speed_kmh=args.target_speed_kmh,
            checkpoint_path=checkpoint_path,
            device=args.device,
            ignore_traffic_lights=args.ignore_traffic_lights,
            ignore_stop_signs=args.ignore_stop_signs,
            ignore_vehicles=args.ignore_vehicles,
            sampling_resolution_m=route_config.sampling_resolution_m,
            route_geometry=route_geometry,
        )
        stack_description = stack.describe()

        world_frame = world.tick()
        current_image = wait_for_image(image_queue, world_frame, args.sensor_timeout)
        while True:
            current_rgb = carla_image_to_rgb_array(current_image)
            current_speed = speed_mps(vehicle)
            vehicle_transform = vehicle.get_transform()
            vehicle_location = vehicle_transform.location
            step_result = stack.run_step(
                timestamp_s=elapsed_seconds,
                vehicle_transform=vehicle_transform,
                speed_mps=current_speed,
                current_rgb=current_rgb,
                steering_smoothing=args.steer_smoothing,
                max_steer_delta=args.max_steer_delta,
            )
            decision = step_result.decision
            expert_decision = step_result.expert_decision
            predicted_steer = step_result.applied_steer if step_result.applied_steer is not None else decision.command.steer
            control = to_carla_control(carla, decision.command)
            vehicle.apply_control(control)

            image_path = image_dir / f"{frame_index:06d}.png"
            current_image.save_to_disk(str(image_path))

            collision, lane_invasion = frame_events.consume_frame_flags()
            speed_sum += current_speed
            frame_index += 1
            elapsed_seconds = frame_index * args.fixed_delta_seconds
            average_speed_mps = speed_sum / frame_index

            if current_speed < args.stationary_speed_threshold_mps:
                current_stationary_seconds += args.fixed_delta_seconds
            else:
                current_stationary_seconds = 0.0
            max_stationary_seconds = max(max_stationary_seconds, current_stationary_seconds)

            current_completion_ratio = step_result.progress_ratio
            max_completion_ratio = max(max_completion_ratio, current_completion_ratio)
            distance_to_goal_m = vehicle_location.distance(goal_location)
            route_point = step_result.route_point

            reached_success_goal = (
                not collision
                and current_completion_ratio >= success_criteria["route_completion_ratio_min"]
                and distance_to_goal_m <= success_criteria["goal_tolerance_m_max"]
            )

            record = EpisodeRecord(
                episode_id=episode_id,
                frame_id=frame_index - 1,
                town_id=route_config.town,
                route_id=route_config.name,
                weather_id=args.weather,
                timestamp=current_image.timestamp,
                front_rgb_path=relative_to_project(image_path),
                speed=current_speed,
                command=decision.behavior,
                steer=predicted_steer,
                throttle=control.throttle,
                brake=control.brake,
                collision=collision,
                lane_invasion=lane_invasion,
                success=not collision,
                vehicle_x=vehicle_location.x,
                vehicle_y=vehicle_location.y,
                vehicle_z=vehicle_location.z,
                vehicle_yaw_deg=vehicle_transform.rotation.yaw,
                route_completion_ratio=current_completion_ratio,
                distance_to_goal_m=distance_to_goal_m,
                expert_steer=expert_decision.command.steer if expert_decision is not None else None,
                route_target_x=route_point[0] if route_point is not None else None,
                route_target_y=route_point[1] if route_point is not None else None,
            )
            append_jsonl(manifest_path, record)

            if collision:
                failure_reason = "collision"
                break

            if reached_success_goal:
                break

            if max_stationary_seconds >= args.max_stop_seconds:
                failure_reason = "stalled"
                break

            if step_result.done:
                break

            if elapsed_seconds >= args.max_seconds:
                failure_reason = "max_seconds_exceeded"
                break

            world_frame = world.tick()
            current_image = wait_for_image(image_queue, world_frame, args.sensor_timeout)

        max_completion_ratio = max(max_completion_ratio, current_completion_ratio if frame_index else 0.0)
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

    if args.record_video:
        try:
            rendered_video = render_png_sequence_to_mp4(
                image_dir=image_dir,
                output_path=episode_dir / "front_rgb.mp4",
                fps=1.0 / args.fixed_delta_seconds,
                crf=args.video_crf,
            )
            video_path = relative_to_project(rendered_video)
        except Exception as exc:
            video_error = str(exc)

    summary = {
        "episode_id": episode_id,
        "policy_type": "learned_lateral_policy",
        "control_source": "ad_stack_learned_lateral_agent",
        "route_intent_source": "global_route_planner",
        "lateral_control_source": "pilotnet_learning_runtime_via_ad_stack",
        "longitudinal_control_source": "ad_stack_expert_basic_agent_longitudinal",
        "is_camera_e2e_policy": True,
        "e2e_scope": "front_rgb_plus_speed_to_steer",
        "model_checkpoint_path": relative_to_project(checkpoint_path),
        "model_name": stack_description.model_name,
        "git_commit_id": git_commit_id,
        "steer_smoothing": args.steer_smoothing,
        "max_steer_delta": args.max_steer_delta,
        "route_name": route_config.name,
        "route_config_path": relative_to_project(route_config_path),
        "town": route_config.town,
        "weather": args.weather,
        "frame_count": frame_index,
        "elapsed_seconds": round(elapsed_seconds, 2),
        "lap_time_seconds": round(elapsed_seconds, 2),
        "target_speed_kmh": args.target_speed_kmh,
        "camera_width": args.camera_width,
        "camera_height": args.camera_height,
        "camera_fov": args.camera_fov,
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
        "video_fps": round(1.0 / args.fixed_delta_seconds, 3) if video_path else None,
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
