from __future__ import annotations

import argparse
import importlib
import json
import random
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[4]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from ad_stack.overtake.infrastructure.carla import build_overtake_scenario_validation
from libs.carla_utils import build_planned_route, destroy_actors, load_route_config, require_blueprint, setup_world
from libs.project import current_git_commit_short, ensure_clean_git_worktree, relative_to_project
from simulation.environment_config import load_environment_config
from simulation.pipelines.route_loop_run_config import load_route_loop_run_config

route_run = importlib.import_module("ad_stack.run")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Inspect stopped-obstacle scenario preflight validity without running the route-loop policy."
        )
    )
    parser.add_argument(
        "config_paths",
        nargs="+",
        help="Path(s) to stopped-obstacle route-loop run config JSON files.",
    )
    return parser


def _project_relative_or_absolute(path: Path) -> str:
    try:
        return relative_to_project(path)
    except ValueError:
        return str(path.resolve())


def _build_output_path(config_path: Path) -> Path:
    run_id = (
        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        f"_{config_path.stem}_inspect_{current_git_commit_short()}"
    )
    output_dir = PROJECT_ROOT / "outputs" / "inspect" / "stopped_obstacle"
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir / f"{run_id}.json"


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
            return float(target_s) - float(origin_s)
    return float(origin_waypoint.transform.location.distance(target_waypoint.transform.location))


def _spawn_candidates_near_start(*, world_map: Any, ego_waypoint: Any, limit: int = 8) -> dict[str, Any]:
    left_waypoint = ego_waypoint.get_left_lane()
    right_waypoint = ego_waypoint.get_right_lane()
    left_lane_id = _lane_id(left_waypoint)
    right_lane_id = _lane_id(right_waypoint)
    candidates = {
        "same_lane_ahead": [],
        "left_lane": [],
        "right_lane": [],
    }
    for spawn_index, transform in enumerate(world_map.get_spawn_points()):
        waypoint = world_map.get_waypoint(transform.location)
        candidate = {
            "spawn_index": spawn_index,
            "lane_id": _lane_id(waypoint),
            "longitudinal_distance_m": _distance_ahead_m(
                origin_waypoint=ego_waypoint,
                target_waypoint=waypoint,
            ),
            "location": {
                "x": round(float(transform.location.x), 3),
                "y": round(float(transform.location.y), 3),
                "z": round(float(transform.location.z), 3),
            },
        }
        if candidate["lane_id"] == _lane_id(ego_waypoint):
            longitudinal_distance_m = candidate["longitudinal_distance_m"]
            if longitudinal_distance_m is not None and longitudinal_distance_m > 0.0:
                candidates["same_lane_ahead"].append(candidate)
        if left_lane_id is not None and candidate["lane_id"] == left_lane_id:
            candidates["left_lane"].append(candidate)
        if right_lane_id is not None and candidate["lane_id"] == right_lane_id:
            candidates["right_lane"].append(candidate)

    def _sort_key(item: dict[str, Any]) -> tuple[float, int]:
        longitudinal = item["longitudinal_distance_m"]
        if longitudinal is None:
            return (float("inf"), int(item["spawn_index"]))
        return (float(longitudinal), int(item["spawn_index"]))

    for key in candidates:
        candidates[key] = sorted(candidates[key], key=_sort_key)[:limit]
    return {
        "ego_lane_id": _lane_id(ego_waypoint),
        "left_lane_id": left_lane_id,
        "right_lane_id": right_lane_id,
        "candidates": candidates,
    }


def _forward_lane_samples(
    start_waypoint: Any | None,
    *,
    step_m: float = 5.0,
    count: int = 8,
) -> list[dict[str, Any]]:
    if start_waypoint is None:
        return []
    samples: list[dict[str, Any]] = []
    current_waypoint = start_waypoint
    progress_m = 0.0
    for sample_index in range(count):
        location = current_waypoint.transform.location
        rotation = current_waypoint.transform.rotation
        samples.append(
            {
                "sample_index": sample_index,
                "progress_m": round(progress_m, 3),
                "lane_id": _lane_id(current_waypoint),
                "s": round(float(getattr(current_waypoint, "s", 0.0)), 3),
                "transform": {
                    "x": round(float(location.x), 3),
                    "y": round(float(location.y), 3),
                    "z": round(float(location.z), 3),
                    "yaw_deg": round(float(rotation.yaw), 3),
                    "pitch_deg": round(float(rotation.pitch), 3),
                    "roll_deg": round(float(rotation.roll), 3),
                },
            }
        )
        next_waypoints = current_waypoint.next(step_m)
        if not next_waypoints:
            break
        current_waypoint = next_waypoints[0]
        progress_m += step_m
    return samples


def _inspect_single_config(config_path: Path) -> dict[str, Any]:
    loaded_config = load_route_loop_run_config(config_path)
    request = loaded_config.request
    scenario = route_run._validate_route_loop_request(request)
    runtime = request.runtime
    random_seed = random.Random(runtime.seed)

    if scenario.environment_config_path is None:
        raise ValueError("Stopped-obstacle inspection requires scenario.environment_config_path.")

    environment_config_path = Path(scenario.environment_config_path).resolve()
    environment_config = load_environment_config(environment_config_path)
    if environment_config.overtake_scenario is None:
        raise ValueError(
            "Stopped-obstacle inspection requires environment.overtake_scenario."
        )

    route_config_path = Path(scenario.route_config_path).resolve()
    route_config = load_route_config(route_config_path)

    carla = route_run._require_carla()
    client = carla.Client(runtime.host, runtime.port)
    client.set_timeout(30.0)
    world, original_settings = setup_world(client, route_config.town, runtime.fixed_delta_seconds)
    world.set_weather(route_run._resolve_weather(carla, environment_config.weather or scenario.weather))

    planned_route = build_planned_route(world.get_map(), route_config)
    actors: list[Any] = []
    npc_actor_refs: list[Any] = []
    try:
        vehicle_blueprint = require_blueprint(world, runtime.vehicle_filter, rng=random_seed)
        vehicle_blueprint.set_attribute("role_name", "hero")
        vehicle = world.try_spawn_actor(vehicle_blueprint, planned_route.anchor_transforms[0])
        if vehicle is None:
            raise RuntimeError("Failed to spawn ego vehicle at the first route anchor.")
        actors.append(vehicle)

        npc_actors_summary = route_run._spawn_npc_vehicles(
            carla_module=carla,
            client=client,
            world=world,
            runtime=runtime,
            environment_config=environment_config,
            rng=random_seed,
            actors=actors,
            spawned_actor_refs=npc_actor_refs,
        )
        for _ in range(3):
            world.tick()

        ego_waypoint = world.get_map().get_waypoint(vehicle.get_location())
        scenario_validation = build_overtake_scenario_validation(
            environment_config=environment_config,
            world_map=world.get_map(),
            route_trace=planned_route.trace,
            ego_vehicle=vehicle,
            npc_actor_refs=npc_actor_refs,
            driving_lane_type=carla.LaneType.Driving,
        )
        left_waypoint = ego_waypoint.get_left_lane()
        right_waypoint = ego_waypoint.get_right_lane()
        spawn_candidates = _spawn_candidates_near_start(
            world_map=world.get_map(),
            ego_waypoint=ego_waypoint,
        )
        lane_samples = {
            "ego_lane_id": _lane_id(ego_waypoint),
            "left_lane_id": _lane_id(left_waypoint),
            "right_lane_id": _lane_id(right_waypoint),
            "same_lane_ahead": _forward_lane_samples(ego_waypoint),
            "left_lane_ahead": _forward_lane_samples(left_waypoint),
            "right_lane_ahead": _forward_lane_samples(right_waypoint),
        }
    finally:
        world.apply_settings(original_settings)
        destroy_actors(reversed(actors))

    output_path = _build_output_path(config_path)
    payload = {
        "config_path": _project_relative_or_absolute(loaded_config.config_path),
        "route_config_path": _project_relative_or_absolute(route_config_path),
        "environment_config_path": _project_relative_or_absolute(environment_config_path),
        "route_name": route_config.name,
        "environment_name": environment_config.name,
        "town": route_config.town,
        "overtake_scenario": {
            "scenario_kind": environment_config.overtake_scenario.scenario_kind,
            "obstacle_npc_index": environment_config.overtake_scenario.obstacle_npc_index,
            "blocker_npc_index": environment_config.overtake_scenario.blocker_npc_index,
        },
        "npc_vehicles": npc_actors_summary,
        "scenario_validation": scenario_validation,
        "spawn_candidates": spawn_candidates,
        "lane_samples": lane_samples,
    }
    output_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    payload["output_path"] = _project_relative_or_absolute(output_path)
    return payload


def main() -> None:
    args = build_parser().parse_args()
    ensure_clean_git_worktree(action_label="Stopped obstacle suite inspection")
    results = [_inspect_single_config(Path(path)) for path in args.config_paths]
    if len(results) == 1:
        print(json.dumps(results[0], indent=2))
    else:
        print(json.dumps(results, indent=2))
    if any(not bool(result.get("scenario_validation", {}).get("valid", False)) for result in results):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
