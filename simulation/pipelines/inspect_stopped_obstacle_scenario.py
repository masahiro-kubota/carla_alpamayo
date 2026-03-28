from __future__ import annotations

import argparse
import json
import random
from datetime import datetime
from pathlib import Path
from typing import Any

from ad_stack import run as route_run
from libs.carla_utils import build_planned_route, destroy_actors, load_route_config, require_blueprint, setup_world
from libs.project import PROJECT_ROOT, current_git_commit_short, ensure_clean_git_worktree, relative_to_project
from simulation.environment_config import load_environment_config
from simulation.pipelines.route_loop_run_config import load_route_loop_run_config
from simulation.stopped_obstacle_scenario_validation import build_stopped_obstacle_scenario_validation


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
    if environment_config.stopped_obstacle_scenario is None:
        raise ValueError(
            "Stopped-obstacle inspection requires environment.stopped_obstacle_scenario."
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

        scenario_validation = build_stopped_obstacle_scenario_validation(
            environment_config=environment_config,
            world_map=world.get_map(),
            route_trace=planned_route.trace,
            ego_vehicle=vehicle,
            npc_actor_refs=npc_actor_refs,
            driving_lane_type=carla.LaneType.Driving,
        )
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
        "stopped_obstacle_scenario": {
            "scenario_kind": environment_config.stopped_obstacle_scenario.scenario_kind,
            "obstacle_npc_index": environment_config.stopped_obstacle_scenario.obstacle_npc_index,
            "blocker_npc_index": environment_config.stopped_obstacle_scenario.blocker_npc_index,
        },
        "npc_vehicles": npc_actors_summary,
        "scenario_validation": scenario_validation,
    }
    output_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    payload["output_path"] = _project_relative_or_absolute(output_path)
    return payload


def main() -> None:
    args = build_parser().parse_args()
    ensure_clean_git_worktree(action_label="Stopped obstacle scenario inspection")
    results = [_inspect_single_config(Path(path)) for path in args.config_paths]
    if len(results) == 1:
        print(json.dumps(results[0], indent=2))
    else:
        print(json.dumps(results, indent=2))
    if any(not bool(result.get("scenario_validation", {}).get("valid", False)) for result in results):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
