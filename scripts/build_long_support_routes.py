#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build longer-context support routes for failed Town01 movement-suite routes."
    )
    parser.add_argument("--suite-summary", required=True)
    parser.add_argument(
        "--inventory-path",
        default=str(PROJECT_ROOT / "docs" / "assets" / "town01_movement_inventory.json"),
    )
    parser.add_argument(
        "--output-dir",
        default=str(PROJECT_ROOT / "configs" / "routes" / "town01_movement_support_long"),
    )
    parser.add_argument("--min-extra-approach-m", type=float, default=20.0)
    parser.add_argument("--min-extra-exit-m", type=float, default=20.0)
    return parser


def resolve_path(path_str: str) -> Path:
    path = Path(path_str)
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    return path.resolve()


def route_config_dict(name: str, start_spawn_index: int, end_spawn_index: int, description: str) -> dict:
    return {
        "name": name,
        "town": "Town01",
        "closed_loop": False,
        "sampling_resolution_m": 2.0,
        "anchor_spawn_indices": [start_spawn_index, end_spawn_index],
        "description": description,
    }


def find_movement_for_spawn_pair(inventory: dict, start_spawn_index: int, end_spawn_index: int) -> tuple[dict, dict]:
    for movement in inventory["movements"]:
        for candidate in movement["candidates"]:
            if candidate["start_spawn_index"] == start_spawn_index and candidate["end_spawn_index"] == end_spawn_index:
                return movement, candidate
    raise KeyError(f"Failed to locate movement for spawn pair {start_spawn_index} -> {end_spawn_index}")


def choose_long_support_candidate(
    movement: dict,
    failing_candidate: dict,
    *,
    min_extra_approach_m: float,
    min_extra_exit_m: float,
) -> dict | None:
    current_approach = failing_candidate["approach_length_m"]
    current_exit = failing_candidate["exit_length_m"]
    eligible = []
    for candidate in movement["candidates"]:
        if (
            candidate["start_spawn_index"] == failing_candidate["start_spawn_index"]
            and candidate["end_spawn_index"] == failing_candidate["end_spawn_index"]
        ):
            continue
        if candidate["approach_length_m"] >= current_approach + min_extra_approach_m and candidate["exit_length_m"] >= current_exit + min_extra_exit_m:
            eligible.append(candidate)
    if not eligible:
        return None
    eligible.sort(
        key=lambda item: (
            -min(item["approach_length_m"], item["exit_length_m"]),
            -(item["approach_length_m"] + item["exit_length_m"]),
            item["start_spawn_index"],
            item["end_spawn_index"],
        )
    )
    return eligible[0]


def main() -> None:
    args = build_parser().parse_args()
    suite_summary_path = resolve_path(args.suite_summary)
    inventory_path = resolve_path(args.inventory_path)
    output_dir = resolve_path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    suite_summary = json.load(suite_summary_path.open("r", encoding="utf-8"))
    inventory = json.load(inventory_path.open("r", encoding="utf-8"))

    artifacts = []
    for result in suite_summary["results"]:
        if result["success"]:
            continue
        route_config = json.load(resolve_path(result["route_config_path"]).open("r", encoding="utf-8"))
        start_spawn_index, end_spawn_index = route_config["anchor_spawn_indices"]
        movement, failing_candidate = find_movement_for_spawn_pair(inventory, start_spawn_index, end_spawn_index)
        chosen = choose_long_support_candidate(
            movement,
            failing_candidate,
            min_extra_approach_m=args.min_extra_approach_m,
            min_extra_exit_m=args.min_extra_exit_m,
        )
        if chosen is None:
            continue
        output_name = f"{result['route_name']}_longsupport"
        output_path = output_dir / f"{output_name}.json"
        config = route_config_dict(
            output_name,
            chosen["start_spawn_index"],
            chosen["end_spawn_index"],
            (
                f"Long-context support route for {result['route_name']} "
                f"({chosen['start_spawn_index']} -> {chosen['end_spawn_index']}, "
                f"approach {chosen['approach_length_m']} m, exit {chosen['exit_length_m']} m)."
            ),
        )
        with output_path.open("w", encoding="utf-8") as handle:
            json.dump(config, handle, indent=2)
            handle.write("\n")
        artifacts.append(
            {
                "route_name": result["route_name"],
                "movement_key": movement["movement_key"],
                "failing_spawn_indices": [start_spawn_index, end_spawn_index],
                "failing_approach_m": failing_candidate["approach_length_m"],
                "failing_exit_m": failing_candidate["exit_length_m"],
                "support_spawn_indices": [chosen["start_spawn_index"], chosen["end_spawn_index"]],
                "support_approach_m": chosen["approach_length_m"],
                "support_exit_m": chosen["exit_length_m"],
                "output_path": str(output_path.relative_to(PROJECT_ROOT)),
            }
        )

    print(
        json.dumps(
            {
                "suite_summary": str(suite_summary_path.relative_to(PROJECT_ROOT)),
                "built_count": len(artifacts),
                "output_dir": str(output_dir.relative_to(PROJECT_ROOT)),
                "artifacts": artifacts,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
