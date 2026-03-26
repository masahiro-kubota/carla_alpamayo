#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build one Town01 route config per movement inventory entry."
    )
    parser.add_argument(
        "--inventory-path",
        default=str(PROJECT_ROOT / "docs" / "assets" / "town01_movement_inventory.json"),
    )
    parser.add_argument(
        "--output-dir",
        default=str(PROJECT_ROOT / "data_collection" / "configs" / "routes" / "town01_movement_all"),
    )
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


def candidate_sort_key(candidate: dict) -> tuple[float, float, float, int, int]:
    return (
        -min(candidate["approach_length_m"], candidate["exit_length_m"]),
        -candidate["approach_length_m"],
        -candidate["exit_length_m"],
        candidate["start_spawn_index"],
        candidate["end_spawn_index"],
    )


def choose_candidate(movement: dict) -> tuple[dict, str]:
    if movement.get("recommended_eval") is not None:
        return movement["recommended_eval"], "recommended_eval"
    if movement.get("recommended_train") is not None:
        return movement["recommended_train"], "recommended_train"
    candidates = sorted(movement["candidates"], key=candidate_sort_key)
    if not candidates:
        raise ValueError(f"No candidates available for movement {movement['movement_key']}")
    return candidates[0], "fallback_best_candidate"


def main() -> None:
    args = build_parser().parse_args()
    inventory_path = resolve_path(args.inventory_path)
    output_dir = resolve_path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    with inventory_path.open("r", encoding="utf-8") as handle:
        inventory = json.load(handle)

    artifacts: list[dict] = []
    for movement in inventory["movements"]:
        candidate, source = choose_candidate(movement)
        route_name = (
            f"town01_movement_all_{movement['movement_label'].lower()}_"
            f"j{movement['junction_id']}_{candidate['start_spawn_index']}_{candidate['end_spawn_index']}"
        )
        output_path = output_dir / f"{route_name}.json"
        config = route_config_dict(
            route_name,
            candidate["start_spawn_index"],
            candidate["end_spawn_index"],
            (
                f"Town01 all-movement route for {movement['movement_label']} junction "
                f"{movement['junction_id']} ({candidate['start_spawn_index']} -> {candidate['end_spawn_index']}, "
                f"selected via {source})."
            ),
        )
        with output_path.open("w", encoding="utf-8") as handle:
            json.dump(config, handle, indent=2)
            handle.write("\n")
        artifacts.append(
            {
                "route_name": route_name,
                "movement_key": movement["movement_key"],
                "movement_label": movement["movement_label"],
                "junction_id": movement["junction_id"],
                "start_spawn_index": candidate["start_spawn_index"],
                "end_spawn_index": candidate["end_spawn_index"],
                "selection_source": source,
                "output_path": str(output_path.relative_to(PROJECT_ROOT)),
            }
        )

    print(
        json.dumps(
            {
                "movement_count": len(inventory["movements"]),
                "output_dir": str(output_dir.relative_to(PROJECT_ROOT)),
                "artifacts": artifacts,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
