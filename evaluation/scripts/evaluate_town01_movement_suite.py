#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from libs.project import build_versioned_run_id, ensure_clean_git_worktree_for_evaluation, relative_to_project


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Evaluate a checkpoint over the generated Town01 movement route suite.")
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument(
        "--route-dir",
        default=str(PROJECT_ROOT / "data_collection" / "configs" / "routes" / "town01_movement_eval"),
    )
    parser.add_argument("--route-glob", default="*.json")
    parser.add_argument("--max-routes", type=int, default=None)
    parser.add_argument("--summary-output", default=None)
    parser.add_argument("--record-video", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--target-speed-kmh", type=float, default=30.0)
    parser.add_argument("--weather", default="ClearNoon")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--max-seconds", type=float, default=300.0)
    return parser


def build_output_path(explicit_path: str | None, *, git_commit_id: str) -> Path:
    if explicit_path:
        output_path = Path(explicit_path)
        if not output_path.is_absolute():
            output_path = PROJECT_ROOT / output_path
        return output_path
    run_id = build_versioned_run_id("town01_movement_eval", commit_id=git_commit_id)
    return PROJECT_ROOT / "outputs" / "evaluate_suites" / f"{run_id}.json"


def load_route_paths(route_dir: Path, route_glob: str, max_routes: int | None) -> list[Path]:
    route_paths = sorted(route_dir.glob(route_glob))
    if max_routes is not None:
        route_paths = route_paths[:max_routes]
    return route_paths


def display_path(path: Path) -> str:
    try:
        return relative_to_project(path)
    except ValueError:
        return str(path)


def run_single_route(
    *,
    checkpoint_path: Path,
    route_path: Path,
    args: argparse.Namespace,
) -> dict[str, Any]:
    command = [
        sys.executable,
        "-m",
        "evaluation.pipelines.evaluate_pilotnet_loop",
        "--checkpoint",
        str(checkpoint_path),
        "--route-config",
        str(route_path),
        "--host",
        args.host,
        "--port",
        str(args.port),
        "--target-speed-kmh",
        str(args.target_speed_kmh),
        "--weather",
        args.weather,
        "--seed",
        str(args.seed),
        "--max-seconds",
        str(args.max_seconds),
        "--record-video" if args.record_video else "--no-record-video",
    ]
    completed = subprocess.run(command, capture_output=True, text=True, check=True)
    return json.loads(completed.stdout)


def build_aggregate(
    *,
    checkpoint_path: Path,
    route_dir: Path,
    args: argparse.Namespace,
    git_commit_id: str,
    results: list[dict[str, Any]],
    success_count: int,
    route_count: int,
) -> dict[str, Any]:
    return {
        "git_commit_id": git_commit_id,
        "checkpoint_path": display_path(checkpoint_path),
        "route_dir": display_path(route_dir),
        "route_glob": args.route_glob,
        "route_count": route_count,
        "success_count": success_count,
        "success_rate": success_count / route_count if route_count else 0.0,
        "record_video": args.record_video,
        "max_seconds": args.max_seconds,
        "results": results,
    }


def write_aggregate(output_path: Path, aggregate: dict[str, Any]) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as handle:
        json.dump(aggregate, handle, indent=2)
        handle.write("\n")


def main() -> None:
    args = build_parser().parse_args()
    git_commit_id = ensure_clean_git_worktree_for_evaluation()
    checkpoint_path = Path(args.checkpoint).resolve()
    route_dir = Path(args.route_dir).resolve()
    route_paths = load_route_paths(route_dir, args.route_glob, args.max_routes)
    if not route_paths:
        raise SystemExit(f"No route configs matched: {route_dir} / {args.route_glob}")

    output_path = build_output_path(args.summary_output, git_commit_id=git_commit_id)
    results: list[dict[str, Any]] = []
    success_count = 0
    for route_path in route_paths:
        summary = run_single_route(checkpoint_path=checkpoint_path, route_path=route_path, args=args)
        success_count += int(bool(summary["success"]))
        results.append(
            {
                "route_config_path": display_path(route_path),
                "route_name": summary["route_name"],
                "success": summary["success"],
                "route_completion_ratio": summary["route_completion_ratio"],
                "collision_count": summary["collision_count"],
                "elapsed_seconds": summary["elapsed_seconds"],
                "distance_to_goal_m": summary["distance_to_goal_m"],
                "summary_path": str((PROJECT_ROOT / summary["front_rgb_dir"]).parent / "summary.json"),
                "manifest_path": summary["manifest_path"],
                "video_path": summary["video_path"],
            }
        )
        aggregate = build_aggregate(
            checkpoint_path=checkpoint_path,
            route_dir=route_dir,
            args=args,
            git_commit_id=git_commit_id,
            results=results,
            success_count=success_count,
            route_count=len(route_paths),
        )
        write_aggregate(output_path, aggregate)

    aggregate = build_aggregate(
        checkpoint_path=checkpoint_path,
        route_dir=route_dir,
        args=args,
        git_commit_id=git_commit_id,
        results=results,
        success_count=success_count,
        route_count=len(route_paths),
    )
    write_aggregate(output_path, aggregate)
    print(json.dumps({"summary_output": str(output_path.relative_to(PROJECT_ROOT)), **{k: aggregate[k] for k in ("route_count", "success_count", "success_rate")}}, indent=2))


if __name__ == "__main__":
    main()
