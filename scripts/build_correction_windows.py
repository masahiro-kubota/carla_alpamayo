#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build offline correction windows from failed movement-suite evaluation manifests."
    )
    parser.add_argument("--suite-summary", required=True)
    parser.add_argument("--output-dir", default=str(PROJECT_ROOT / "data" / "manifests" / "corrections"))
    parser.add_argument("--window-frames", type=int, default=125)
    parser.add_argument("--include-success", action=argparse.BooleanOptionalAction, default=False)
    return parser


def resolve_path(path_str: str) -> Path:
    path = Path(path_str)
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    return path.resolve()


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def load_jsonl(path: Path) -> list[dict]:
    records: list[dict] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            records.append(json.loads(line))
    return records


def sanitize_name(route_name: str) -> str:
    return route_name.replace("/", "_")


def main() -> None:
    args = build_parser().parse_args()
    suite_summary_path = resolve_path(args.suite_summary)
    suite_summary = load_json(suite_summary_path)

    output_dir = resolve_path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    built: list[dict] = []
    for result in suite_summary["results"]:
        if result["success"] and not args.include_success:
            continue

        summary_path = resolve_path(result["summary_path"])
        episode_summary = load_json(summary_path)
        manifest_path = resolve_path(episode_summary["manifest_path"])
        records = load_jsonl(manifest_path)
        if not records:
            continue

        window = records[-args.window_frames :]
        route_name = sanitize_name(result["route_name"])
        output_path = output_dir / f"{route_name}_window.jsonl"
        with output_path.open("w", encoding="utf-8") as handle:
            for record in window:
                handle.write(json.dumps(record, ensure_ascii=False))
                handle.write("\n")

        built.append(
            {
                "route_name": result["route_name"],
                "success": result["success"],
                "window_frame_count": len(window),
                "source_manifest_path": str(manifest_path.relative_to(PROJECT_ROOT)),
                "output_manifest_path": str(output_path.relative_to(PROJECT_ROOT)),
            }
        )

    print(
        json.dumps(
            {
                "suite_summary": str(suite_summary_path.relative_to(PROJECT_ROOT)),
                "built_count": len(built),
                "window_frames": args.window_frames,
                "artifacts": built,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
