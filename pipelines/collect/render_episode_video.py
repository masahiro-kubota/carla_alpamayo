from __future__ import annotations

import argparse
import json
from pathlib import Path

from libs.carla_utils import relative_to_project
from libs.utils import render_png_sequence_to_mp4


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Render an MP4 from a collected episode's front RGB frames.")
    parser.add_argument("--episode-dir", required=True)
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--crf", type=int, default=23)
    parser.add_argument("--output-name", default="front_rgb.mp4")
    parser.add_argument(
        "--update-summary",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    episode_dir = Path(args.episode_dir).resolve()
    image_dir = episode_dir / "front_rgb"
    output_path = episode_dir / args.output_name
    summary_path = episode_dir / "summary.json"

    rendered_path = render_png_sequence_to_mp4(
        image_dir=image_dir,
        output_path=output_path,
        fps=args.fps,
        crf=args.crf,
    )

    result = {
        "episode_dir": relative_to_project(episode_dir),
        "video_path": relative_to_project(rendered_path),
        "video_fps": args.fps,
        "video_crf": args.crf,
    }

    if args.update_summary and summary_path.exists():
        with summary_path.open("r", encoding="utf-8") as handle:
            summary = json.load(handle)
        summary["video_path"] = result["video_path"]
        summary["video_fps"] = args.fps
        summary["video_crf"] = args.crf
        with summary_path.open("w", encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2)
            handle.write("\n")
        result["updated_summary_path"] = relative_to_project(summary_path)

    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
