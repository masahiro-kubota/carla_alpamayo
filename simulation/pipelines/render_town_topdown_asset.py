from __future__ import annotations

import argparse
from pathlib import Path

from libs.carla_utils import build_topdown_map_asset, require_carla


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture a CARLA town into a repo-managed top-down RGB PNG + metadata pair."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--town", default="Town01")
    parser.add_argument("--pixels-per-meter", type=float, default=8.0)
    parser.add_argument("--padding-m", type=float, default=20.0)
    parser.add_argument("--lane-sampling-m", type=float, default=2.0)
    parser.add_argument("--camera-height-m", type=float, default=1000.0)
    parser.add_argument(
        "--output-image",
        default=None,
        help="PNG output path. Defaults to scenarios/maps/<town>_topdown.png",
    )
    parser.add_argument(
        "--output-metadata",
        default=None,
        help="JSON output path. Defaults to scenarios/maps/<town>_topdown.json",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    carla = require_carla()
    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    world = client.get_world()
    active_town = world.get_map().name.split("/")[-1]
    if active_town != args.town:
        world = client.load_world(args.town)

    output_dir = Path("scenarios/maps")
    output_dir.mkdir(parents=True, exist_ok=True)
    output_image = Path(args.output_image) if args.output_image else output_dir / f"{args.town.lower()}_topdown.png"
    output_metadata = (
        Path(args.output_metadata)
        if args.output_metadata
        else output_dir / f"{args.town.lower()}_topdown.json"
    )

    asset = build_topdown_map_asset(
        world,
        output_image_path=output_image.resolve(),
        output_metadata_path=output_metadata.resolve(),
        pixels_per_meter=args.pixels_per_meter,
        padding_m=args.padding_m,
        lane_sampling_m=args.lane_sampling_m,
        camera_height_m=args.camera_height_m,
    )
    print(output_image)
    print(output_metadata)
    print(
        f"town={asset.town} size={asset.width}x{asset.height} "
        f"bounds=({asset.min_x},{asset.min_y})-({asset.max_x},{asset.max_y})"
    )


if __name__ == "__main__":
    main()
