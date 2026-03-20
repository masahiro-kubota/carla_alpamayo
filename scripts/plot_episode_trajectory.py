#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Plot an episode trajectory from manifest telemetry.")
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--output", required=True)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    manifest_path = Path(args.manifest).resolve()
    output_path = Path(args.output).resolve()

    rows = [json.loads(line) for line in manifest_path.open("r", encoding="utf-8")]
    telemetry = [
        (row["vehicle_x"], row["vehicle_y"], row.get("collision", False), row.get("lane_invasion", False))
        for row in rows
        if row.get("vehicle_x") is not None and row.get("vehicle_y") is not None
    ]
    if not telemetry:
        raise SystemExit("No vehicle_x/vehicle_y telemetry in manifest.")

    xs = [entry[0] for entry in telemetry]
    ys = [entry[1] for entry in telemetry]

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(xs, ys, color="#e67e22", linewidth=2.2, label="actual trajectory")
    ax.scatter([xs[0]], [ys[0]], color="#27ae60", s=40, zorder=3, label="start")
    ax.scatter([xs[-1]], [ys[-1]], color="#c0392b", s=40, zorder=3, label="end")

    collision_points = [(x, y) for x, y, collision, _ in telemetry if collision]
    if collision_points:
        ax.scatter(
            [point[0] for point in collision_points],
            [point[1] for point in collision_points],
            color="#8e44ad",
            s=30,
            zorder=4,
            label="collision",
        )

    lane_points = [(x, y) for x, y, _, lane_invasion in telemetry if lane_invasion]
    if lane_points:
        ax.scatter(
            [point[0] for point in lane_points],
            [point[1] for point in lane_points],
            color="#2980b9",
            s=10,
            alpha=0.7,
            zorder=2,
            label="lane invasion",
        )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    ax.set_title(manifest_path.parent.name)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.invert_yaxis()
    ax.grid(color="#ecf0f1", linewidth=0.5, alpha=0.6)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
