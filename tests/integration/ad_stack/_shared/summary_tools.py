from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def require(condition: bool, message: str) -> None:
    if not condition:
        raise SystemExit(message)


def summary_path_from_run_output(output_json: str) -> Path:
    payload = json.loads(output_json)
    return Path("outputs/evaluate") / payload["episode_id"] / "summary.json"


def load_summary(path: Path | str) -> dict[str, Any]:
    return json.loads(Path(path).read_text())


def load_manifest(summary: dict[str, Any]) -> list[dict[str, Any]]:
    return [
        json.loads(line)
        for line in Path(summary["manifest_path"]).read_text().splitlines()
        if line.strip()
    ]
