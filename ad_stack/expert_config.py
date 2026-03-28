from __future__ import annotations

import json
from dataclasses import asdict, fields, replace
from pathlib import Path
from typing import Any

from ad_stack.agents import ExpertBasicAgentConfig
from libs.project import PROJECT_ROOT

DEFAULT_EXPERT_CONFIG_PATH = Path("ad_stack/configs/expert/default.json")


def resolve_expert_config_path(path: str | Path | None = None) -> Path:
    candidate = Path(path) if path is not None else DEFAULT_EXPERT_CONFIG_PATH
    if not candidate.is_absolute():
        candidate = PROJECT_ROOT / candidate
    return candidate.resolve()


def load_expert_config(path: str | Path | None = None) -> tuple[ExpertBasicAgentConfig, Path]:
    resolved_path = resolve_expert_config_path(path)
    raw = json.loads(resolved_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"Expert config must be a JSON object: {resolved_path}")

    known_fields = {field.name for field in fields(ExpertBasicAgentConfig)}
    unknown_keys = sorted(set(raw) - known_fields)
    if unknown_keys:
        unknown_display = ", ".join(unknown_keys)
        raise ValueError(f"Unknown expert config keys in {resolved_path}: {unknown_display}")

    return replace(ExpertBasicAgentConfig(), **raw), resolved_path


def bind_runtime_overrides(
    config: ExpertBasicAgentConfig,
    *,
    target_speed_kmh: float | None,
    ignore_traffic_lights: bool,
    ignore_stop_signs: bool,
    ignore_vehicles: bool,
    sampling_resolution_m: float,
) -> ExpertBasicAgentConfig:
    updated = replace(
        config,
        ignore_traffic_lights=ignore_traffic_lights,
        ignore_stop_signs=ignore_stop_signs,
        ignore_vehicles=ignore_vehicles,
        sampling_resolution_m=sampling_resolution_m,
    )
    if target_speed_kmh is not None:
        updated = replace(updated, target_speed_kmh=target_speed_kmh)
    return updated


def expert_config_to_dict(config: ExpertBasicAgentConfig) -> dict[str, Any]:
    return asdict(config)
