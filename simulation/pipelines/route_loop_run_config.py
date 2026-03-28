from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Any

from ad_stack import ArtifactSpec, PolicySpec, RouteLoopScenarioSpec, RunRequest, RuntimeSpec
from libs.project import PROJECT_ROOT, relative_to_project

ROUTE_LOOP_ENTRYPOINT = "simulation.pipelines.run_route_loop"

_TOP_LEVEL_KEYS = {"name", "description", "mode", "scenario", "runtime", "policy", "artifacts", "preview"}
_TOP_LEVEL_REQUIRED_KEYS = {"mode", "scenario", "runtime", "policy", "artifacts", "preview"}
_SCENARIO_KEYS = {
    "route_config_path",
    "weather",
    "goal_tolerance_m",
    "max_stop_seconds",
    "stationary_speed_threshold_mps",
    "max_seconds",
    "environment_config_path",
}
_RUNTIME_KEYS = {
    "host",
    "port",
    "vehicle_filter",
    "fixed_delta_seconds",
    "sensor_timeout",
    "camera_width",
    "camera_height",
    "camera_fov",
    "target_speed_kmh",
    "seed",
}
_POLICY_KEYS = {
    "kind",
    "checkpoint_path",
    "expert_config_path",
    "device",
    "steer_smoothing",
    "max_steer_delta",
    "ignore_traffic_lights",
    "ignore_stop_signs",
    "ignore_vehicles",
}
_ARTIFACT_KEYS = {
    "record_video",
    "video_crf",
    "video_fps",
    "record_mcap",
    "mcap_jpeg_quality",
    "mcap_segment_seconds",
    "record_hz",
    "mcap_map_scope",
}
_PREVIEW_KEYS = {"show_front_camera", "preview_scale"}


@dataclass(slots=True)
class RouteLoopPreviewSpec:
    show_front_camera: bool
    preview_scale: float


@dataclass(slots=True)
class LoadedRouteLoopRunConfig:
    config_path: Path
    request: RunRequest
    preview: RouteLoopPreviewSpec
    config_payload: dict[str, Any]
    name: str | None = None
    description: str = ""


def _project_relative_or_absolute(path: Path) -> str:
    try:
        return relative_to_project(path)
    except ValueError:
        return str(path.resolve())


def resolve_user_path(raw_path: str | Path) -> Path:
    path = Path(raw_path)
    if path.is_absolute():
        return path.resolve()
    return (PROJECT_ROOT / path).resolve()


def _load_json_object(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise FileNotFoundError(f"Run config not found: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON in run config {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"Run config must contain a JSON object at the top level: {path}")
    return payload


def _validate_keys(
    section_name: str,
    payload: dict[str, Any],
    *,
    allowed_keys: set[str],
    required_keys: set[str],
) -> None:
    unknown_keys = sorted(set(payload) - allowed_keys)
    if unknown_keys:
        raise ValueError(f"{section_name} has unsupported keys: {', '.join(unknown_keys)}")
    missing_keys = sorted(required_keys - set(payload))
    if missing_keys:
        raise ValueError(f"{section_name} is missing required keys: {', '.join(missing_keys)}")


def _require_object(payload: dict[str, Any], key: str) -> dict[str, Any]:
    value = payload.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"{key} must be a JSON object.")
    return value


def _require_string(payload: dict[str, Any], key: str) -> str:
    value = payload.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{key} must be a non-empty string.")
    return value


def _require_optional_string(payload: dict[str, Any], key: str) -> str | None:
    value = payload.get(key)
    if value is None:
        return None
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{key} must be a non-empty string or null.")
    return value


def _require_bool(payload: dict[str, Any], key: str) -> bool:
    value = payload.get(key)
    if type(value) is not bool:
        raise ValueError(f"{key} must be a boolean.")
    return value


def _require_int(payload: dict[str, Any], key: str) -> int:
    value = payload.get(key)
    if type(value) is bool or not isinstance(value, int):
        raise ValueError(f"{key} must be an integer.")
    return value


def _require_float(payload: dict[str, Any], key: str) -> float:
    value = payload.get(key)
    if type(value) is bool or not isinstance(value, (int, float)):
        raise ValueError(f"{key} must be a number.")
    return float(value)


def _require_optional_float(payload: dict[str, Any], key: str) -> float | None:
    value = payload.get(key)
    if value is None:
        return None
    if type(value) is bool or not isinstance(value, (int, float)):
        raise ValueError(f"{key} must be a number or null.")
    return float(value)


def _require_existing_path(payload: dict[str, Any], key: str, *, allow_none: bool) -> Path | None:
    raw_path = payload.get(key)
    if raw_path is None:
        if allow_none:
            return None
        raise ValueError(f"{key} must be a string path.")
    if not isinstance(raw_path, str) or not raw_path.strip():
        raise ValueError(f"{key} must be a non-empty string path.")
    resolved_path = resolve_user_path(raw_path)
    if not resolved_path.exists():
        raise FileNotFoundError(f"{key} does not exist: {resolved_path}")
    return Path(raw_path)


def _build_request_from_payload(payload: dict[str, Any]) -> tuple[RunRequest, RouteLoopPreviewSpec]:
    mode = _require_string(payload, "mode")
    if mode != "evaluate":
        raise ValueError(f"mode must be 'evaluate' for route-loop runs, got {mode!r}")

    scenario_payload = _require_object(payload, "scenario")
    runtime_payload = _require_object(payload, "runtime")
    policy_payload = _require_object(payload, "policy")
    artifacts_payload = _require_object(payload, "artifacts")
    preview_payload = _require_object(payload, "preview")

    _validate_keys("scenario", scenario_payload, allowed_keys=_SCENARIO_KEYS, required_keys=_SCENARIO_KEYS)
    _validate_keys("runtime", runtime_payload, allowed_keys=_RUNTIME_KEYS, required_keys=_RUNTIME_KEYS)
    _validate_keys("policy", policy_payload, allowed_keys=_POLICY_KEYS, required_keys=_POLICY_KEYS)
    _validate_keys("artifacts", artifacts_payload, allowed_keys=_ARTIFACT_KEYS, required_keys=_ARTIFACT_KEYS)
    _validate_keys("preview", preview_payload, allowed_keys=_PREVIEW_KEYS, required_keys=_PREVIEW_KEYS)

    route_config_path = _require_existing_path(scenario_payload, "route_config_path", allow_none=False)
    environment_config_path = _require_existing_path(scenario_payload, "environment_config_path", allow_none=True)
    expert_config_path = _require_existing_path(policy_payload, "expert_config_path", allow_none=True)
    checkpoint_path = _require_existing_path(policy_payload, "checkpoint_path", allow_none=True)

    policy_kind = _require_string(policy_payload, "kind")
    if policy_kind not in {"expert", "learned"}:
        raise ValueError(f"policy.kind must be 'expert' or 'learned', got {policy_kind!r}")
    if policy_kind == "learned" and checkpoint_path is None:
        raise ValueError("policy.checkpoint_path must be set when policy.kind='learned'.")
    if policy_kind == "expert" and checkpoint_path is not None:
        raise ValueError("policy.checkpoint_path must be null when policy.kind='expert'.")
    if expert_config_path is None:
        raise ValueError("policy.expert_config_path must be set.")

    mcap_map_scope = _require_string(artifacts_payload, "mcap_map_scope")
    if mcap_map_scope not in {"full", "near_route"}:
        raise ValueError(f"artifacts.mcap_map_scope must be 'full' or 'near_route', got {mcap_map_scope!r}")

    request = RunRequest(
        mode="evaluate",
        scenario=RouteLoopScenarioSpec(
            route_config_path=route_config_path,
            weather=_require_string(scenario_payload, "weather"),
            goal_tolerance_m=_require_float(scenario_payload, "goal_tolerance_m"),
            max_stop_seconds=_require_float(scenario_payload, "max_stop_seconds"),
            stationary_speed_threshold_mps=_require_float(scenario_payload, "stationary_speed_threshold_mps"),
            max_seconds=_require_float(scenario_payload, "max_seconds"),
            environment_config_path=environment_config_path,
        ),
        runtime=RuntimeSpec(
            host=_require_string(runtime_payload, "host"),
            port=_require_int(runtime_payload, "port"),
            vehicle_filter=_require_string(runtime_payload, "vehicle_filter"),
            fixed_delta_seconds=_require_float(runtime_payload, "fixed_delta_seconds"),
            sensor_timeout=_require_float(runtime_payload, "sensor_timeout"),
            camera_width=_require_int(runtime_payload, "camera_width"),
            camera_height=_require_int(runtime_payload, "camera_height"),
            camera_fov=_require_int(runtime_payload, "camera_fov"),
            target_speed_kmh=_require_optional_float(runtime_payload, "target_speed_kmh"),
            seed=_require_int(runtime_payload, "seed"),
        ),
        policy=PolicySpec(
            kind=policy_kind,
            checkpoint_path=checkpoint_path,
            expert_config_path=expert_config_path,
            device=_require_optional_string(policy_payload, "device"),
            steer_smoothing=_require_float(policy_payload, "steer_smoothing"),
            max_steer_delta=_require_optional_float(policy_payload, "max_steer_delta"),
            ignore_traffic_lights=_require_bool(policy_payload, "ignore_traffic_lights"),
            ignore_stop_signs=_require_bool(policy_payload, "ignore_stop_signs"),
            ignore_vehicles=_require_bool(policy_payload, "ignore_vehicles"),
        ),
        artifacts=ArtifactSpec(
            record_video=_require_bool(artifacts_payload, "record_video"),
            video_crf=_require_int(artifacts_payload, "video_crf"),
            video_fps=_require_optional_float(artifacts_payload, "video_fps"),
            record_mcap=_require_bool(artifacts_payload, "record_mcap"),
            mcap_jpeg_quality=_require_int(artifacts_payload, "mcap_jpeg_quality"),
            mcap_segment_seconds=_require_float(artifacts_payload, "mcap_segment_seconds"),
            record_hz=_require_float(artifacts_payload, "record_hz"),
            mcap_map_scope=mcap_map_scope,
        ),
    )
    preview = RouteLoopPreviewSpec(
        show_front_camera=_require_bool(preview_payload, "show_front_camera"),
        preview_scale=_require_float(preview_payload, "preview_scale"),
    )
    return request, preview


def load_route_loop_run_config(config_path: Path) -> LoadedRouteLoopRunConfig:
    resolved_config_path = resolve_user_path(config_path)
    payload = _load_json_object(resolved_config_path)
    _validate_keys(
        "run config",
        payload,
        allowed_keys=_TOP_LEVEL_KEYS,
        required_keys=_TOP_LEVEL_REQUIRED_KEYS,
    )
    request, preview = _build_request_from_payload(payload)
    name = payload.get("name")
    if name is not None and (not isinstance(name, str) or not name.strip()):
        raise ValueError("name must be a non-empty string when set.")
    description = payload.get("description", "")
    if not isinstance(description, str):
        raise ValueError("description must be a string when set.")
    return LoadedRouteLoopRunConfig(
        config_path=resolved_config_path,
        request=request,
        preview=preview,
        config_payload=payload,
        name=name,
        description=description,
    )


def _jsonable(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def build_resolved_request_payload(request: RunRequest) -> dict[str, object]:
    return {
        "mode": request.mode,
        "scenario": _jsonable(asdict(request.scenario)),
        "runtime": _jsonable(asdict(request.runtime)),
        "policy": {
            "kind": request.policy.kind,
            "checkpoint_path": str(request.policy.checkpoint_path) if request.policy.checkpoint_path else None,
            "expert_config_path": str(request.policy.expert_config_path) if request.policy.expert_config_path else None,
            "device": request.policy.device,
            "steer_smoothing": request.policy.steer_smoothing,
            "max_steer_delta": request.policy.max_steer_delta,
            "ignore_traffic_lights": request.policy.ignore_traffic_lights,
            "ignore_stop_signs": request.policy.ignore_stop_signs,
            "ignore_vehicles": request.policy.ignore_vehicles,
            "preview_enabled": request.policy.preview_sink is not None,
        },
        "artifacts": _jsonable(asdict(request.artifacts)),
    }


def write_route_loop_run_metadata(
    *,
    output_dir: Path,
    argv: list[str],
    request: RunRequest,
    config_path: Path,
    config_payload: dict[str, Any],
    summary_path: Path | None,
    preview_requested: bool,
    preview_enabled: bool,
) -> tuple[Path, Path, Path]:
    cli_args_path = output_dir / "cli_args.json"
    run_request_path = output_dir / "run_request.json"
    run_config_path = output_dir / "run_config.json"

    cli_args_payload = {
        "entrypoint": ROUTE_LOOP_ENTRYPOINT,
        "argv": argv,
        "config_path": _project_relative_or_absolute(config_path),
    }
    cli_args_path.write_text(json.dumps(cli_args_payload, indent=2) + "\n", encoding="utf-8")
    run_request_path.write_text(
        json.dumps(build_resolved_request_payload(request), indent=2) + "\n",
        encoding="utf-8",
    )
    run_config_path.write_text(json.dumps(config_payload, indent=2) + "\n", encoding="utf-8")

    if summary_path is not None and summary_path.exists():
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        summary["cli_args_path"] = _project_relative_or_absolute(cli_args_path)
        summary["run_request_path"] = _project_relative_or_absolute(run_request_path)
        summary["run_config_path"] = _project_relative_or_absolute(run_config_path)
        summary["run_config_source_path"] = _project_relative_or_absolute(config_path)
        summary["front_camera_preview_requested"] = preview_requested
        summary["front_camera_preview_enabled"] = preview_enabled
        summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    return cli_args_path, run_request_path, run_config_path
