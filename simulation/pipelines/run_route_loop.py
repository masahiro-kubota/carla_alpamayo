from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

from ad_stack import run
from libs.project import PROJECT_ROOT, current_git_commit_short, ensure_clean_git_worktree
from simulation.pipelines.front_camera_preview import FrontCameraPreview, has_display
from simulation.pipelines.route_loop_run_config import (
    ROUTE_LOOP_ENTRYPOINT,
    load_route_loop_run_config,
    write_route_loop_run_metadata,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run one or more route-loop JSON configs. "
            "Every simulation setting must come from the config file; no CLI overrides are accepted."
        )
    )
    parser.add_argument(
        "config_paths", nargs="+", help="Path(s) to route-loop run config JSON files."
    )
    return parser


def _project_relative_or_absolute(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(PROJECT_ROOT))
    except ValueError:
        return str(path.resolve())


def _build_launcher_output_dir() -> Path:
    run_id = (
        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_route_loop_batch_{current_git_commit_short()}"
    )
    output_dir = PROJECT_ROOT / "outputs" / "launcher_runs" / run_id
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def _write_launcher_manifest(
    *,
    output_dir: Path,
    argv: list[str],
    loaded_configs: list[Any],
    worker_records: list[dict[str, Any]],
) -> Path:
    manifest_path = output_dir / "launcher_manifest.json"
    payload = {
        "entrypoint": ROUTE_LOOP_ENTRYPOINT,
        "argv": argv,
        "config_paths": [
            _project_relative_or_absolute(config.config_path) for config in loaded_configs
        ],
        "workers": worker_records,
    }
    manifest_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return manifest_path


def _run_single_config(config_path: Path, argv: list[str]) -> dict[str, Any]:
    loaded_config = load_route_loop_run_config(config_path)
    request = loaded_config.request
    preview: FrontCameraPreview | None = None
    preview_requested = loaded_config.preview.show_front_camera
    preview_enabled = preview_requested
    if preview_enabled and not has_display():
        print("DISPLAY is not set, disabling front camera preview. Export DISPLAY=:1 to enable it.")
        preview_enabled = False

    try:
        if preview_enabled:
            preview = FrontCameraPreview(
                source_width=request.runtime.camera_width,
                source_height=request.runtime.camera_height,
                display_scale=loaded_config.preview.preview_scale,
                title="CARLA Front Camera (Route Loop)",
            )

        def preview_sink(rgb_array, status_text: str) -> bool | None:
            if preview is not None and not preview.closed:
                preview.update(rgb_array, status_text)
                return True
            return not (preview is not None and preview.closed)

        request.policy.preview_sink = preview_sink if preview_enabled else None
        result = run(request)
        if result.output_dir is not None:
            cli_args_path, run_request_path, run_config_path = write_route_loop_run_metadata(
                output_dir=result.output_dir,
                argv=argv,
                request=request,
                config_path=loaded_config.config_path,
                config_payload=loaded_config.config_payload,
                summary_path=result.summary_path,
                preview_requested=preview_requested,
                preview_enabled=preview_enabled,
            )
            result.summary["cli_args_path"] = _project_relative_or_absolute(cli_args_path)
            result.summary["run_request_path"] = _project_relative_or_absolute(run_request_path)
            result.summary["run_config_path"] = _project_relative_or_absolute(run_config_path)
            result.summary["run_config_source_path"] = _project_relative_or_absolute(
                loaded_config.config_path
            )
            result.summary["front_camera_preview_requested"] = preview_requested
            result.summary["front_camera_preview_enabled"] = preview_enabled
    finally:
        if preview is not None:
            preview.close()

    print(json.dumps(result.summary, indent=2))
    return result.summary


def _run_parallel_configs(config_paths: list[Path], argv: list[str]) -> None:
    loaded_configs = [load_route_loop_run_config(path) for path in config_paths]
    launcher_dir = _build_launcher_output_dir()
    worker_records: list[dict[str, Any]] = []
    processes: list[subprocess.Popen] = []
    log_handles = []
    child_env = os.environ.copy()
    child_env["PYTHONPATH"] = ""

    try:
        for index, config in enumerate(loaded_configs, start=1):
            log_path = launcher_dir / f"worker_{index}.log"
            log_handle = log_path.open("w", encoding="utf-8")
            log_handles.append(log_handle)
            command = [sys.executable, "-m", ROUTE_LOOP_ENTRYPOINT, str(config.config_path)]
            process = subprocess.Popen(
                command,
                cwd=PROJECT_ROOT,
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                env=child_env,
            )
            processes.append(process)
            worker_records.append(
                {
                    "worker_id": index,
                    "name": config.name,
                    "config_path": _project_relative_or_absolute(config.config_path),
                    "port": config.request.runtime.port,
                    "route_config_path": str(config.request.scenario.route_config_path),
                    "log_path": _project_relative_or_absolute(log_path),
                    "pid": process.pid,
                }
            )

        manifest_path = _write_launcher_manifest(
            output_dir=launcher_dir,
            argv=argv,
            loaded_configs=loaded_configs,
            worker_records=worker_records,
        )

        for process, worker in zip(processes, worker_records, strict=True):
            worker["exit_code"] = process.wait()

        _write_launcher_manifest(
            output_dir=launcher_dir,
            argv=argv,
            loaded_configs=loaded_configs,
            worker_records=worker_records,
        )
    finally:
        for log_handle in log_handles:
            log_handle.close()

    print(
        json.dumps(
            {
                "launcher_manifest_path": _project_relative_or_absolute(manifest_path),
                "workers": worker_records,
            },
            indent=2,
        )
    )
    if any(worker["exit_code"] != 0 for worker in worker_records):
        raise SystemExit(1)


def main() -> None:
    args = build_parser().parse_args()
    ensure_clean_git_worktree(action_label="Route loop run")
    config_paths = [Path(path) for path in args.config_paths]
    if len(config_paths) == 1:
        _run_single_config(config_paths[0], sys.argv[1:])
        return
    _run_parallel_configs(config_paths, sys.argv[1:])


if __name__ == "__main__":
    main()
