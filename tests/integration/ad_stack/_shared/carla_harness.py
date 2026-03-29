from __future__ import annotations

import json
import os
import signal
import socket
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

from .summary_tools import summary_path_from_run_output


@dataclass(frozen=True, slots=True)
class CarlaHarnessConfig:
    root_dir: Path
    carla_root: Path
    launch_args: tuple[str, ...]
    ports: tuple[int, ...] = (2000, 2001, 2002)
    warmup_seconds: float = 2.0
    startup_timeout_seconds: float = 60.0
    shutdown_timeout_seconds: float = 30.0
    scenario_max_attempts: int = 2
    log_path: Path = Path("/tmp/carla_integration_harness.log")


class CarlaHarness:
    def __init__(self, config: CarlaHarnessConfig) -> None:
        self._config = config
        self._process: subprocess.Popen[str] | None = None
        self._log_handle: Any | None = None

    def run_route_loop(self, run_config: str | Path) -> Path:
        run_config_str = str(run_config)
        for attempt in range(1, self._config.scenario_max_attempts + 1):
            print(
                f"==> running {run_config_str} "
                f"(attempt {attempt}/{self._config.scenario_max_attempts})",
                flush=True,
            )
            self.start()
            completed = self._run_command(
                [
                    "uv",
                    "run",
                    "python",
                    "-m",
                    "simulation.pipelines.run_route_loop",
                    run_config_str,
                ]
            )
            print(completed.stdout, end="", flush=True)
            self.stop()
            if completed.returncode == 0:
                summary_path = summary_path_from_run_output(completed.stdout)
                if summary_path.is_file():
                    print(f"    summary: {summary_path}", flush=True)
                    return summary_path
                print(f"missing summary after successful run: {run_config_str}", flush=True)
            else:
                print(
                    f"scenario failed with exit code {completed.returncode}: {run_config_str}",
                    flush=True,
                )
            if attempt == self._config.scenario_max_attempts:
                raise RuntimeError(f"failed to execute scenario: {run_config_str}")
            print(f"retrying {run_config_str}", flush=True)
        raise RuntimeError(f"failed to execute scenario: {run_config_str}")

    def run_json_command(self, args: Sequence[str]) -> dict[str, Any]:
        self.start()
        completed = self._run_command(args)
        print(completed.stdout, end="", flush=True)
        self.stop()
        if completed.returncode != 0:
            raise RuntimeError(f"command failed with exit code {completed.returncode}: {' '.join(args)}")
        return json.loads(completed.stdout)

    def start(self) -> None:
        if self._ports_are_in_use():
            raise RuntimeError(
                f"CARLA ports are already in use: {', '.join(str(port) for port in self._config.ports)}"
            )
        self._log_handle = self._config.log_path.open("w", encoding="utf-8")
        self._process = subprocess.Popen(
            [str(self._config.carla_root / "CarlaUE4.sh"), *self._config.launch_args],
            cwd=self._config.carla_root,
            stdout=self._log_handle,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,
        )
        if not self._wait_for_ports(open_expected=True, timeout_s=self._config.startup_timeout_seconds):
            self.stop()
            raise RuntimeError("timed out waiting for CARLA to start")
        time.sleep(self._config.warmup_seconds)

    def stop(self) -> None:
        if self._process is not None and self._process.poll() is None:
            try:
                os.killpg(os.getpgid(self._process.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                self._process.wait(timeout=self._config.shutdown_timeout_seconds)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(self._process.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self._process.wait(timeout=5)
        self._process = None
        if self._log_handle is not None:
            self._log_handle.close()
            self._log_handle = None
        self._wait_for_ports(open_expected=False, timeout_s=self._config.shutdown_timeout_seconds)

    def _run_command(self, args: Sequence[str]) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            list(args),
            cwd=self._config.root_dir,
            text=True,
            capture_output=True,
            env={**os.environ, "PYTHONPATH": ""},
            check=False,
        )

    def _ports_are_in_use(self) -> bool:
        return any(self._port_open(port) for port in self._config.ports)

    def _wait_for_ports(self, *, open_expected: bool, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            port_state = self._ports_are_in_use()
            if port_state is open_expected:
                return True
            time.sleep(1.0)
        return False

    @staticmethod
    def _port_open(port: int) -> bool:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(0.25)
            return sock.connect_ex(("127.0.0.1", port)) == 0
