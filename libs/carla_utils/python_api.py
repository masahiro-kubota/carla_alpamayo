from __future__ import annotations

from pathlib import Path
import sys


DEFAULT_CARLA_PYTHONAPI_ROOT = Path("/home/masa/sim/carla-0.9.16/PythonAPI")


def ensure_carla_agents_on_path(pythonapi_root: Path | None = None) -> Path:
    root = pythonapi_root or DEFAULT_CARLA_PYTHONAPI_ROOT
    agents_root = root / "carla"
    if not agents_root.exists():
        raise RuntimeError(
            f"CARLA agents path does not exist: {agents_root}. "
            "Confirm the simulator is installed at ~/sim/carla-0.9.16."
        )
    if str(agents_root) not in sys.path:
        sys.path.append(str(agents_root))
    return agents_root


def require_carla():
    try:
        import carla
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "The 'carla' Python package is not installed. "
            "Run 'uv sync' after confirming the CARLA wheel path in pyproject.toml."
        ) from exc
    return carla
