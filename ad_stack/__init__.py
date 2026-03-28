"""Public single-entrypoint facade for the online AD stack."""

from __future__ import annotations

from importlib import import_module
from typing import TYPE_CHECKING, Any

__all__ = [
    "ArtifactSpec",
    "InteractiveScenarioSpec",
    "PolicySpec",
    "RouteLoopScenarioSpec",
    "RunRequest",
    "RunResult",
    "RuntimeSpec",
    "run",
]

if TYPE_CHECKING:
    from ad_stack.run import (
        ArtifactSpec,
        InteractiveScenarioSpec,
        PolicySpec,
        RouteLoopScenarioSpec,
        RunRequest,
        RunResult,
        RuntimeSpec,
        run,
    )


def __getattr__(name: str) -> Any:
    if name not in __all__:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    route_run = import_module("ad_stack.run")
    return getattr(route_run, name)
