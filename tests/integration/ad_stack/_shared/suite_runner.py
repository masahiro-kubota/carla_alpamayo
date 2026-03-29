from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

from .carla_harness import CarlaHarness


@dataclass(frozen=True, slots=True)
class RouteLoopSuiteCase:
    name: str
    run_config_path: Path


@dataclass(frozen=True, slots=True)
class InspectOnlySuiteCase:
    name: str
    command: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class CarlaIntegrationSuiteSpec:
    name: str
    route_loop_cases: tuple[RouteLoopSuiteCase, ...]
    inspect_only_cases: tuple[InspectOnlySuiteCase, ...] = ()


@dataclass(frozen=True, slots=True)
class CarlaIntegrationSuiteResult:
    summary_paths: dict[str, Path]
    inspection_payloads: dict[str, dict[str, Any]]


def execute_carla_integration_suite(
    harness: CarlaHarness,
    suite_spec: CarlaIntegrationSuiteSpec,
) -> CarlaIntegrationSuiteResult:
    summary_paths = {
        case.name: harness.run_route_loop(case.run_config_path)
        for case in suite_spec.route_loop_cases
    }
    inspection_payloads = {
        case.name: harness.run_json_command(case.command)
        for case in suite_spec.inspect_only_cases
    }
    return CarlaIntegrationSuiteResult(
        summary_paths=summary_paths,
        inspection_payloads=inspection_payloads,
    )
