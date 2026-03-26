from __future__ import annotations

from dataclasses import dataclass
from statistics import mean
from typing import Sequence

from evaluation.runners.scenario_runner import ScenarioResult


@dataclass(slots=True)
class SuiteSummary:
    scenario_count: int
    average_completion_ratio: float
    total_collisions: int
    total_lane_invasions: int


def build_suite_summary(results: Sequence[ScenarioResult]) -> SuiteSummary:
    if not results:
        return SuiteSummary(
            scenario_count=0,
            average_completion_ratio=0.0,
            total_collisions=0,
            total_lane_invasions=0,
        )
    return SuiteSummary(
        scenario_count=len(results),
        average_completion_ratio=mean(result.metrics.route_completion_ratio for result in results),
        total_collisions=sum(result.metrics.collision_count for result in results),
        total_lane_invasions=sum(result.metrics.lane_invasion_count for result in results),
    )
