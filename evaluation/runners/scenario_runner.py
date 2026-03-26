from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable

from ad_stack.agents.base import AutonomyAgent
from ad_stack.world_model.scene_state import SceneState
from evaluation.metrics.route_metrics import RouteMetrics, RouteMetricsAccumulator


@dataclass(slots=True)
class ScenarioResult:
    scenario_id: str
    step_count: int
    metrics: RouteMetrics
    debug: dict[str, Any] = field(default_factory=dict)


class ScenarioRunner:
    """Generic runner for feeding scene-state streams through an AD stack agent."""

    def run(self, scenario_id: str, scene_stream: Iterable[SceneState], agent: AutonomyAgent) -> ScenarioResult:
        accumulator = RouteMetricsAccumulator()
        step_count = 0
        last_scene: SceneState | None = None
        agent.reset()
        for scene_state in scene_stream:
            decision = agent.step(scene_state)
            accumulator.observe(scene_state, decision)
            last_scene = scene_state
            step_count += 1
        return ScenarioResult(
            scenario_id=scenario_id,
            step_count=step_count,
            metrics=accumulator.snapshot(last_scene),
        )
