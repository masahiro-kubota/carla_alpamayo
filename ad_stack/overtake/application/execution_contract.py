from __future__ import annotations

from dataclasses import dataclass
from typing import Literal


ExecutionActivationOutcome = Literal[
    "activated",
    "unavailable",
    "fallback_trace_activated",
    "base_route_resumed",
    "missing_base_trace",
]


@dataclass(frozen=True, slots=True)
class LaneChangePathStatus:
    available: bool
    failure_reason: str | None = None


@dataclass(frozen=True, slots=True)
class ExecutionActivationResult:
    outcome: ExecutionActivationOutcome
    target_lane_id: str | None
    lane_change_path: LaneChangePathStatus

    @property
    def activated(self) -> bool:
        return self.outcome in {
            "activated",
            "fallback_trace_activated",
            "base_route_resumed",
        }
