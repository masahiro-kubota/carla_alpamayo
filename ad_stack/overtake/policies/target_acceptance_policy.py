from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol
from typing import Literal

from ad_stack.overtake.domain import OvertakeContext, OvertakeLeadSnapshot, OvertakeTargetSnapshot


@dataclass(frozen=True, slots=True)
class TargetAcceptanceRequest:
    context: OvertakeContext
    lead: OvertakeLeadSnapshot | None
    active_target: OvertakeTargetSnapshot | None
    target_distance_m: float | None
    target_speed_mps: float | None
    overtake_trigger_distance_m: float
    overtake_speed_delta_kmh: float


@dataclass(frozen=True, slots=True)
class TargetAcceptanceResult:
    accepted: bool
    reject_reason: str | None = None
    planner_state_on_reject: Literal["car_follow", "nominal_cruise"] = "car_follow"


class TargetAcceptancePolicy(Protocol):
    def __call__(self, request: TargetAcceptanceRequest) -> TargetAcceptanceResult: ...

