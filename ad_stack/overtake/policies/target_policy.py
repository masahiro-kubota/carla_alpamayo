from __future__ import annotations

from typing import Protocol

from ad_stack.overtake.domain import OvertakeLeadSnapshot, OvertakeTargetSnapshot


class TargetPolicy(Protocol):
    def __call__(
        self,
        leads: list[OvertakeLeadSnapshot],
        *,
        cluster_merge_gap_m: float,
        cluster_max_member_speed_mps: float,
    ) -> list[OvertakeTargetSnapshot]: ...
