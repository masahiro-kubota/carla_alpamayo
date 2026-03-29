from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

from ad_stack.overtake.domain import OvertakeMemory, OvertakeTargetSnapshot


@dataclass(slots=True)
class OvertakeRuntimeState:
    state: str = "idle"
    direction: Literal["left", "right"] | None = None
    origin_lane_id: str | None = None
    target_actor_id: int | None = None
    target_member_actor_ids: tuple[int, ...] = ()
    aborted: bool = False
    memory: OvertakeMemory = field(default_factory=OvertakeMemory)

    def reset(self) -> None:
        self.state = "idle"
        self.direction = None
        self.origin_lane_id = None
        self.target_actor_id = None
        self.target_member_actor_ids = ()
        self.aborted = False
        self.memory = OvertakeMemory()

    def begin_lane_change_out(
        self,
        *,
        direction: Literal["left", "right"],
        origin_lane_id: str | None,
        active_target: OvertakeTargetSnapshot | None,
        lead_actor_id: int | None,
        lead_lane_id: str | None,
        lead_distance_m: float | None,
        target_lane_id: str | None,
    ) -> None:
        self.state = "lane_change_out"
        self.direction = direction
        self.origin_lane_id = origin_lane_id
        self.aborted = False
        self.target_actor_id = (
            active_target.primary_actor_id if active_target is not None else lead_actor_id
        )
        self.target_member_actor_ids = (
            active_target.member_actor_ids
            if active_target is not None
            else (lead_actor_id,) if lead_actor_id is not None else ()
        )
        self.memory = OvertakeMemory(
            state="lane_change_out",
            direction=direction,
            origin_lane_id=origin_lane_id,
            target_lane_id=target_lane_id,
            target_actor_id=self.target_actor_id,
            target_actor_lane_id=active_target.lane_id if active_target is not None else lead_lane_id,
            target_kind=active_target.kind if active_target is not None else "single_actor",
            target_member_actor_ids=self.target_member_actor_ids,
            target_exit_distance_m=(
                active_target.exit_distance_m if active_target is not None else lead_distance_m
            ),
        )
