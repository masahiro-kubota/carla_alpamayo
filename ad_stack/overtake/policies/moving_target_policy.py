from __future__ import annotations

from ad_stack.overtake.domain import (
    OvertakeLeadSnapshot,
    OvertakeTargetSnapshot,
    TargetKind,
)


def build_moving_overtake_targets(
    leads: list[OvertakeLeadSnapshot],
    *,
    max_target_speed_mps: float,
    min_relative_speed_mps: float,
    max_distance_m: float,
    cluster_merge_gap_m: float,
    cluster_max_member_speed_delta_mps: float,
) -> list[OvertakeTargetSnapshot]:
    moving_leads = [
        lead
        for lead in leads
        if (
            lead.actor_id is not None
            and lead.lane_id is not None
            and lead.distance_m is not None
            and not lead.is_stopped
            and float(lead.speed_mps) <= max_target_speed_mps
            and float(lead.relative_speed_mps) >= min_relative_speed_mps
            and float(lead.distance_m) <= max_distance_m
        )
    ]
    moving_leads.sort(key=lambda lead: (lead.lane_id or "", float(lead.distance_m or 0.0)))

    targets: list[OvertakeTargetSnapshot] = []
    current_cluster: list[OvertakeLeadSnapshot] = []
    for lead in moving_leads:
        if not current_cluster:
            current_cluster = [lead]
            continue
        previous = current_cluster[-1]
        same_lane = previous.lane_id == lead.lane_id
        gap_m = float(lead.distance_m) - float(previous.distance_m)
        speed_delta_mps = abs(float(lead.speed_mps) - float(previous.speed_mps))
        if (
            same_lane
            and gap_m <= cluster_merge_gap_m
            and speed_delta_mps <= cluster_max_member_speed_delta_mps
        ):
            current_cluster.append(lead)
            continue
        targets.append(_cluster_to_target(current_cluster))
        current_cluster = [lead]

    if current_cluster:
        targets.append(_cluster_to_target(current_cluster))
    return targets


def next_moving_overtake_target(
    targets: list[OvertakeTargetSnapshot],
    *,
    current_primary_actor_id: int | None,
) -> OvertakeTargetSnapshot | None:
    if current_primary_actor_id is None:
        return targets[0] if targets else None
    for index, target in enumerate(targets):
        if target.primary_actor_id == current_primary_actor_id:
            return targets[index + 1] if index + 1 < len(targets) else None
    return targets[0] if targets else None


def _cluster_to_target(cluster: list[OvertakeLeadSnapshot]) -> OvertakeTargetSnapshot:
    first = cluster[0]
    last = cluster[-1]
    member_actor_ids = tuple(lead.actor_id for lead in cluster if lead.actor_id is not None)
    kind: TargetKind = "cluster" if len(member_actor_ids) > 1 else "single_actor"
    return OvertakeTargetSnapshot(
        kind=kind,
        primary_actor_id=int(first.actor_id),
        member_actor_ids=member_actor_ids,
        lane_id=str(first.lane_id),
        entry_distance_m=float(first.distance_m),
        exit_distance_m=float(last.distance_m),
        speed_mps=max(float(lead.speed_mps) for lead in cluster),
        is_stopped=False,
    )
