from __future__ import annotations

from ad_stack.overtake.domain import (
    OvertakeLeadSnapshot,
    OvertakeTargetSnapshot,
    TargetKind,
)


def build_stopped_obstacle_targets(
    leads: list[OvertakeLeadSnapshot],
    *,
    cluster_merge_gap_m: float,
    cluster_max_member_speed_mps: float,
) -> list[OvertakeTargetSnapshot]:
    stopped_leads = [
        lead
        for lead in leads
        if (
            lead.actor_id is not None
            and lead.lane_id is not None
            and lead.distance_m is not None
            and lead.is_stopped
        )
    ]
    stopped_leads.sort(key=lambda lead: (lead.lane_id or "", float(lead.distance_m or 0.0)))
    targets: list[OvertakeTargetSnapshot] = []
    current_cluster: list[OvertakeLeadSnapshot] = []
    for lead in stopped_leads:
        if not current_cluster:
            current_cluster = [lead]
            continue
        previous = current_cluster[-1]
        same_lane = previous.lane_id == lead.lane_id
        gap_m = float(lead.distance_m) - float(previous.distance_m)
        speed_ok = max(previous.speed_mps, lead.speed_mps) <= cluster_max_member_speed_mps
        if same_lane and gap_m <= cluster_merge_gap_m and speed_ok:
            current_cluster.append(lead)
            continue
        targets.append(_cluster_to_target(current_cluster))
        current_cluster = [lead]
    if current_cluster:
        targets.append(_cluster_to_target(current_cluster))
    return targets


def next_stopped_obstacle_target(
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
        speed_mps=max(lead.speed_mps for lead in cluster),
        is_stopped=all(lead.is_stopped for lead in cluster),
    )
