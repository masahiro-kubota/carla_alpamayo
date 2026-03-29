from __future__ import annotations

from ad_stack.overtake.domain.models import AdjacentLaneGapSnapshot


def lane_gap_for_lane_id(
    lane_gaps_by_lane_id: dict[str, AdjacentLaneGapSnapshot], lane_id: str | None
) -> AdjacentLaneGapSnapshot | None:
    if lane_id is None:
        return None
    return lane_gaps_by_lane_id.get(lane_id)
