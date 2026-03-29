from __future__ import annotations

from ad_stack.overtake.domain import LaneChangePlanPoint, LaneChangePlanResult


def build_route_aligned_lane_change_plan(
    origin_lane_samples: list[LaneChangePlanPoint],
    target_lane_samples: list[LaneChangePlanPoint],
    *,
    distance_same_lane_m: float,
    lane_change_distance_m: float,
    distance_other_lane_m: float,
) -> LaneChangePlanResult:
    if not origin_lane_samples or not target_lane_samples:
        return LaneChangePlanResult(False, [], "missing_lane_samples")
    start_progress_m = origin_lane_samples[0].progress_m
    for previous, current in zip(origin_lane_samples, origin_lane_samples[1:], strict=False):
        if current.progress_m < previous.progress_m:
            return LaneChangePlanResult(False, [], "origin_progress_not_monotonic")
    for previous, current in zip(target_lane_samples, target_lane_samples[1:], strict=False):
        if current.progress_m < previous.progress_m:
            return LaneChangePlanResult(False, [], "target_progress_not_monotonic")

    same_lane_end_m = start_progress_m + distance_same_lane_m
    lane_change_end_m = same_lane_end_m + lane_change_distance_m
    target_lane_end_m = lane_change_end_m + distance_other_lane_m

    plan_points = [
        sample for sample in origin_lane_samples if sample.progress_m <= same_lane_end_m + 1e-6
    ]
    target_points = [
        sample
        for sample in target_lane_samples
        if sample.progress_m >= lane_change_end_m - 1e-6
        and sample.progress_m <= target_lane_end_m + 1e-6
    ]
    if not target_points:
        return LaneChangePlanResult(False, plan_points, "adjacent_lane_sample_missing")
    plan_points.extend(target_points)
    for previous, current in zip(plan_points, plan_points[1:], strict=False):
        if current.progress_m < previous.progress_m:
            return LaneChangePlanResult(False, plan_points, "plan_progress_reversed")
    return LaneChangePlanResult(True, plan_points)
