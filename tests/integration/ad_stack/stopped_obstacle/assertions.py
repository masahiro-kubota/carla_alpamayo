from __future__ import annotations

from pathlib import Path
from typing import Any

from tests.integration.ad_stack._shared import load_manifest, load_summary, require

SCENARIO_ORDER = (
    "clear",
    "blocked_static",
    "blocked_oncoming",
    "double_stopped_separated",
    "double_stopped_clustered",
    "signal_suppressed",
    "near_junction_preflight_reject",
    "adjacent_lane_closed",
    "curve_clear",
    "rejoin_blocked_then_release",
)


def load_stopped_obstacle_summaries(
    summary_paths: list[str] | tuple[str, ...] | list[Path] | tuple[Path, ...],
) -> dict[str, dict[str, Any]]:
    require(
        len(summary_paths) == len(SCENARIO_ORDER),
        f"expected {len(SCENARIO_ORDER)} summary paths, got {len(summary_paths)}",
    )
    return {
        scenario_name: load_summary(path)
        for scenario_name, path in zip(SCENARIO_ORDER, summary_paths, strict=True)
    }


def assert_stopped_obstacle_suite(summaries: dict[str, dict[str, Any]]) -> None:
    clear_summary = summaries["clear"]
    require(bool(clear_summary["success"]), "clear scenario did not succeed")
    require(clear_summary["collision_count"] == 0, "clear scenario collided")
    require(clear_summary["overtake_attempt_count"] >= 1, "clear scenario never attempted overtake")
    require(clear_summary["overtake_success_count"] >= 1, "clear scenario never completed overtake")

    blocked_static_summary = summaries["blocked_static"]
    require(not blocked_static_summary["success"], "blocked_static unexpectedly succeeded")
    require(
        blocked_static_summary["failure_reason"] == "stalled",
        f"blocked_static failure_reason unexpected: {blocked_static_summary['failure_reason']}",
    )
    require(blocked_static_summary["collision_count"] == 0, "blocked_static collided")
    require(
        blocked_static_summary["overtake_attempt_count"] == 0,
        "blocked_static unexpectedly attempted overtake",
    )
    require(
        blocked_static_summary["unsafe_lane_change_reject_count"] >= 1,
        "blocked_static never recorded unsafe lane change reject",
    )

    blocked_oncoming_summary = summaries["blocked_oncoming"]
    require(bool(blocked_oncoming_summary["success"]), "blocked_oncoming did not succeed")
    require(blocked_oncoming_summary["collision_count"] == 0, "blocked_oncoming collided")
    require(
        blocked_oncoming_summary["unsafe_lane_change_reject_count"] >= 1,
        "blocked_oncoming never rejected initial overtake",
    )
    require(
        blocked_oncoming_summary["overtake_attempt_count"] >= 1,
        "blocked_oncoming never attempted overtake after waiting",
    )
    require(
        blocked_oncoming_summary["overtake_success_count"] >= 1,
        "blocked_oncoming never completed overtake",
    )

    double_stopped_separated_summary = summaries["double_stopped_separated"]
    require(bool(double_stopped_separated_summary["success"]), "double_stopped_separated did not succeed")
    require(
        double_stopped_separated_summary["collision_count"] == 0,
        "double_stopped_separated collided",
    )
    require(
        double_stopped_separated_summary["overtake_attempt_count"] >= 2,
        "double_stopped_separated never attempted two overtakes",
    )
    require(
        double_stopped_separated_summary["overtake_success_count"] >= 2,
        "double_stopped_separated never completed two overtakes",
    )
    separated_manifest = load_manifest(double_stopped_separated_summary)
    separated_targets = [
        row["overtake_target_actor_id"]
        for row in separated_manifest
        if row.get("overtake_target_actor_id") is not None
    ]
    require(
        len(set(separated_targets)) >= 2,
        "double_stopped_separated never switched target actor",
    )

    double_stopped_clustered_summary = summaries["double_stopped_clustered"]
    require(bool(double_stopped_clustered_summary["success"]), "double_stopped_clustered did not succeed")
    require(
        double_stopped_clustered_summary["collision_count"] == 0,
        "double_stopped_clustered collided",
    )
    require(
        double_stopped_clustered_summary["overtake_attempt_count"] >= 1,
        "double_stopped_clustered never attempted overtake",
    )
    require(
        double_stopped_clustered_summary["overtake_success_count"] >= 1,
        "double_stopped_clustered never completed overtake",
    )
    clustered_manifest = load_manifest(double_stopped_clustered_summary)
    require(
        any(row.get("overtake_target_kind") == "cluster" for row in clustered_manifest),
        "double_stopped_clustered never reported cluster target kind",
    )
    require(
        any(len(row.get("overtake_target_member_actor_ids") or []) >= 2 for row in clustered_manifest),
        "double_stopped_clustered never kept multi-actor cluster members",
    )

    signal_suppressed_summary = summaries["signal_suppressed"]
    require(not signal_suppressed_summary["success"], "signal_suppressed unexpectedly succeeded")
    require(
        signal_suppressed_summary["failure_reason"] == "stalled",
        f"signal_suppressed failure_reason unexpected: {signal_suppressed_summary['failure_reason']}",
    )
    require(signal_suppressed_summary["collision_count"] == 0, "signal_suppressed collided")
    require(
        signal_suppressed_summary["overtake_attempt_count"] == 0,
        "signal_suppressed unexpectedly attempted overtake",
    )
    require(
        signal_suppressed_summary["scenario_validation"]["valid"] is True,
        "signal_suppressed scenario validation was not valid",
    )

    near_junction_summary = summaries["near_junction_preflight_reject"]
    require(not near_junction_summary["success"], "near_junction_preflight_reject unexpectedly succeeded")
    require(
        near_junction_summary["failure_reason"] == "scenario_preflight_invalid",
        "near_junction_preflight_reject failure_reason unexpected: "
        f"{near_junction_summary['failure_reason']}",
    )
    require(
        near_junction_summary["scenario_validation"]["valid"] is False,
        "near_junction_preflight_reject scenario validation unexpectedly valid",
    )
    require(
        "junction_nearby" in near_junction_summary["scenario_validation"]["errors"],
        "near_junction_preflight_reject did not report junction_nearby",
    )

    adjacent_lane_closed_summary = summaries["adjacent_lane_closed"]
    require(not adjacent_lane_closed_summary["success"], "adjacent_lane_closed unexpectedly succeeded")
    require(
        adjacent_lane_closed_summary["failure_reason"] == "stalled",
        f"adjacent_lane_closed failure_reason unexpected: {adjacent_lane_closed_summary['failure_reason']}",
    )
    require(adjacent_lane_closed_summary["collision_count"] == 0, "adjacent_lane_closed collided")
    require(
        adjacent_lane_closed_summary["overtake_attempt_count"] == 0,
        "adjacent_lane_closed unexpectedly attempted overtake",
    )
    require(
        adjacent_lane_closed_summary["unsafe_lane_change_reject_count"] >= 1,
        "adjacent_lane_closed never rejected the closed lane",
    )

    curve_clear_summary = summaries["curve_clear"]
    require(bool(curve_clear_summary["success"]), "curve_clear did not succeed")
    require(curve_clear_summary["collision_count"] == 0, "curve_clear collided")
    require(curve_clear_summary["overtake_attempt_count"] >= 1, "curve_clear never attempted overtake")
    require(curve_clear_summary["overtake_success_count"] >= 1, "curve_clear never completed overtake")

    rejoin_blocked_then_release_summary = summaries["rejoin_blocked_then_release"]
    require(
        bool(rejoin_blocked_then_release_summary["success"]),
        "rejoin_blocked_then_release did not succeed",
    )
    require(
        rejoin_blocked_then_release_summary["collision_count"] == 0,
        "rejoin_blocked_then_release collided",
    )
    require(
        rejoin_blocked_then_release_summary["overtake_attempt_count"] >= 1,
        "rejoin_blocked_then_release never attempted overtake",
    )
    require(
        rejoin_blocked_then_release_summary["overtake_success_count"] >= 1,
        "rejoin_blocked_then_release never completed overtake",
    )
    wait_after_pass_s = rejoin_blocked_then_release_summary.get("rejoin_wait_after_target_passed_s")
    require(
        wait_after_pass_s is not None and wait_after_pass_s > 0.0,
        "rejoin_blocked_then_release never waited after passing target",
    )

