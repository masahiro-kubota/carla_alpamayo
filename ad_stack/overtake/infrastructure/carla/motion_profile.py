from __future__ import annotations

from ad_stack.overtake.domain import MotionProfile


_DEFAULT_SLOW_TARGET_SPEED_MPS = 5.0 / 3.6


def classify_motion_profile(
    *,
    speed_mps: float,
    stopped_speed_threshold_mps: float,
    slow_target_speed_threshold_mps: float = _DEFAULT_SLOW_TARGET_SPEED_MPS,
) -> MotionProfile:
    absolute_speed_mps = abs(float(speed_mps))
    if absolute_speed_mps <= stopped_speed_threshold_mps:
        return "stopped"
    if absolute_speed_mps <= max(slow_target_speed_threshold_mps, stopped_speed_threshold_mps):
        return "slow"
    return "moving"
