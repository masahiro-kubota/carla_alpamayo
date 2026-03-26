"""Safety guards that can veto nominal planner outputs."""

from ad_stack.safety.emergency_brake import EmergencyBrakeConfig, EmergencyBrakeGuard

__all__ = [
    "EmergencyBrakeConfig",
    "EmergencyBrakeGuard",
]
