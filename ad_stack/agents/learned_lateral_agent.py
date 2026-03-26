from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

from ad_stack.agents.base import ControlDecision, VehicleCommand
from ad_stack.safety.emergency_brake import EmergencyBrakeGuard
from ad_stack.world_model.scene_state import SceneState
from learning.libs.ml import PilotNetInferenceRuntime

SteerPolicy = Callable[[SceneState], float]
LongitudinalPolicy = Callable[[SceneState], tuple[float, float]]


@dataclass(slots=True)
class PilotNetScenePolicy:
    """Adapt SceneState metadata into the PilotNet inference runtime inputs."""

    runtime: PilotNetInferenceRuntime
    rgb_history_key: str = "front_rgb_history"
    command_key: str = "command"
    route_point_key: str = "route_point"

    def __call__(self, scene_state: SceneState) -> float:
        rgb_history = scene_state.metadata.get(self.rgb_history_key)
        if not isinstance(rgb_history, list) or not rgb_history:
            raise ValueError(f"SceneState.metadata[{self.rgb_history_key!r}] must be a non-empty list of RGB arrays.")

        command = scene_state.metadata.get(self.command_key) or scene_state.route.maneuver
        route_point = scene_state.metadata.get(self.route_point_key)
        normalized_route_point: tuple[float, float] | None = None
        if route_point is not None:
            normalized_route_point = (float(route_point[0]), float(route_point[1]))

        return self.runtime.predict_steer(
            rgb_history=rgb_history,
            speed_mps=scene_state.ego.speed_mps,
            command=str(command),
            route_point=normalized_route_point,
        )


@dataclass(slots=True)
class LearnedLateralAgent:
    """Wrap a learned steer policy with pluggable longitudinal control and safety guards."""

    lateral_policy: SteerPolicy
    longitudinal_policy: LongitudinalPolicy | None = None
    safety_guard: EmergencyBrakeGuard = field(default_factory=EmergencyBrakeGuard)
    name: str = field(init=False, default="learned_lateral_agent")

    def reset(self) -> None:
        return None

    def step(self, scene_state: SceneState) -> ControlDecision:
        steer = max(-1.0, min(1.0, float(self.lateral_policy(scene_state))))
        if self.longitudinal_policy is None:
            throttle, brake = self._fallback_longitudinal(scene_state)
        else:
            throttle, brake = self.longitudinal_policy(scene_state)
        decision = ControlDecision(
            command=VehicleCommand(
                steer=steer,
                throttle=float(throttle),
                brake=float(brake),
            ).bounded(),
            behavior=scene_state.route.maneuver,
            planner_state="learned_lateral",
            debug={"route_progress_ratio": scene_state.route.progress_ratio},
        )
        return self.safety_guard.apply(scene_state, decision)

    @staticmethod
    def _fallback_longitudinal(scene_state: SceneState) -> tuple[float, float]:
        target_speed_mps = scene_state.route.target_speed_mps or scene_state.ego.speed_limit_mps or 8.3
        speed_error = target_speed_mps - scene_state.ego.speed_mps
        if speed_error >= 0.0:
            return min(1.0, speed_error * 0.5), 0.0
        return 0.0, min(1.0, (-speed_error) * 0.5)
