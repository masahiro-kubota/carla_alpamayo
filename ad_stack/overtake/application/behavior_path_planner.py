from __future__ import annotations

from dataclasses import dataclass

from ad_stack.overtake.domain.planning_models import (
    BehaviorState,
    ActiveTargetKind,
    BehaviorPlan,
    Pose3D,
    PlanningScene,
    RouteBackbone,
    Trajectory,
    TrackedTarget,
)
from .trajectory_generation import (
    build_pose_trajectory,
    build_route_backbone_trajectory,
    build_signal_stop_trajectory,
)


@dataclass(frozen=True, slots=True)
class BehaviorPathPlannerConfig:
    cruise_speed_mps: float = 20.0 / 3.6
    car_follow_speed_mps: float = 10.0 / 3.6
    overtake_speed_mps: float = 20.0 / 3.6
    signal_stop_distance_m: float = 25.0
    overtake_trigger_distance_m: float = 25.0
    trajectory_horizon_m: float = 60.0
    minimum_horizon_m: float = 30.0
    resample_interval_m: float = 1.0


@dataclass(slots=True)
class BehaviorPathPlanner:
    config: BehaviorPathPlannerConfig = BehaviorPathPlannerConfig()

    def plan(
        self,
        *,
        route_backbone: RouteBackbone,
        scene: PlanningScene,
        previous_behavior_plan: BehaviorPlan | None = None,
        previous_trajectory: Trajectory | None = None,
    ) -> tuple[BehaviorPlan, Trajectory]:
        del previous_trajectory
        route_command = route_backbone.road_option_for_index(scene.route_index)
        signal_light = _select_stop_light(
            scene,
            signal_stop_distance_m=self.config.signal_stop_distance_m,
        )
        lead_target = _select_lead_target(scene)

        if (
            signal_light is not None
            and previous_behavior_plan is not None
            and previous_behavior_plan.state in {"lane_change_out", "pass_vehicle"}
        ):
            plan = BehaviorPlan(
                state="abort_return",
                route_command=route_command,
                active_target_id=previous_behavior_plan.active_target_id,
                active_target_kind=previous_behavior_plan.active_target_kind,
                origin_lane_id=previous_behavior_plan.origin_lane_id,
                target_lane_id=previous_behavior_plan.origin_lane_id,
                reject_reason="signal_stop",
            )
            return plan, build_route_backbone_trajectory(
                start_route_index=scene.route_index,
                route_backbone=route_backbone,
                desired_speed_mps=self.config.car_follow_speed_mps,
                trajectory_id="abort_return",
                origin_lane_id=plan.origin_lane_id,
                target_lane_id=plan.target_lane_id,
                horizon_m=self.config.trajectory_horizon_m,
                minimum_horizon_m=self.config.minimum_horizon_m,
                resample_interval_m=self.config.resample_interval_m,
            )

        if signal_light is not None:
            stop_distance_m = signal_light.stop_line_distance_m
            assert stop_distance_m is not None
            plan = BehaviorPlan(
                state="signal_stop",
                route_command=route_command,
            )
            return plan, build_signal_stop_trajectory(
                route_backbone=route_backbone,
                start_route_index=scene.route_index,
                desired_speed_mps=self.config.cruise_speed_mps,
                stop_line_distance_m=stop_distance_m,
                trajectory_id="signal_stop",
                horizon_m=self.config.trajectory_horizon_m,
                minimum_horizon_m=self.config.minimum_horizon_m,
                resample_interval_m=self.config.resample_interval_m,
            )

        if previous_behavior_plan is not None:
            continued = self._continue_previous_plan(
                previous_behavior_plan=previous_behavior_plan,
                route_backbone=route_backbone,
                scene=scene,
                lead_target=lead_target,
                route_command=route_command,
            )
            if continued is not None:
                return continued

        if lead_target is None:
            plan = BehaviorPlan(
                state="lane_follow",
                route_command=route_command,
            )
            return plan, build_route_backbone_trajectory(
                route_backbone=route_backbone,
                start_route_index=scene.route_index,
                desired_speed_mps=self.config.cruise_speed_mps,
                trajectory_id="lane_follow",
                horizon_m=self.config.trajectory_horizon_m,
                minimum_horizon_m=self.config.minimum_horizon_m,
                resample_interval_m=self.config.resample_interval_m,
            )

        if (
            lead_target.longitudinal_distance_m <= self.config.overtake_trigger_distance_m
            and (scene.adjacent_lane_availability.left_open or scene.adjacent_lane_availability.right_open)
        ):
            target_lane_id = (
                f"{scene.current_lane_id}:left"
                if scene.adjacent_lane_availability.left_open
                else f"{scene.current_lane_id}:right"
            )
            plan = BehaviorPlan(
                state="lane_change_out",
                route_command=route_command,
                active_target_id=lead_target.actor_id,
                active_target_kind=lead_target.target_kind,
                origin_lane_id=scene.current_lane_id,
                target_lane_id=target_lane_id,
            )
            return plan, build_route_backbone_trajectory(
                route_backbone=route_backbone,
                start_route_index=scene.route_index,
                desired_speed_mps=self.config.overtake_speed_mps,
                trajectory_id="lane_change_out",
                origin_lane_id=plan.origin_lane_id,
                target_lane_id=plan.target_lane_id,
                horizon_m=self.config.trajectory_horizon_m,
                minimum_horizon_m=self.config.minimum_horizon_m,
                resample_interval_m=self.config.resample_interval_m,
            )

        plan = BehaviorPlan(
            state="car_follow",
            route_command=route_command,
            active_target_id=lead_target.actor_id,
            active_target_kind=lead_target.target_kind,
            origin_lane_id=scene.current_lane_id,
            reject_reason="adjacent_lane_closed",
        )
        return plan, build_route_backbone_trajectory(
            route_backbone=route_backbone,
            start_route_index=scene.route_index,
            desired_speed_mps=min(self.config.car_follow_speed_mps, lead_target.speed_mps),
            trajectory_id="car_follow",
            origin_lane_id=plan.origin_lane_id,
            horizon_m=self.config.trajectory_horizon_m,
            minimum_horizon_m=self.config.minimum_horizon_m,
            resample_interval_m=self.config.resample_interval_m,
        )

    def plan_runtime(
        self,
        *,
        route_backbone: RouteBackbone,
        route_index: int,
        route_command: str,
        planner_state: str,
        desired_speed_mps: float,
        active_target_id: int | None,
        active_target_kind: ActiveTargetKind | None,
        origin_lane_id: str | None,
        target_lane_id: str | None,
        reject_reason: str | None,
        signal_stop_distance_m: float | None = None,
        local_path_samples: tuple[Pose3D, ...] = (),
    ) -> tuple[BehaviorPlan, Trajectory]:
        behavior_state = _behavior_state_from_planner_state(planner_state)
        plan = BehaviorPlan(
            state=behavior_state,
            route_command=route_command,  # type: ignore[arg-type]
            active_target_id=active_target_id,
            active_target_kind=active_target_kind,
            origin_lane_id=origin_lane_id,
            target_lane_id=target_lane_id,
            reject_reason=reject_reason,
        )
        trajectory_id = behavior_state
        if behavior_state == "signal_stop" and signal_stop_distance_m is not None:
            trajectory = build_signal_stop_trajectory(
                route_backbone=route_backbone,
                start_route_index=route_index,
                desired_speed_mps=desired_speed_mps,
                stop_line_distance_m=signal_stop_distance_m,
                trajectory_id=trajectory_id,
                horizon_m=self.config.trajectory_horizon_m,
                minimum_horizon_m=self.config.minimum_horizon_m,
                resample_interval_m=self.config.resample_interval_m,
                origin_lane_id=origin_lane_id,
                target_lane_id=target_lane_id,
            )
            return plan, trajectory
        if local_path_samples:
            trajectory = build_pose_trajectory(
                pose_samples=local_path_samples,
                desired_speed_mps=desired_speed_mps,
                trajectory_id=trajectory_id,
                origin_lane_id=origin_lane_id,
                target_lane_id=target_lane_id,
                resample_interval_m=self.config.resample_interval_m,
            )
            return plan, trajectory
        trajectory = build_route_backbone_trajectory(
            route_backbone=route_backbone,
            start_route_index=route_index,
            desired_speed_mps=desired_speed_mps,
            trajectory_id=trajectory_id,
            horizon_m=self.config.trajectory_horizon_m,
            minimum_horizon_m=self.config.minimum_horizon_m,
            resample_interval_m=self.config.resample_interval_m,
            origin_lane_id=origin_lane_id,
            target_lane_id=target_lane_id,
        )
        return plan, trajectory

    def _continue_previous_plan(
        self,
        *,
        previous_behavior_plan: BehaviorPlan,
        route_backbone: RouteBackbone,
        scene: PlanningScene,
        lead_target: TrackedTarget | None,
        route_command: str,
    ) -> tuple[BehaviorPlan, Trajectory] | None:
        if previous_behavior_plan.state == "lane_change_out":
            if scene.current_lane_id == previous_behavior_plan.target_lane_id:
                plan = BehaviorPlan(
                    state="pass_vehicle",
                    route_command=route_command,  # type: ignore[arg-type]
                    active_target_id=previous_behavior_plan.active_target_id,
                    active_target_kind=previous_behavior_plan.active_target_kind,
                    origin_lane_id=previous_behavior_plan.origin_lane_id,
                    target_lane_id=previous_behavior_plan.target_lane_id,
                )
                return plan, build_route_backbone_trajectory(
                    route_backbone=route_backbone,
                    start_route_index=scene.route_index,
                    desired_speed_mps=self.config.overtake_speed_mps,
                    trajectory_id="pass_vehicle",
                    origin_lane_id=plan.origin_lane_id,
                    target_lane_id=plan.target_lane_id,
                    horizon_m=self.config.trajectory_horizon_m,
                    minimum_horizon_m=self.config.minimum_horizon_m,
                    resample_interval_m=self.config.resample_interval_m,
                )

        if previous_behavior_plan.state == "pass_vehicle":
            if lead_target is None or lead_target.longitudinal_distance_m <= 0.0:
                plan = BehaviorPlan(
                    state="lane_change_back",
                    route_command=route_command,  # type: ignore[arg-type]
                    active_target_id=previous_behavior_plan.active_target_id,
                    active_target_kind=previous_behavior_plan.active_target_kind,
                    origin_lane_id=previous_behavior_plan.origin_lane_id,
                    target_lane_id=previous_behavior_plan.origin_lane_id,
                )
                return plan, build_route_backbone_trajectory(
                    route_backbone=route_backbone,
                    start_route_index=scene.route_index,
                    desired_speed_mps=self.config.overtake_speed_mps,
                    trajectory_id="lane_change_back",
                    origin_lane_id=plan.origin_lane_id,
                    target_lane_id=plan.target_lane_id,
                    horizon_m=self.config.trajectory_horizon_m,
                    minimum_horizon_m=self.config.minimum_horizon_m,
                    resample_interval_m=self.config.resample_interval_m,
                )
            plan = BehaviorPlan(
                state="pass_vehicle",
                route_command=route_command,  # type: ignore[arg-type]
                active_target_id=previous_behavior_plan.active_target_id,
                active_target_kind=previous_behavior_plan.active_target_kind,
                origin_lane_id=previous_behavior_plan.origin_lane_id,
                target_lane_id=previous_behavior_plan.target_lane_id,
            )
            return plan, build_route_backbone_trajectory(
                route_backbone=route_backbone,
                start_route_index=scene.route_index,
                desired_speed_mps=self.config.overtake_speed_mps,
                trajectory_id="pass_vehicle",
                origin_lane_id=plan.origin_lane_id,
                target_lane_id=plan.target_lane_id,
                horizon_m=self.config.trajectory_horizon_m,
                minimum_horizon_m=self.config.minimum_horizon_m,
                resample_interval_m=self.config.resample_interval_m,
            )

        if previous_behavior_plan.state in {"lane_change_back", "abort_return"}:
            if scene.current_lane_id == previous_behavior_plan.origin_lane_id:
                plan = BehaviorPlan(
                    state="lane_follow",
                    route_command=route_command,  # type: ignore[arg-type]
                )
                return plan, build_route_backbone_trajectory(
                    route_backbone=route_backbone,
                    start_route_index=scene.route_index,
                    desired_speed_mps=self.config.cruise_speed_mps,
                    trajectory_id="lane_follow",
                    horizon_m=self.config.trajectory_horizon_m,
                    minimum_horizon_m=self.config.minimum_horizon_m,
                    resample_interval_m=self.config.resample_interval_m,
                )

        return None


def _behavior_state_from_planner_state(planner_state: str) -> BehaviorState:
    if planner_state == "nominal_cruise":
        return "lane_follow"
    if planner_state == "traffic_light_stop":
        return "signal_stop"
    return planner_state  # type: ignore[return-value]


def _select_stop_light(
    scene: PlanningScene,
    *,
    signal_stop_distance_m: float,
):
    candidates = [
        light
        for light in scene.traffic_lights
        if light.affects_ego
        and light.state in {"red", "yellow"}
        and light.stop_line_distance_m is not None
        and light.stop_line_distance_m <= signal_stop_distance_m
    ]
    if not candidates:
        return None
    return min(candidates, key=lambda light: float(light.stop_line_distance_m))


def _select_lead_target(scene: PlanningScene) -> TrackedTarget | None:
    candidates = [
        target
        for target in scene.tracked_targets
        if target.affects_route
        and target.lane_id == scene.current_lane_id
        and target.longitudinal_distance_m > 0.0
    ]
    if not candidates:
        return None
    return min(candidates, key=lambda target: target.longitudinal_distance_m)

