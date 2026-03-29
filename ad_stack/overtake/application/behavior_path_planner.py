from __future__ import annotations

import math
from dataclasses import dataclass

from ad_stack.overtake.domain.planning_models import (
    ActiveTargetKind,
    BehaviorPlan,
    PlanningScene,
    RouteBackbone,
    RouteTracePoint,
    Trajectory,
    TrajectoryPoint,
    TrackedTarget,
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
            return plan, _build_constant_speed_trajectory(
                route_backbone,
                start_route_index=scene.route_index,
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
            return plan, _build_signal_stop_trajectory(
                route_backbone,
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
            return plan, _build_constant_speed_trajectory(
                route_backbone,
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
            return plan, _build_constant_speed_trajectory(
                route_backbone,
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
        return plan, _build_constant_speed_trajectory(
            route_backbone,
            start_route_index=scene.route_index,
            desired_speed_mps=min(self.config.car_follow_speed_mps, lead_target.speed_mps),
            trajectory_id="car_follow",
            origin_lane_id=plan.origin_lane_id,
            horizon_m=self.config.trajectory_horizon_m,
            minimum_horizon_m=self.config.minimum_horizon_m,
            resample_interval_m=self.config.resample_interval_m,
        )

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
                return plan, _build_constant_speed_trajectory(
                    route_backbone,
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
                return plan, _build_constant_speed_trajectory(
                    route_backbone,
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
            return plan, _build_constant_speed_trajectory(
                route_backbone,
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
                return plan, _build_constant_speed_trajectory(
                    route_backbone,
                    start_route_index=scene.route_index,
                    desired_speed_mps=self.config.cruise_speed_mps,
                    trajectory_id="lane_follow",
                    horizon_m=self.config.trajectory_horizon_m,
                    minimum_horizon_m=self.config.minimum_horizon_m,
                    resample_interval_m=self.config.resample_interval_m,
                )

        return None


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


def _build_constant_speed_trajectory(
    route_backbone: RouteBackbone,
    *,
    start_route_index: int,
    desired_speed_mps: float,
    trajectory_id: str,
    horizon_m: float,
    minimum_horizon_m: float,
    resample_interval_m: float,
    origin_lane_id: str | None = None,
    target_lane_id: str | None = None,
) -> Trajectory:
    points, end_route_index = _sample_backbone_points(
        route_backbone,
        start_route_index=start_route_index,
        horizon_m=horizon_m,
        minimum_horizon_m=minimum_horizon_m,
        resample_interval_m=resample_interval_m,
    )
    return Trajectory(
        points=tuple(
            TrajectoryPoint(
                x=point.x,
                y=point.y,
                z=point.z,
                yaw_deg=point.yaw_deg,
                longitudinal_velocity_mps=desired_speed_mps,
            )
            for point in points
        ),
        trajectory_id=trajectory_id,
        origin_lane_id=origin_lane_id,
        target_lane_id=target_lane_id,
        source_route_start_index=start_route_index,
        source_route_end_index=end_route_index,
    )


def _build_signal_stop_trajectory(
    route_backbone: RouteBackbone,
    *,
    start_route_index: int,
    desired_speed_mps: float,
    stop_line_distance_m: float,
    trajectory_id: str,
    horizon_m: float,
    minimum_horizon_m: float,
    resample_interval_m: float,
) -> Trajectory:
    points, end_route_index = _sample_backbone_points(
        route_backbone,
        start_route_index=start_route_index,
        horizon_m=horizon_m,
        minimum_horizon_m=minimum_horizon_m,
        resample_interval_m=resample_interval_m,
    )
    speed_points: list[TrajectoryPoint] = []
    for index, point in enumerate(points):
        distance_from_start = index * resample_interval_m
        if distance_from_start >= stop_line_distance_m:
            speed_mps = 0.0
        else:
            remaining_ratio = max(0.0, (stop_line_distance_m - distance_from_start) / max(stop_line_distance_m, 1e-3))
            speed_mps = desired_speed_mps * remaining_ratio
        speed_points.append(
            TrajectoryPoint(
                x=point.x,
                y=point.y,
                z=point.z,
                yaw_deg=point.yaw_deg,
                longitudinal_velocity_mps=speed_mps,
            )
        )
    return Trajectory(
        points=tuple(speed_points),
        trajectory_id=trajectory_id,
        source_route_start_index=start_route_index,
        source_route_end_index=end_route_index,
    )


def _sample_backbone_points(
    route_backbone: RouteBackbone,
    *,
    start_route_index: int,
    horizon_m: float,
    minimum_horizon_m: float,
    resample_interval_m: float,
) -> tuple[list[RouteTracePoint], int]:
    start_trace_index = route_backbone.trace_index_for_route_index(start_route_index)
    start_progress = route_backbone.progress_m[start_trace_index]
    remaining_horizon = route_backbone.progress_m[-1] - start_progress
    target_horizon = min(horizon_m, remaining_horizon)
    if target_horizon < minimum_horizon_m and remaining_horizon >= minimum_horizon_m:
        target_horizon = minimum_horizon_m
    end_progress = min(route_backbone.progress_m[-1], start_progress + target_horizon)
    sampled_points: list[RouteTracePoint] = []
    sample_progress = start_progress
    while sample_progress <= end_progress + 1e-6:
        sampled_points.append(_interpolate_route_trace(route_backbone, sample_progress))
        sample_progress += resample_interval_m
    if len(sampled_points) < 2:
        sampled_points.append(route_backbone.trace[min(start_trace_index + 1, len(route_backbone.trace) - 1)])
    end_trace_index = _find_trace_index_at_progress(route_backbone, end_progress)
    end_route_index = start_route_index
    for route_index, trace_index in enumerate(route_backbone.route_index_to_trace_index):
        if trace_index <= end_trace_index:
            end_route_index = route_index
    return sampled_points, end_route_index


def _interpolate_route_trace(route_backbone: RouteBackbone, progress_m: float) -> RouteTracePoint:
    trace = route_backbone.trace
    progress = route_backbone.progress_m
    if progress_m <= progress[0]:
        return trace[0]
    if progress_m >= progress[-1]:
        return trace[-1]
    for index in range(len(progress) - 1):
        start_progress = progress[index]
        end_progress = progress[index + 1]
        if start_progress <= progress_m <= end_progress:
            span = max(end_progress - start_progress, 1e-6)
            ratio = (progress_m - start_progress) / span
            start = trace[index]
            end = trace[index + 1]
            return RouteTracePoint(
                x=_lerp(start.x, end.x, ratio),
                y=_lerp(start.y, end.y, ratio),
                z=_lerp(start.z, end.z, ratio),
                yaw_deg=_interpolate_yaw(start.yaw_deg, end.yaw_deg, ratio),
                lane_id=start.lane_id,
                road_option=start.road_option,
            )
    return trace[-1]


def _find_trace_index_at_progress(route_backbone: RouteBackbone, progress_m: float) -> int:
    for index, value in enumerate(route_backbone.progress_m):
        if value >= progress_m:
            return index
    return len(route_backbone.progress_m) - 1


def _lerp(a: float, b: float, ratio: float) -> float:
    return a + (b - a) * ratio


def _interpolate_yaw(a_deg: float, b_deg: float, ratio: float) -> float:
    a_rad = math.radians(a_deg)
    b_rad = math.radians(b_deg)
    delta = (b_rad - a_rad + math.pi) % (2.0 * math.pi) - math.pi
    return math.degrees(a_rad + delta * ratio)
