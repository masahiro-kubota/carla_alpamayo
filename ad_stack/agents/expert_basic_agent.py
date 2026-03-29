from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Literal

from ad_stack.agents.base import ControlDecision, VehicleCommand
from ad_stack.overtake.application.behavior_path_planner import (
    BehaviorPathPlanner,
    BehaviorPathPlannerConfig,
)
from ad_stack.overtake.application.control_profile import (
    is_traffic_light_violation,
    should_stop_for_light,
    speed_control,
    traffic_light_stop_control,
    traffic_light_stop_target_distance_m,
)
from ad_stack.overtake.application.decision_service import evaluate_pass_progress
from ad_stack.overtake.application.runtime_state import OvertakeRuntimeState
from ad_stack.overtake.application.step_service import OvertakeStepRequest, resolve_overtake_step
from ad_stack.overtake.application.pure_pursuit_controller import PurePursuitController
from ad_stack.overtake.application.traffic_light_service import resolve_active_light
from ad_stack.overtake.policies import TargetAcceptancePolicy, TargetPolicy
from ad_stack.overtake.infrastructure.carla.execution_manager import OvertakeExecutionManager
from ad_stack.overtake.infrastructure.carla.route_backbone_builder import build_route_backbone
from ad_stack.overtake.infrastructure.carla.candidate_extractor import build_target_candidates
from ad_stack.overtake.infrastructure.carla.scene_assembler import (
    build_overtake_pass_snapshot,
    build_overtake_scene_snapshot,
    lane_gaps_for_lane_id,
)
from ad_stack.overtake.infrastructure.carla.telemetry_mapper import build_overtake_planning_debug
from ad_stack.overtake.infrastructure.carla.controller_adapter import run_tracking_control
from ad_stack.overtake.infrastructure.carla.trajectory_materializer import (
    waypoints_to_pose_samples,
)
from ad_stack.overtake.domain.planning_models import RouteBackbone

if TYPE_CHECKING:
    from ad_stack.world_model import DynamicVehicleStateView, SceneState, TrafficLightStateView


@dataclass(slots=True)
class ExpertBasicAgentConfig:
    target_speed_kmh: float = 20.0
    ignore_traffic_lights: bool = False
    ignore_stop_signs: bool = True
    ignore_vehicles: bool = False
    sampling_resolution_m: float = 2.0
    follow_headway_seconds: float = 1.8
    yellow_stop_margin_seconds: float = 1.0
    traffic_light_stop_buffer_m: float = 3.0
    traffic_light_brake_start_distance_m: float = 10.0
    traffic_light_creep_resume_distance_m: float = 1.0
    traffic_light_creep_speed_kmh: float = 4.0
    traffic_light_red_latch_seconds: float = 0.5
    overtake_target_speed_kmh: float = 20.0
    overtake_min_front_gap_m: float = 20.0
    overtake_min_rear_gap_m: float = 15.0
    overtake_signal_suppression_distance_m: float = 35.0
    overtake_trigger_distance_m: float = 18.0
    overtake_resume_front_gap_m: float = 12.0
    preferred_deceleration_mps2: float = 4.0
    reaction_margin_m: float = 2.0
    allow_overtake: bool = True
    preferred_overtake_direction: Literal["left_first", "right_first"] = "left_first"
    lane_change_same_lane_distance_m: float = 6.0
    lane_change_distance_m: float = 14.0
    lane_change_other_lane_distance_m: float = 22.0
    overtake_hold_distance_m: float = 30.0
    overtake_cluster_merge_gap_m: float = 10.0
    overtake_cluster_max_member_speed_mps: float = 0.5
    stopped_speed_threshold_mps: float = 0.3


class ExpertBasicAgent:
    """Rule-based expert policy that drives a unified trajectory + pure-pursuit controller."""

    name = "expert_route_policy"

    def __init__(
        self,
        vehicle: Any,
        world_map: Any,
        *,
        config: ExpertBasicAgentConfig | None = None,
        target_policy: TargetPolicy,
        target_acceptance_policy: TargetAcceptancePolicy,
    ) -> None:
        import carla

        self._carla = carla
        self.config = config or ExpertBasicAgentConfig()
        self._vehicle = vehicle
        self._world = vehicle.get_world()
        self._map = world_map
        self._controller = PurePursuitController()
        self._planner = BehaviorPathPlanner(
            config=BehaviorPathPlannerConfig(
                cruise_speed_mps=self.config.target_speed_kmh / 3.6,
                car_follow_speed_mps=(10.0 / 3.6),
                overtake_speed_mps=self.config.overtake_target_speed_kmh / 3.6,
                signal_stop_distance_m=self.config.overtake_signal_suppression_distance_m,
            )
        )
        self._base_trace: list[tuple[Any, Any]] = []
        self._route_backbone: RouteBackbone | None = None
        self._route_point_to_trace_index: list[int] = []
        self._route_point_progress_m: list[float] = []
        self._max_route_index: int = 0
        self._current_route_index: int = 0
        self._current_route_command: str = "lane_follow"
        self._overtake = OvertakeRuntimeState()
        self._target_policy = target_policy
        self._target_acceptance_policy = target_acceptance_policy
        self._execution = OvertakeExecutionManager(
            sampling_resolution_m=self.config.sampling_resolution_m,
        )
        self._car_follow_active = False
        self._waiting_on_light = False
        self._longitudinal_speed_error_kmh = 0.0
        self._previous_planner_state: str | None = None
        self._latched_red_light: TrafficLightStateView | None = None
        self._latched_red_until_s: float = -1.0

    def set_global_plan(self, trace: list[tuple[Any, Any]]) -> None:
        self._base_trace = list(trace)
        self._route_backbone = build_route_backbone(self._base_trace)
        self._route_point_to_trace_index = []
        self._route_point_progress_m = []
        last_point: tuple[float, float] | None = None
        cumulative_progress_m = 0.0
        for trace_index, (waypoint, _option) in enumerate(self._base_trace):
            location = waypoint.transform.location
            point = (location.x, location.y)
            if (
                last_point is None
                or math.hypot(point[0] - last_point[0], point[1] - last_point[1]) > 0.05
            ):
                if last_point is not None:
                    cumulative_progress_m += math.hypot(
                        point[0] - last_point[0],
                        point[1] - last_point[1],
                    )
                self._route_point_to_trace_index.append(trace_index)
                self._route_point_progress_m.append(cumulative_progress_m)
                last_point = point
        self._current_route_index = 0
        if self._route_backbone is not None:
            self._current_route_command = self._route_backbone.road_option_for_index(0)

    def current_route_command(self) -> str:
        return self._current_route_command

    def done(self) -> bool:
        if self._route_backbone is None:
            return False
        return self._max_route_index >= (len(self._route_backbone.route_index_to_trace_index) - 1)

    def reset(self) -> None:
        self._max_route_index = 0
        self._current_route_index = 0
        self._overtake.reset()
        self._execution.reset()
        self._car_follow_active = False
        self._waiting_on_light = False
        self._longitudinal_speed_error_kmh = 0.0
        self._previous_planner_state = None
        self._latched_red_light = None
        self._latched_red_until_s = -1.0
        self._controller.previous_nearest_index = 0
        self._controller.previous_filtered_steer = 0.0
        self._controller.previous_applied_steer = 0.0
        if self._route_backbone is not None:
            self._current_route_command = self._route_backbone.road_option_for_index(0)

    def step(self, scene_state: SceneState) -> ControlDecision:
        current_speed_mps = scene_state.ego.speed_mps
        route_index = scene_state.route.route_index
        if route_index is not None:
            self._max_route_index = max(self._max_route_index, route_index)
            self._current_route_index = route_index
            if self._route_backbone is not None:
                bounded_route_index = max(
                    0,
                    min(route_index, len(self._route_backbone.route_index_to_road_option) - 1),
                )
                self._current_route_command = self._route_backbone.road_option_for_index(
                    bounded_route_index
                )

        active_light, self._latched_red_light, self._latched_red_until_s = resolve_active_light(
            traffic_lights=scene_state.traffic_lights,
            timestamp_s=scene_state.timestamp_s,
            latched_red_light=self._latched_red_light,
            latched_red_until_s=self._latched_red_until_s,
            red_latch_seconds=self.config.traffic_light_red_latch_seconds,
        )
        planner_state = "nominal_cruise"
        target_speed_kmh = self.config.target_speed_kmh
        target_lane_id = scene_state.route.target_lane_id or scene_state.ego.lane_id
        current_lane_id = scene_state.ego.lane_id
        ego_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        lane_center_offset_m = (
            float(self._vehicle.get_location().distance(ego_waypoint.transform.location))
            if ego_waypoint is not None
            else None
        )
        overtake_scene = build_overtake_scene_snapshot(
            tracked_objects=scene_state.tracked_objects,
            timestamp_s=scene_state.timestamp_s,
            current_speed_mps=current_speed_mps,
            current_lane_id=current_lane_id,
            route_target_lane_id=scene_state.route.target_lane_id,
            route_index=route_index,
            ego_waypoint=ego_waypoint,
            adjacent_lanes_open=scene_state.ego.adjacent_lanes_open,
            target_speed_kmh=self.config.target_speed_kmh,
            follow_headway_seconds=self.config.follow_headway_seconds,
            stopped_speed_threshold_mps=self.config.stopped_speed_threshold_mps,
            cluster_merge_gap_m=self.config.overtake_cluster_merge_gap_m,
            cluster_max_member_speed_mps=self.config.overtake_cluster_max_member_speed_mps,
            candidate_builder=build_target_candidates,
            target_policy=self._target_policy,
            active_signal_state=active_light.state if active_light is not None else None,
            signal_stop_distance_m=(
                active_light.stop_line_distance_m if active_light is not None else None
            ),
            allow_overtake=self.config.allow_overtake,
            preferred_direction=self.config.preferred_overtake_direction,
            world_map=self._map,
            carla_module=self._carla,
            base_trace=self._base_trace,
            route_point_to_trace_index=self._route_point_to_trace_index,
            route_point_progress_m=self._route_point_progress_m,
        )
        follow_actor = overtake_scene.follow_actor
        follow_lead = overtake_scene.follow_lead
        active_overtake_target = overtake_scene.active_target
        follow_distance_m = overtake_scene.follow_distance_m
        follow_speed_mps = overtake_scene.follow_speed_mps
        same_lane_follow_distance_m = overtake_scene.same_lane_follow_distance_m
        same_lane_follow_speed_mps = overtake_scene.same_lane_follow_speed_mps
        closing_speed_mps = overtake_scene.closing_speed_mps
        min_ttc = overtake_scene.min_ttc
        same_lane_min_ttc = overtake_scene.same_lane_min_ttc
        follow_target_speed_kmh = overtake_scene.follow_target_speed_kmh
        left_front_gap_m = overtake_scene.left_front_gap_m
        left_rear_gap_m = overtake_scene.left_rear_gap_m
        right_front_gap_m = overtake_scene.right_front_gap_m
        right_rear_gap_m = overtake_scene.right_rear_gap_m
        rejoin_front_gap_m = float("inf")
        rejoin_rear_gap_m = float("inf")
        overtake_reject_reason: str | None = None
        overtake_considered = False
        event_flags = {
            "event_traffic_light_stop": False,
            "event_traffic_light_resume": False,
            "event_car_follow_start": False,
            "event_overtake_attempt": False,
            "event_overtake_success": False,
            "event_overtake_abort": False,
            "event_unsafe_lane_change_reject": False,
        }
        stop_for_light = should_stop_for_light(
            ignore_traffic_lights=self.config.ignore_traffic_lights,
            light_state=active_light.state if active_light is not None else None,
            raw_stop_distance_m=(
                active_light.stop_line_distance_m
                if active_light is not None and active_light.stop_line_distance_m is not None
                else active_light.distance_m if active_light is not None else None
            ),
            current_speed_mps=current_speed_mps,
            stop_buffer_m=self.config.traffic_light_stop_buffer_m,
            reaction_margin_m=self.config.reaction_margin_m,
            preferred_deceleration_mps2=self.config.preferred_deceleration_mps2,
            yellow_stop_margin_seconds=self.config.yellow_stop_margin_seconds,
        )

        stop_target_distance_m = traffic_light_stop_target_distance_m(
            distance_m=(
                active_light.stop_line_distance_m
                if active_light is not None and active_light.stop_line_distance_m is not None
                else active_light.distance_m if active_light is not None else None
            ),
            stop_buffer_m=self.config.traffic_light_stop_buffer_m,
        )
        step_decision = resolve_overtake_step(
            OvertakeStepRequest(
                runtime_state=self._overtake,
                decision_context=overtake_scene.decision_context,
                target_acceptance_policy=self._target_acceptance_policy,
                stop_for_light=stop_for_light,
                ignore_traffic_lights=self.config.ignore_traffic_lights,
                ignore_vehicles=self.config.ignore_vehicles,
                lead_vehicle_present=follow_actor is not None,
                active_target_present=active_overtake_target is not None,
                lead_distance_m=follow_distance_m,
                lead_speed_mps=follow_speed_mps,
                follow_target_speed_kmh=follow_target_speed_kmh,
                current_speed_mps=current_speed_mps,
                current_lane_id=current_lane_id,
                route_target_lane_id=scene_state.route.target_lane_id,
                lane_center_offset_m=lane_center_offset_m,
                target_speed_kmh=self.config.target_speed_kmh,
                active_light_state=active_light.state if active_light is not None else None,
                signal_stop_distance_m=stop_target_distance_m,
                traffic_light_brake_start_distance_m=self.config.traffic_light_brake_start_distance_m,
                traffic_light_creep_resume_distance_m=self.config.traffic_light_creep_resume_distance_m,
                traffic_light_creep_speed_kmh=self.config.traffic_light_creep_speed_kmh,
                overtake_target_speed_kmh=self.config.overtake_target_speed_kmh,
                overtake_trigger_distance_m=self.config.overtake_trigger_distance_m,
                overtake_min_front_gap_m=self.config.overtake_min_front_gap_m,
                overtake_min_rear_gap_m=self.config.overtake_min_rear_gap_m,
                overtake_signal_suppression_distance_m=self.config.overtake_signal_suppression_distance_m,
            )
        )
        planner_state = step_decision.planner_state
        target_speed_kmh = step_decision.target_speed_kmh
        overtake_considered = step_decision.overtake_considered
        overtake_reject_reason = step_decision.overtake_reject_reason
        event_flags["event_traffic_light_stop"] = step_decision.event_flags.traffic_light_stop
        event_flags["event_overtake_abort"] = step_decision.event_flags.overtake_abort
        event_flags["event_overtake_success"] = step_decision.event_flags.overtake_success
        event_flags["event_unsafe_lane_change_reject"] = (
            step_decision.event_flags.unsafe_lane_change_reject
        )
        if step_decision.next_runtime_state is not None:
            self._overtake.state = step_decision.next_runtime_state
        if step_decision.next_aborted is not None:
            self._overtake.aborted = step_decision.next_aborted
        if step_decision.request_prepare_abort_return:
            self._execution.prepare_abort_return(
                carla_module=self._carla,
                direction=self._overtake.direction,
                route_index=route_index,
                base_trace=self._base_trace,
                route_point_to_trace_index=self._route_point_to_trace_index,
                origin_lane_id=self._overtake.origin_lane_id,
                target_lane_id=self._execution.target_lane_id,
                lane_change_distance_m=self.config.lane_change_distance_m,
            )
        if step_decision.completed:
            self._overtake.reset()
            self._execution.clear()
            self._execution.resume_base_route(
                route_index=route_index,
                base_trace=self._base_trace,
                route_point_to_trace_index=self._route_point_to_trace_index,
            )
            planner_state = "nominal_cruise"
        elif step_decision.request_overtake_direction is not None:
            activation = self._execution.activate_overtake_plan(
                carla_module=self._carla,
                direction=step_decision.request_overtake_direction,
                route_index=route_index,
                base_trace=self._base_trace,
                route_point_to_trace_index=self._route_point_to_trace_index,
                distance_same_lane_m=self.config.lane_change_same_lane_distance_m,
                lane_change_distance_m=self.config.lane_change_distance_m,
                overtake_hold_distance_m=self.config.overtake_hold_distance_m,
            )
            if activation.activated:
                self._overtake.begin_lane_change_out(
                    direction=step_decision.request_overtake_direction,
                    origin_lane_id=current_lane_id,
                    active_target=active_overtake_target,
                    lead_actor_id=follow_lead.actor_id if follow_lead is not None else None,
                    lead_lane_id=follow_lead.lane_id if follow_lead is not None else None,
                    lead_distance_m=follow_distance_m,
                    target_lane_id=activation.target_lane_id,
                )
                planner_state = "lane_change_out"
                event_flags["event_overtake_attempt"] = True
                overtake_reject_reason = None
            else:
                self._overtake.reset()
                planner_state = "car_follow"
                target_speed_kmh = follow_target_speed_kmh
                overtake_reject_reason = (
                    self._execution.lane_change_path.failure_reason or "lane_change_path_failed"
                )
                event_flags["event_unsafe_lane_change_reject"] = True

        target_actor = None
        target_actor_visible = False
        if self._overtake.target_actor_id is not None:
            pass_snapshot = build_overtake_pass_snapshot(
                tracked_objects=scene_state.tracked_objects,
                target_actor_id=self._overtake.target_actor_id,
                target_member_actor_ids=self._overtake.target_member_actor_ids,
                route_index=route_index,
                base_trace=self._base_trace,
                route_point_to_trace_index=self._route_point_to_trace_index,
                route_point_progress_m=self._route_point_progress_m,
            )
            target_actor = pass_snapshot.target_actor
            target_actor_visible = pass_snapshot.target_actor_visible
            self._overtake.memory = evaluate_pass_progress(
                self._overtake.memory,
                timestamp_s=scene_state.timestamp_s,
                target_actor_visible=target_actor_visible,
                target_longitudinal_distance_m=pass_snapshot.target_longitudinal_distance_m,
                overtake_resume_front_gap_m=self.config.overtake_resume_front_gap_m,
                target_kind=self._overtake.memory.target_kind,
                target_exit_longitudinal_distance_m=pass_snapshot.target_exit_longitudinal_distance_m,
            )
            if self._overtake.origin_lane_id is not None:
                rejoin_front_gap_m, rejoin_rear_gap_m = lane_gaps_for_lane_id(
                    scene_state.tracked_objects,
                    self._overtake.origin_lane_id,
                )
            rejoin_decision = resolve_overtake_step(
                OvertakeStepRequest(
                    runtime_state=self._overtake,
                    decision_context=overtake_scene.decision_context,
                    target_acceptance_policy=self._target_acceptance_policy,
                    stop_for_light=stop_for_light,
                    ignore_traffic_lights=self.config.ignore_traffic_lights,
                    ignore_vehicles=self.config.ignore_vehicles,
                    lead_vehicle_present=follow_actor is not None,
                    active_target_present=active_overtake_target is not None,
                    lead_distance_m=follow_distance_m,
                    lead_speed_mps=follow_speed_mps,
                    follow_target_speed_kmh=follow_target_speed_kmh,
                    current_speed_mps=current_speed_mps,
                    current_lane_id=current_lane_id,
                    route_target_lane_id=scene_state.route.target_lane_id,
                    lane_center_offset_m=lane_center_offset_m,
                    target_speed_kmh=self.config.target_speed_kmh,
                    active_light_state=active_light.state if active_light is not None else None,
                    signal_stop_distance_m=stop_target_distance_m,
                    traffic_light_brake_start_distance_m=self.config.traffic_light_brake_start_distance_m,
                    traffic_light_creep_resume_distance_m=self.config.traffic_light_creep_resume_distance_m,
                    traffic_light_creep_speed_kmh=self.config.traffic_light_creep_speed_kmh,
                    overtake_target_speed_kmh=self.config.overtake_target_speed_kmh,
                    overtake_trigger_distance_m=self.config.overtake_trigger_distance_m,
                    overtake_min_front_gap_m=self.config.overtake_min_front_gap_m,
                    overtake_min_rear_gap_m=self.config.overtake_min_rear_gap_m,
                    overtake_signal_suppression_distance_m=self.config.overtake_signal_suppression_distance_m,
                    rejoin_front_gap_m=(
                        None if not math.isfinite(rejoin_front_gap_m) else rejoin_front_gap_m
                    ),
                    rejoin_rear_gap_m=(
                        None if not math.isfinite(rejoin_rear_gap_m) else rejoin_rear_gap_m
                    ),
                )
            )
            if rejoin_decision.request_rejoin:
                rejoin_activation = self._execution.try_activate_rejoin_plan(
                    carla_module=self._carla,
                    direction=self._overtake.direction,
                    route_index=route_index,
                    base_trace=self._base_trace,
                    route_point_to_trace_index=self._route_point_to_trace_index,
                    origin_lane_id=self._overtake.origin_lane_id,
                    target_lane_id=self._execution.target_lane_id,
                    lane_change_distance_m=self.config.lane_change_distance_m,
                )
                if rejoin_activation.activated:
                    self._overtake.state = "lane_change_back"
                    planner_state = "lane_change_back"
            self._overtake.memory.state = self._overtake.state

        if planner_state == "car_follow" and not self._car_follow_active:
            event_flags["event_car_follow_start"] = True
        self._car_follow_active = planner_state == "car_follow"

        if planner_state == "traffic_light_stop" and not self._waiting_on_light:
            event_flags["event_traffic_light_stop"] = True
        if planner_state != "traffic_light_stop" and self._waiting_on_light:
            event_flags["event_traffic_light_resume"] = True
        self._waiting_on_light = planner_state == "traffic_light_stop"

        if planner_state != self._previous_planner_state:
            self._longitudinal_speed_error_kmh = 0.0
        self._previous_planner_state = planner_state

        active_waypoints: list[Any] = []
        if self._overtake.state != "idle":
            self._execution.advance(vehicle_location=self._vehicle.get_location())
            active_waypoints = self._execution.remaining_waypoints()
        local_path_samples = (
            waypoints_to_pose_samples(active_waypoints)
            if active_waypoints
            else ()
        )
        behavior_plan, trajectory = self._planner.plan_runtime(
            route_backbone=self._require_route_backbone(),
            route_index=max(0, route_index or 0),
            route_command=self._current_route_command,
            planner_state=planner_state,
            desired_speed_mps=max(0.0, target_speed_kmh) / 3.6,
            active_target_id=(
                active_overtake_target.primary_actor_id
                if planner_state == "car_follow" and active_overtake_target is not None
                else self._overtake.target_actor_id
            ),
            active_target_kind=(
                active_overtake_target.kind
                if planner_state == "car_follow" and active_overtake_target is not None
                else (
                    self._overtake.memory.target_kind
                    if self._overtake.target_actor_id is not None
                    else None
                )
            ),
            origin_lane_id=self._overtake.origin_lane_id or self._overtake.memory.origin_lane_id,
            target_lane_id=(
                self._execution.target_lane_id
                if planner_state != "car_follow"
                else None
            ) or self._overtake.origin_lane_id,
            reject_reason=overtake_reject_reason,
            signal_stop_distance_m=stop_target_distance_m,
            local_path_samples=local_path_samples,
        )
        tracking = run_tracking_control(
            carla_module=self._carla,
            controller=self._controller,
            ego_transform=self._vehicle.get_transform(),
            ego_speed_mps=current_speed_mps,
            trajectory=trajectory,
        )
        control = tracking.control
        if planner_state == "traffic_light_stop":
            traffic_light_throttle, traffic_light_brake, self._longitudinal_speed_error_kmh = traffic_light_stop_control(
                current_speed_mps=current_speed_mps,
                light_state=active_light.state if active_light is not None else None,
                stop_target_distance_m=stop_target_distance_m,
                target_speed_kmh=target_speed_kmh,
                previous_error_kmh=self._longitudinal_speed_error_kmh,
                preferred_deceleration_mps2=self.config.preferred_deceleration_mps2,
            )
            control.throttle = traffic_light_throttle
            control.brake = traffic_light_brake
        else:
            control.throttle, control.brake, self._longitudinal_speed_error_kmh = speed_control(
                current_speed_mps=current_speed_mps,
                target_speed_kmh=target_speed_kmh,
                previous_error_kmh=self._longitudinal_speed_error_kmh,
                max_throttle=0.75,
                max_brake=0.8,
            )

        event_flags["traffic_light_violation"] = is_traffic_light_violation(
            light_state=active_light.state if active_light is not None else None,
            stop_line_distance_m=(
                active_light.stop_line_distance_m if active_light is not None else None
            ),
            current_speed_mps=current_speed_mps,
        )

        behavior = behavior_plan.state
        planning_debug = build_overtake_planning_debug(
            behavior_state=behavior_plan.state,
            route_command=behavior_plan.route_command,
            remaining_waypoints=max(
                len(self._require_route_backbone().route_index_to_trace_index) - 1 - self._max_route_index,
                0,
            ),
            route_index=route_index,
            max_route_index=self._max_route_index,
            current_lane_id=current_lane_id,
            lane_center_offset_m=lane_center_offset_m,
            route_target_lane_id=scene_state.route.target_lane_id,
            left_lane_open=bool(scene_state.ego.adjacent_lanes_open.get("left", False)),
            right_lane_open=bool(scene_state.ego.adjacent_lanes_open.get("right", False)),
            active_light=active_light,
            red_light_latched=bool(
                active_light is not None
                and self._latched_red_light is not None
                and active_light.actor_id == self._latched_red_light.actor_id
                and active_light.state == "red"
                and scene_state.timestamp_s <= self._latched_red_until_s
            ),
            traffic_light_stop_buffer_m=self.config.traffic_light_stop_buffer_m,
            traffic_light_stop_target_distance_m=stop_target_distance_m,
            target_speed_kmh=target_speed_kmh,
            desired_speed_mps=tracking.desired_speed_mps,
            applied_speed_mps=current_speed_mps,
            lookahead_distance_m=tracking.lookahead_distance_m,
            lateral_error_m=tracking.lateral_error_m,
            heading_error_deg=tracking.heading_error_deg,
            controller_steer_raw=tracking.controller_steer_raw,
            controller_steer_applied=tracking.controller_steer_applied,
            follow_lead=follow_lead,
            active_target=active_overtake_target,
            follow_distance_m=follow_distance_m,
            follow_speed_mps=follow_speed_mps,
            closing_speed_mps=closing_speed_mps,
            left_lane_front_gap_m=left_front_gap_m,
            left_lane_rear_gap_m=left_rear_gap_m,
            right_lane_front_gap_m=right_front_gap_m,
            right_lane_rear_gap_m=right_rear_gap_m,
            rejoin_front_gap_m=rejoin_front_gap_m,
            rejoin_rear_gap_m=rejoin_rear_gap_m,
            overtake_considered=overtake_considered,
            overtake_reject_reason=overtake_reject_reason,
            overtake_state=self._overtake.state,
            overtake_direction=self._overtake.direction,
            overtake_origin_lane_id=self._overtake.origin_lane_id,
            overtake_target_actor_id=self._overtake.target_actor_id,
            overtake_target_kind=self._overtake.memory.target_kind,
            overtake_target_member_actor_ids=self._overtake.target_member_actor_ids,
            overtake_target_lane_id=self._execution.target_lane_id,
            target_passed=self._overtake.memory.target_passed,
            distance_past_target_m=self._overtake.memory.target_pass_distance_m,
            target_actor_visible=target_actor_visible,
            target_actor_last_seen_s=self._overtake.memory.target_actor_last_seen_s,
            target_lane_id=target_lane_id,
            min_ttc=min_ttc,
            event_flags=event_flags,
        )
        return ControlDecision(
            command=VehicleCommand(
                steer=float(control.steer),
                throttle=float(control.throttle),
                brake=float(control.brake),
                hand_brake=bool(control.hand_brake),
                reverse=bool(control.reverse),
            ).bounded(),
            behavior=behavior,
            planner_state=planner_state,
            debug=planning_debug,
        )

    def _require_route_backbone(self) -> RouteBackbone:
        if self._route_backbone is None:
            raise RuntimeError("global plan must be set before running the expert agent")
        return self._route_backbone
