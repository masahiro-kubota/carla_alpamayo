from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Literal

from ad_stack.agents.base import ControlDecision, VehicleCommand
from ad_stack.overtake import (
    OvertakeRuntimeState,
    build_stopped_obstacle_targets,
    choose_overtake_action,
    evaluate_pass_progress,
    is_traffic_light_violation,
    resolve_active_light,
    resolve_overtake_runtime_transition,
    should_stop_for_light,
    should_begin_rejoin,
    speed_control,
    traffic_light_stop_control,
    traffic_light_stop_target_distance_m,
    traffic_light_stop_target_speed_kmh,
)
from ad_stack.overtake.infrastructure.carla import (
    OvertakeExecutionManager,
    build_overtake_pass_snapshot,
    build_overtake_scene_snapshot,
    build_overtake_planning_debug,
    lane_gaps_for_lane_id,
    run_tracking_control,
)
from libs.carla_utils import ensure_carla_agents_on_path, road_option_name

if TYPE_CHECKING:
    from ad_stack.world_model import DynamicVehicleStateView, SceneState, TrafficLightStateView


@dataclass(slots=True)
class ExpertBasicAgentConfig:
    target_speed_kmh: float = 30.0
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
    overtake_speed_delta_kmh: float = 8.0
    overtake_min_front_gap_m: float = 20.0
    overtake_min_rear_gap_m: float = 15.0
    overtake_signal_suppression_distance_m: float = 35.0
    overtake_trigger_distance_m: float = 18.0
    overtake_resume_front_gap_m: float = 12.0
    preferred_deceleration_mps2: float = 4.0
    reaction_margin_m: float = 2.0
    ttc_emergency_threshold_s: float = 0.75
    lead_brake_distance_m: float = 4.5
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
    """Rule-based expert policy that reuses CARLA's local planner for low-level control."""

    name = "expert_route_policy"

    def __init__(
        self,
        vehicle: Any,
        world_map: Any,
        *,
        config: ExpertBasicAgentConfig | None = None,
    ) -> None:
        import carla

        ensure_carla_agents_on_path()
        from agents.navigation.basic_agent import BasicAgent
        from agents.navigation.controller import VehiclePIDController

        self._carla = carla
        self.config = config or ExpertBasicAgentConfig()
        self._vehicle = vehicle
        self._world = vehicle.get_world()
        self._map = world_map
        self._agent = BasicAgent(
            vehicle,
            target_speed=self.config.target_speed_kmh,
            opt_dict={
                # Hazard handling is implemented in this policy. The local planner is only reused
                # for waypoint tracking and longitudinal/lateral PID generation.
                "ignore_traffic_lights": True,
                "ignore_stop_signs": True,
                "ignore_vehicles": True,
                "sampling_resolution": self.config.sampling_resolution_m,
            },
            map_inst=world_map,
        )
        self._overtake_controller = VehiclePIDController(
            vehicle,
            args_lateral={"K_P": 1.95, "K_I": 0.05, "K_D": 0.2, "dt": 1.0 / 20.0},
            args_longitudinal={"K_P": 1.0, "K_I": 0.05, "K_D": 0.0, "dt": 1.0 / 20.0},
            max_throttle=0.75,
            max_brake=0.5,
            max_steering=0.8,
        )
        self._base_trace: list[tuple[Any, Any]] = []
        self._route_point_to_trace_index: list[int] = []
        self._route_point_progress_m: list[float] = []
        self._max_route_index: int = 0
        self._overtake = OvertakeRuntimeState()
        self._execution = OvertakeExecutionManager(
            local_agent=self._agent,
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
        self._agent.set_global_plan(trace, stop_waypoint_creation=True, clean_queue=True)

    def remaining_waypoints(self) -> int:
        return len(self._agent.get_local_planner().get_plan())

    def current_behavior(self) -> str:
        return road_option_name(self._agent.get_local_planner().target_road_option)

    def done(self) -> bool:
        return bool(self._agent.done())

    def reset(self) -> None:
        self._max_route_index = 0
        self._overtake.reset()
        self._execution.reset()
        self._car_follow_active = False
        self._waiting_on_light = False
        self._longitudinal_speed_error_kmh = 0.0
        self._previous_planner_state = None
        self._latched_red_light = None
        self._latched_red_until_s = -1.0

    def step(self, scene_state: SceneState) -> ControlDecision:
        current_speed_mps = scene_state.ego.speed_mps
        route_index = scene_state.route.route_index
        if route_index is not None:
            self._max_route_index = max(self._max_route_index, route_index)

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
            target_policy=build_stopped_obstacle_targets,
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
        lead_vehicle = overtake_scene.lead_vehicle
        active_overtake_target = overtake_scene.active_target
        lead_distance_m = overtake_scene.lead_distance_m
        lead_speed_mps = overtake_scene.lead_speed_mps
        same_lane_lead_distance_m = overtake_scene.same_lane_lead_distance_m
        same_lane_lead_speed_mps = overtake_scene.same_lane_lead_speed_mps
        closing_speed_mps = overtake_scene.closing_speed_mps
        min_ttc = overtake_scene.min_ttc
        same_lane_min_ttc = overtake_scene.same_lane_min_ttc
        follow_target_speed_kmh = overtake_scene.follow_target_speed_kmh
        lead_speed_kmh = lead_speed_mps * 3.6
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

        if self._overtake.state != "idle":
            transition = resolve_overtake_runtime_transition(
                state=self._overtake.state,
                aborted=self._overtake.aborted,
                current_lane_id=current_lane_id,
                target_lane_id=self._execution.target_lane_id,
                origin_lane_id=self._overtake.origin_lane_id,
                route_target_lane_id=scene_state.route.target_lane_id,
                lane_center_offset_m=lane_center_offset_m,
                should_stop_for_light=stop_for_light,
                target_speed_kmh=self.config.target_speed_kmh,
                follow_target_speed_kmh=follow_target_speed_kmh,
                lead_speed_kmh=lead_speed_kmh,
                overtake_speed_delta_kmh=self.config.overtake_speed_delta_kmh,
            )
            self._overtake.state = transition.state
            self._overtake.aborted = transition.aborted
            planner_state = transition.planner_state
            if transition.limited_target_speed_kmh is not None:
                target_speed_kmh = min(target_speed_kmh, transition.limited_target_speed_kmh)
            if transition.should_prepare_abort_return:
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
            if transition.event_overtake_abort:
                event_flags["event_overtake_abort"] = True
            if transition.completed:
                if transition.event_overtake_success:
                    event_flags["event_overtake_success"] = True
                self._overtake.reset()
                self._execution.clear()
                self._execution.resume_base_route(
                    route_index=route_index,
                    base_trace=self._base_trace,
                    route_point_to_trace_index=self._route_point_to_trace_index,
                )
                planner_state = "nominal_cruise"

        stop_target_distance_m = traffic_light_stop_target_distance_m(
            distance_m=(
                active_light.stop_line_distance_m
                if active_light is not None and active_light.stop_line_distance_m is not None
                else active_light.distance_m if active_light is not None else None
            ),
            stop_buffer_m=self.config.traffic_light_stop_buffer_m,
        )
        if stop_for_light and self._overtake.state == "idle":
            planner_state = "traffic_light_stop"
            target_speed_kmh = traffic_light_stop_target_speed_kmh(
                stop_target_distance_m=stop_target_distance_m,
                current_speed_mps=current_speed_mps,
                target_speed_kmh=self.config.target_speed_kmh,
                brake_start_distance_m=self.config.traffic_light_brake_start_distance_m,
                creep_resume_distance_m=self.config.traffic_light_creep_resume_distance_m,
                creep_speed_kmh=self.config.traffic_light_creep_speed_kmh,
            )
            if lead_vehicle is not None and not self.config.ignore_vehicles:
                # Red-light stopping still needs to respect a stopped or slow lead in the same lane.
                # Without this cap, the light-stop profile can keep throttle applied until the ego
                # reaches the stop line even when a stationary obstacle is sitting well before it.
                target_speed_kmh = min(target_speed_kmh, follow_target_speed_kmh)
                if (
                    self.config.allow_overtake
                    and active_light is not None
                    and active_light.state in {"red", "yellow"}
                ):
                    stop_distance = (
                        active_light.stop_line_distance_m
                        if active_light.stop_line_distance_m is not None
                        else active_light.distance_m
                    )
                    if stop_distance <= self.config.overtake_signal_suppression_distance_m:
                        overtake_considered = True
                        overtake_reject_reason = "signal_suppressed"
                        event_flags["event_unsafe_lane_change_reject"] = True
        elif (
            (lead_vehicle is not None or active_overtake_target is not None)
            and not self.config.ignore_vehicles
            and self._overtake.state == "idle"
        ):
            overtake_considered = True
            overtake_decision = choose_overtake_action(
                overtake_scene.decision_context,
                overtake_trigger_distance_m=self.config.overtake_trigger_distance_m,
                overtake_speed_delta_kmh=self.config.overtake_speed_delta_kmh,
                overtake_min_front_gap_m=self.config.overtake_min_front_gap_m,
                overtake_min_rear_gap_m=self.config.overtake_min_rear_gap_m,
                signal_suppression_distance_m=self.config.overtake_signal_suppression_distance_m,
            )
            if overtake_decision.planner_state == "lane_change_out" and overtake_decision.direction is not None:
                activation = self._execution.activate_overtake_plan(
                    carla_module=self._carla,
                    direction=overtake_decision.direction,
                    route_index=route_index,
                    base_trace=self._base_trace,
                    route_point_to_trace_index=self._route_point_to_trace_index,
                    distance_same_lane_m=self.config.lane_change_same_lane_distance_m,
                    lane_change_distance_m=self.config.lane_change_distance_m,
                    overtake_hold_distance_m=self.config.overtake_hold_distance_m,
                )
                if activation.activated:
                    self._overtake.begin_lane_change_out(
                        direction=overtake_decision.direction,
                        origin_lane_id=current_lane_id,
                        active_target=active_overtake_target,
                        lead_actor_id=lead_vehicle.actor_id if lead_vehicle is not None else None,
                        lead_lane_id=lead_vehicle.lane_id if lead_vehicle is not None else None,
                        lead_distance_m=lead_distance_m,
                        target_lane_id=activation.target_lane_id,
                    )
                    planner_state = "lane_change_out"
                    event_flags["event_overtake_attempt"] = True
                    overtake_reject_reason = None
                else:
                    event_flags["event_unsafe_lane_change_reject"] = True
                    planner_state = "car_follow"
                    target_speed_kmh = follow_target_speed_kmh
                    self._overtake.reset()
                    overtake_reject_reason = (
                        self._execution.lane_change_path.failure_reason or "lane_change_path_failed"
                    )
            else:
                overtake_reject_reason = overtake_decision.reject_reason
                if (
                    overtake_reject_reason == "lead_out_of_range"
                    and self.config.allow_overtake
                    and lead_distance_m is not None
                    and lead_distance_m > self.config.overtake_trigger_distance_m
                    and lead_speed_mps <= 0.3
                ):
                    planner_state = "nominal_cruise"
                    target_speed_kmh = self.config.target_speed_kmh
                else:
                    planner_state = "car_follow"
                    target_speed_kmh = follow_target_speed_kmh
                    if overtake_reject_reason in {
                        "adjacent_front_gap_insufficient",
                        "adjacent_rear_gap_insufficient",
                        "adjacent_lane_closed",
                        "signal_suppressed",
                    }:
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
            if (
                self._overtake.state == "pass_vehicle"
                and not self._overtake.aborted
                and should_begin_rejoin(
                    self._overtake.memory,
                    rejoin_front_gap_m=(
                        None if not math.isfinite(rejoin_front_gap_m) else rejoin_front_gap_m
                    ),
                    rejoin_rear_gap_m=(
                        None if not math.isfinite(rejoin_rear_gap_m) else rejoin_rear_gap_m
                    ),
                    overtake_min_front_gap_m=self.config.overtake_min_front_gap_m,
                    overtake_min_rear_gap_m=self.config.overtake_min_rear_gap_m,
                )
            ):
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

        target_waypoint = (
            self._execution.consume_next_waypoint(vehicle_location=self._vehicle.get_location())
            if self._overtake.state != "idle"
            else None
        )
        control = run_tracking_control(
            local_agent=self._agent,
            overtake_controller=self._overtake_controller,
            target_speed_kmh=target_speed_kmh,
            target_waypoint=target_waypoint,
        )

        emergency_stop = (
            not self.config.ignore_vehicles
            and same_lane_lead_distance_m is not None
            and (
                same_lane_lead_distance_m <= self.config.lead_brake_distance_m
                or (
                    math.isfinite(same_lane_min_ttc)
                    and same_lane_min_ttc <= self.config.ttc_emergency_threshold_s
                )
            )
        )
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
        if emergency_stop:
            planner_state = "emergency_brake"
            control.throttle = 0.0
            control.brake = max(float(control.brake), 0.8)

        event_flags["traffic_light_violation"] = is_traffic_light_violation(
            light_state=active_light.state if active_light is not None else None,
            stop_line_distance_m=(
                active_light.stop_line_distance_m if active_light is not None else None
            ),
            current_speed_mps=current_speed_mps,
        )

        behavior = self.current_behavior()
        planning_debug = build_overtake_planning_debug(
            remaining_waypoints=len(self._agent.get_local_planner().get_plan()),
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
            lead_vehicle=lead_vehicle,
            active_target=active_overtake_target,
            lead_distance_m=lead_distance_m,
            lead_speed_mps=lead_speed_mps,
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
            lane_change_path_available=self._execution.lane_change_path.available,
            lane_change_path_failed_reason=self._execution.lane_change_path.failure_reason,
            target_lane_id=target_lane_id,
            min_ttc=min_ttc,
            emergency_stop=emergency_stop,
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
