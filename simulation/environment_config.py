from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from ad_stack.overtake.validation import (
    StoppedObstacleScenarioConfig,
    parse_stopped_obstacle_scenario_config,
)
from libs.project import PROJECT_ROOT


@dataclass(slots=True)
class NPCProfileSpec:
    name: str
    default_target_speed_kmh: float = 20.0
    speed_jitter_kmh: float = 0.0
    enable_autopilot: bool = True


@dataclass(slots=True)
class NPCSpawnTransformSpec:
    x: float
    y: float
    z: float
    yaw_deg: float
    pitch_deg: float = 0.0
    roll_deg: float = 0.0


@dataclass(slots=True)
class NPCVehicleSpec:
    spawn_index: int | None = None
    spawn_transform: NPCSpawnTransformSpec | None = None
    npc_profile_id: str | None = None
    target_speed_kmh: float | None = None
    lane_behavior: str = "keep_lane"
    vehicle_filter: str = "vehicle.*"


@dataclass(slots=True)
class TrafficLightOverrideSpec:
    actor_id: int
    state: str
    freeze: bool = True


@dataclass(slots=True)
class TrafficLightSchedulePhaseSpec:
    at_seconds: float
    state: str
    freeze: bool = True


@dataclass(slots=True)
class TrafficLightScheduleSpec:
    actor_id: int
    phases: list[TrafficLightSchedulePhaseSpec] = field(default_factory=list)


@dataclass(slots=True)
class TrafficLightPhaseCycleSpec:
    green_seconds: float
    yellow_seconds: float
    all_red_seconds: float
    reset_groups: bool = True
    initial_offset_seconds: float = 0.0


@dataclass(slots=True)
class EnvironmentConfigSpec:
    name: str
    town: str
    weather: str | None = None
    goal_tolerance_m: float | None = None
    max_stop_seconds: float | None = None
    stationary_speed_threshold_mps: float | None = None
    max_seconds: float | None = None
    npc_vehicles: list[NPCVehicleSpec] = field(default_factory=list)
    traffic_light_phase_cycle: TrafficLightPhaseCycleSpec | None = None
    traffic_light_overrides: list[TrafficLightOverrideSpec] = field(default_factory=list)
    traffic_light_schedules: list[TrafficLightScheduleSpec] = field(default_factory=list)
    stopped_obstacle_scenario: StoppedObstacleScenarioConfig | None = None
    description: str = ""


def _load_json(path: Path) -> dict[str, object]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def load_npc_profile(profile_id: str) -> NPCProfileSpec:
    profile_path = PROJECT_ROOT / "scenarios" / "npc_profiles" / f"{profile_id}.json"
    raw = _load_json(profile_path)
    return NPCProfileSpec(
        name=str(raw["name"]),
        default_target_speed_kmh=float(raw.get("default_target_speed_kmh", 20.0)),
        speed_jitter_kmh=float(raw.get("speed_jitter_kmh", 0.0)),
        enable_autopilot=bool(raw.get("enable_autopilot", True)),
    )


def load_environment_config(path: Path) -> EnvironmentConfigSpec:
    raw = _load_json(path)
    raw_cycle = raw.get("traffic_light_phase_cycle")
    if raw_cycle is None:
        raw_cycle = raw.get("traffic_light_group_cycle")
    raw_stopped_obstacle_scenario = raw.get("stopped_obstacle_scenario")
    all_red_seconds = None
    if raw_cycle is not None:
        all_red_seconds = raw_cycle.get("all_red_seconds")
        if all_red_seconds is None:
            all_red_seconds = raw_cycle["red_seconds"]
    return EnvironmentConfigSpec(
        name=str(raw["name"]),
        town=str(raw["town"]),
        weather=str(raw["weather"]) if raw.get("weather") is not None else None,
        goal_tolerance_m=float(raw["goal_tolerance_m"])
        if raw.get("goal_tolerance_m") is not None
        else None,
        max_stop_seconds=float(raw["max_stop_seconds"])
        if raw.get("max_stop_seconds") is not None
        else None,
        stationary_speed_threshold_mps=(
            float(raw["stationary_speed_threshold_mps"])
            if raw.get("stationary_speed_threshold_mps") is not None
            else None
        ),
        max_seconds=float(raw["max_seconds"]) if raw.get("max_seconds") is not None else None,
        npc_vehicles=[
            NPCVehicleSpec(
                spawn_index=(
                    int(item["spawn_index"]) if item.get("spawn_index") is not None else None
                ),
                spawn_transform=(
                    NPCSpawnTransformSpec(
                        x=float(item["spawn_transform"]["x"]),
                        y=float(item["spawn_transform"]["y"]),
                        z=float(item["spawn_transform"].get("z", 0.0)),
                        yaw_deg=float(item["spawn_transform"]["yaw_deg"]),
                        pitch_deg=float(item["spawn_transform"].get("pitch_deg", 0.0)),
                        roll_deg=float(item["spawn_transform"].get("roll_deg", 0.0)),
                    )
                    if item.get("spawn_transform") is not None
                    else None
                ),
                npc_profile_id=str(item["npc_profile_id"]) if item.get("npc_profile_id") else None,
                target_speed_kmh=float(item["target_speed_kmh"])
                if item.get("target_speed_kmh") is not None
                else None,
                lane_behavior=str(item.get("lane_behavior", "keep_lane")),
                vehicle_filter=str(item.get("vehicle_filter", "vehicle.*")),
            )
            for item in raw.get("npc_vehicles", [])
        ],
        traffic_light_phase_cycle=(
            TrafficLightPhaseCycleSpec(
                green_seconds=float(raw_cycle["green_seconds"]),
                yellow_seconds=float(raw_cycle["yellow_seconds"]),
                all_red_seconds=float(all_red_seconds),
                reset_groups=bool(raw_cycle.get("reset_groups", True)),
                initial_offset_seconds=float(raw_cycle.get("initial_offset_seconds", 0.0)),
            )
            if raw_cycle is not None
            else None
        ),
        traffic_light_overrides=[
            TrafficLightOverrideSpec(
                actor_id=int(item["actor_id"]),
                state=str(item["state"]),
                freeze=bool(item.get("freeze", True)),
            )
            for item in raw.get("traffic_light_overrides", [])
        ],
        traffic_light_schedules=[
            TrafficLightScheduleSpec(
                actor_id=int(item["actor_id"]),
                phases=[
                    TrafficLightSchedulePhaseSpec(
                        at_seconds=float(phase["at_seconds"]),
                        state=str(phase["state"]),
                        freeze=bool(phase.get("freeze", True)),
                    )
                    for phase in sorted(
                        item.get("phases", []),
                        key=lambda phase: float(phase["at_seconds"]),
                    )
                ],
            )
            for item in raw.get("traffic_light_schedules", [])
        ],
        stopped_obstacle_scenario=parse_stopped_obstacle_scenario_config(raw_stopped_obstacle_scenario),
        description=str(raw.get("description", "")),
    )
