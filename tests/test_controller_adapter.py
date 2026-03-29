from __future__ import annotations

import unittest
from dataclasses import dataclass

from ad_stack.overtake.application.pure_pursuit_controller import PurePursuitController
from ad_stack.overtake.infrastructure.carla.controller_adapter import run_tracking_control
from tests.planning_controller_test_helpers import make_straight_route_backbone
from ad_stack.overtake.infrastructure.carla.trajectory_materializer import build_route_backbone_trajectory


@dataclass
class _FakeLocation:
    x: float
    y: float
    z: float = 0.0


@dataclass
class _FakeRotation:
    yaw: float


@dataclass
class _FakeTransform:
    location: _FakeLocation
    rotation: _FakeRotation


class _FakeVehicleControl:
    def __init__(
        self,
        *,
        throttle: float,
        steer: float,
        brake: float,
        hand_brake: bool,
        reverse: bool,
        manual_gear_shift: bool,
        gear: int,
    ) -> None:
        self.throttle = throttle
        self.steer = steer
        self.brake = brake
        self.hand_brake = hand_brake
        self.reverse = reverse
        self.manual_gear_shift = manual_gear_shift
        self.gear = gear


class _FakeCarlaModule:
    VehicleControl = _FakeVehicleControl


class ControllerAdapterTests(unittest.TestCase):
    def test_run_tracking_control_returns_vehicle_control_and_tracking_metrics(self) -> None:
        trajectory = build_route_backbone_trajectory(
            route_backbone=make_straight_route_backbone(num_points=80),
            start_route_index=0,
            desired_speed_mps=5.0,
            trajectory_id="lane_follow",
        )
        result = run_tracking_control(
            carla_module=_FakeCarlaModule(),
            controller=PurePursuitController(),
            ego_transform=_FakeTransform(
                location=_FakeLocation(0.0, 0.5),
                rotation=_FakeRotation(0.0),
            ),
            ego_speed_mps=5.0,
            trajectory=trajectory,
        )

        self.assertIsInstance(result.control, _FakeVehicleControl)
        self.assertAlmostEqual(result.desired_speed_mps, 5.0)
        self.assertGreaterEqual(result.lookahead_distance_m, 4.0)
        self.assertLessEqual(abs(result.control.steer), 1.0)
