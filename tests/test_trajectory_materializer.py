from __future__ import annotations

import unittest
from dataclasses import dataclass

from ad_stack.overtake.infrastructure.carla.trajectory_materializer import (
    build_route_backbone_trajectory,
    build_waypoint_trajectory,
)
from tests.planning_controller_test_helpers import make_straight_route_backbone


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


@dataclass
class _FakeWaypoint:
    transform: _FakeTransform


class TrajectoryMaterializerTests(unittest.TestCase):
    def test_build_route_backbone_trajectory_resamples_to_one_meter_spacing(self) -> None:
        trajectory = build_route_backbone_trajectory(
            route_backbone=make_straight_route_backbone(num_points=80),
            start_route_index=5,
            desired_speed_mps=5.0,
            trajectory_id="lane_follow",
        )

        self.assertGreaterEqual(len(trajectory.points), 30)
        self.assertEqual(trajectory.trajectory_id, "lane_follow")
        self.assertEqual(trajectory.source_route_start_index, 5)

    def test_build_waypoint_trajectory_handles_single_point_by_extrapolating(self) -> None:
        trajectory = build_waypoint_trajectory(
            waypoints=[
                _FakeWaypoint(
                    transform=_FakeTransform(
                        location=_FakeLocation(10.0, 2.0),
                        rotation=_FakeRotation(90.0),
                    )
                )
            ],
            desired_speed_mps=4.0,
            trajectory_id="lane_change_out",
            origin_lane_id="15:-1",
            target_lane_id="15:1",
        )

        self.assertEqual(len(trajectory.points), 2)
        self.assertEqual(trajectory.origin_lane_id, "15:-1")
        self.assertEqual(trajectory.target_lane_id, "15:1")
        self.assertAlmostEqual(trajectory.points[1].y - trajectory.points[0].y, 1.0, places=3)

