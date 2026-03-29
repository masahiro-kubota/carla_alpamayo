from __future__ import annotations

import unittest
from dataclasses import dataclass

from ad_stack.overtake.infrastructure.carla.route_backbone_builder import (
    build_route_backbone,
    normalize_route_command,
)


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
    road_id: int
    lane_id: int
    transform: _FakeTransform


class _FakeOption:
    def __init__(self, name: str) -> None:
        self.name = name


class RouteBackboneBuilderTests(unittest.TestCase):
    def test_build_route_backbone_normalizes_route_commands(self) -> None:
        trace = [
            (
                _FakeWaypoint(
                    road_id=15,
                    lane_id=-1,
                    transform=_FakeTransform(_FakeLocation(0.0, 0.0), _FakeRotation(0.0)),
                ),
                _FakeOption("LANEFOLLOW"),
            ),
            (
                _FakeWaypoint(
                    road_id=15,
                    lane_id=-1,
                    transform=_FakeTransform(_FakeLocation(1.0, 0.0), _FakeRotation(0.0)),
                ),
                _FakeOption("LEFT"),
            ),
            (
                _FakeWaypoint(
                    road_id=15,
                    lane_id=-1,
                    transform=_FakeTransform(_FakeLocation(2.0, 0.0), _FakeRotation(0.0)),
                ),
                _FakeOption("STRAIGHT"),
            ),
        ]

        route_backbone = build_route_backbone(trace)

        self.assertEqual(route_backbone.route_index_to_road_option[0], "lane_follow")
        self.assertEqual(route_backbone.route_index_to_road_option[1], "left")
        self.assertEqual(route_backbone.route_index_to_road_option[2], "straight")
        self.assertEqual(route_backbone.route_index_to_lane_id[0], "15:-1")

    def test_normalize_route_command_defaults_unknown_to_lane_follow(self) -> None:
        self.assertEqual(normalize_route_command(None), "lane_follow")
        self.assertEqual(normalize_route_command(_FakeOption("VOID")), "lane_follow")

