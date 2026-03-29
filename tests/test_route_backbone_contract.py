from __future__ import annotations

import unittest

from ad_stack.overtake.domain.planning_models import RouteBackbone
from tests.planning_controller_test_helpers import make_straight_route_backbone


class RouteBackboneContractTests(unittest.TestCase):
    def test_route_command_and_lane_id_are_deterministic(self) -> None:
        backbone = make_straight_route_backbone(num_points=8, lane_id="15:-1", road_option="left")

        self.assertEqual(backbone.road_option_for_index(3), "left")
        self.assertEqual(backbone.lane_id_for_index(3), "15:-1")
        self.assertEqual(backbone.trace_index_for_route_index(3), 3)

    def test_progress_must_be_strictly_increasing(self) -> None:
        backbone = make_straight_route_backbone(num_points=5)

        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            RouteBackbone(
                trace=backbone.trace,
                xy_points=backbone.xy_points,
                progress_m=(0.0, 1.0, 1.0, 3.0, 4.0),
                route_index_to_trace_index=backbone.route_index_to_trace_index,
                route_index_to_road_option=backbone.route_index_to_road_option,
                route_index_to_lane_id=backbone.route_index_to_lane_id,
            )
