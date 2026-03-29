from __future__ import annotations

import unittest

from ad_stack.overtake.domain.planning_models import PlanningScene, Pose3D
from tests.planning_controller_test_helpers import make_scene


class PlanningSceneContractTests(unittest.TestCase):
    def test_planning_scene_requires_all_contract_fields(self) -> None:
        with self.assertRaises(TypeError):
            PlanningScene(  # type: ignore[call-arg]
                ego_pose=Pose3D(x=0.0, y=0.0, z=0.0, yaw_deg=0.0),
                ego_speed_mps=1.0,
                route_index=0,
                route_progress_m=0.0,
                current_lane_id="15:-1",
                tracked_targets=(),
                traffic_lights=(),
            )

    def test_planning_scene_rejects_negative_route_progress(self) -> None:
        with self.assertRaisesRegex(ValueError, "route_progress_m must be non-negative"):
            make_scene(route_progress_m=-1.0)

    def test_planning_scene_accepts_valid_input(self) -> None:
        scene = make_scene(route_index=4, route_progress_m=4.0)

        self.assertEqual(scene.route_index, 4)
        self.assertEqual(scene.current_lane_id, "15:-1")
