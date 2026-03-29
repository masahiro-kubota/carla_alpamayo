from __future__ import annotations

import unittest
from types import SimpleNamespace

from ad_stack.overtake.infrastructure.carla.candidate_extractor import build_target_candidates
from ad_stack.overtake.infrastructure.carla.scene_assembler import build_overtake_scene_snapshot
from ad_stack.overtake.policies import build_stopped_obstacle_targets


class _LaneType:
    Driving = "Driving"


class _CarlaModule:
    LaneType = _LaneType

    @staticmethod
    def Location(*, x: float, y: float, z: float) -> SimpleNamespace:
        return SimpleNamespace(x=x, y=y, z=z)


def _waypoint(road_id: int, lane_id: int) -> SimpleNamespace:
    waypoint = SimpleNamespace(
        road_id=road_id,
        lane_id=lane_id,
        lane_type=_LaneType.Driving,
        transform=SimpleNamespace(
            location=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        ),
    )
    waypoint.get_left_lane = lambda: SimpleNamespace(
        road_id=road_id,
        lane_id=1,
        lane_type=_LaneType.Driving,
        transform=SimpleNamespace(location=SimpleNamespace(x=0.0, y=0.0, z=0.0)),
    )
    waypoint.get_right_lane = lambda: None
    return waypoint


class _WorldMap:
    def get_waypoint(self, _location: SimpleNamespace, lane_type: str | None = None) -> SimpleNamespace:
        del lane_type
        return _waypoint(15, -1)


class OvertakeSceneAssemblerTests(unittest.TestCase):
    def test_stopped_lead_uses_ten_kmh_follow_cap_instead_of_two_kmh(self) -> None:
        tracked_objects = (
            SimpleNamespace(
                actor_id=101,
                relation="same_lane",
                is_ahead=True,
                longitudinal_distance_m=20.0,
                speed_mps=0.0,
                lane_id="15:-1",
                x_m=20.0,
                y_m=0.0,
            ),
        )

        snapshot = build_overtake_scene_snapshot(
            tracked_objects=tracked_objects,
            timestamp_s=0.0,
            current_speed_mps=20.0 / 3.6,
            current_lane_id="15:-1",
            route_target_lane_id="15:-1",
            route_index=0,
            ego_waypoint=_waypoint(15, -1),
            adjacent_lanes_open={"left": True, "right": False},
            target_speed_kmh=20.0,
            follow_headway_seconds=1.8,
            stopped_speed_threshold_mps=0.3,
            cluster_merge_gap_m=10.0,
            cluster_max_member_speed_mps=0.5,
            candidate_builder=build_target_candidates,
            target_policy=build_stopped_obstacle_targets,
            active_signal_state=None,
            signal_stop_distance_m=None,
            allow_overtake=True,
            preferred_direction="left_first",
            world_map=_WorldMap(),
            carla_module=_CarlaModule(),
            base_trace=[],
            route_point_to_trace_index=[],
            route_point_progress_m=[],
        )

        self.assertEqual(snapshot.follow_target_speed_kmh, 10.0)


if __name__ == "__main__":
    unittest.main()
