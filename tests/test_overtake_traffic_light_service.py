from __future__ import annotations

import unittest
from dataclasses import dataclass

from ad_stack.overtake import resolve_active_light, select_active_light


@dataclass
class _Light:
    actor_id: int
    state: str
    affects_ego: bool
    distance_m: float
    stop_line_distance_m: float | None = None


class OvertakeTrafficLightServiceTests(unittest.TestCase):
    def test_select_active_light_prefers_nearest_stop_line(self) -> None:
        light = select_active_light(
            (
                _Light(actor_id=1, state="green", affects_ego=True, distance_m=20.0, stop_line_distance_m=8.0),
                _Light(actor_id=2, state="red", affects_ego=True, distance_m=15.0, stop_line_distance_m=4.0),
            )
        )

        self.assertIsNotNone(light)
        self.assertEqual(light.actor_id, 2)

    def test_resolve_active_light_latches_red(self) -> None:
        red_light = _Light(actor_id=7, state="red", affects_ego=True, distance_m=10.0, stop_line_distance_m=6.0)
        active_light, latched_light, latched_until_s = resolve_active_light(
            traffic_lights=(red_light,),
            timestamp_s=5.0,
            latched_red_light=None,
            latched_red_until_s=-1.0,
            red_latch_seconds=0.5,
        )

        self.assertEqual(active_light.actor_id, 7)
        self.assertEqual(latched_light.actor_id, 7)
        self.assertEqual(latched_until_s, 5.5)

        resolved_light, _, _ = resolve_active_light(
            traffic_lights=(),
            timestamp_s=5.2,
            latched_red_light=latched_light,
            latched_red_until_s=latched_until_s,
            red_latch_seconds=0.5,
        )
        self.assertEqual(resolved_light.actor_id, 7)

