from __future__ import annotations

import unittest

from tests.integration.ad_stack._shared import (
    ManifestExpectation,
    assert_any_field_equals,
    assert_manifest_expectations,
    assert_any_sequence_len_at_least,
    assert_min_numeric_field_where_equals,
    assert_no_field_equals,
    assert_min_unique_non_null_values,
)


class IntegrationManifestAssertionsTest(unittest.TestCase):
    def test_assert_min_unique_non_null_values_accepts_distinct_values(self) -> None:
        rows = [
            {"overtake_target_actor_id": 101},
            {"overtake_target_actor_id": None},
            {"overtake_target_actor_id": 202},
        ]

        assert_min_unique_non_null_values(
            rows,
            field="overtake_target_actor_id",
            min_unique=2,
            message="expected at least two target actors",
        )

    def test_assert_any_field_equals_accepts_matching_row(self) -> None:
        rows = [
            {"overtake_target_kind": "single_actor"},
            {"overtake_target_kind": "cluster"},
        ]

        assert_any_field_equals(
            rows,
            field="overtake_target_kind",
            expected="cluster",
            message="expected cluster row",
        )

    def test_assert_any_sequence_len_at_least_accepts_long_enough_sequence(self) -> None:
        rows = [
            {"overtake_target_member_actor_ids": [301]},
            {"overtake_target_member_actor_ids": [301, 302]},
        ]

        assert_any_sequence_len_at_least(
            rows,
            field="overtake_target_member_actor_ids",
            min_len=2,
            message="expected cluster members",
        )

    def test_assert_no_field_equals_accepts_when_value_is_absent(self) -> None:
        rows = [
            {"overtake_reject_reason": None},
            {"overtake_reject_reason": "target_out_of_range"},
        ]

        assert_no_field_equals(
            rows,
            field="overtake_reject_reason",
            expected="adjacent_front_gap_insufficient",
            message="unexpected front-gap reject",
        )

    def test_assert_min_numeric_field_where_equals_accepts_threshold(self) -> None:
        rows = [
            {"overtake_state": "idle", "target_speed_kmh": 30.0},
            {"overtake_state": "lane_change_out", "target_speed_kmh": 15.0},
            {"overtake_state": "lane_change_out", "target_speed_kmh": 18.0},
        ]

        assert_min_numeric_field_where_equals(
            rows,
            field="target_speed_kmh",
            filter_field="overtake_state",
            filter_equals="lane_change_out",
            min_value=15.0,
            message="lane_change_out target speed collapsed below floor",
        )

    def test_assert_manifest_expectations_accepts_mixed_contract(self) -> None:
        rows = [
            {
                "overtake_target_actor_id": 101,
                "overtake_target_kind": "cluster",
                "overtake_target_member_actor_ids": [101, 102],
            },
            {
                "overtake_target_actor_id": 202,
                "overtake_target_kind": "single_actor",
                "overtake_target_member_actor_ids": [202],
            },
            {
                "overtake_state": "lane_change_out",
                "target_speed_kmh": 15.0,
            },
        ]

        assert_manifest_expectations(
            rows,
            (
                ManifestExpectation(
                    field="overtake_target_actor_id",
                    kind="min_unique_non_null",
                    min_unique=2,
                    message="expected target switch",
                ),
                ManifestExpectation(
                    field="overtake_target_kind",
                    kind="any_equals",
                    expected="cluster",
                    message="expected cluster row",
                ),
                ManifestExpectation(
                    field="overtake_target_kind",
                    kind="none_equals",
                    expected="failed",
                    message="unexpected failed row",
                ),
                ManifestExpectation(
                    field="overtake_target_member_actor_ids",
                    kind="any_sequence_len_at_least",
                    min_len=2,
                    message="expected cluster members",
                ),
                ManifestExpectation(
                    field="target_speed_kmh",
                    kind="min_numeric_where_equals",
                    filter_field="overtake_state",
                    filter_equals="lane_change_out",
                    min_value=15.0,
                    message="expected lane_change_out target speed floor",
                ),
            ),
        )


if __name__ == "__main__":
    unittest.main()
