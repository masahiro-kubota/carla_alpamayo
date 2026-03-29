# Curve Clear With Opposite Static Blocked Scenario

カーブ区間で、route-aligned opposite lane 上の static blocker が pass corridor を塞いでいるため出ない scenario です。

## Status

- verified

## Run Config

- [town01_stopped_obstacle_curve_clear_with_opposite_static_blocked_long_expert.json](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_curve_clear_with_opposite_static_blocked_long_expert.json)

## Scenario Contract

- curve route 上に same-lane 停止障害物が 1 台ある
- route-aligned opposite lane 上に static blocker が 1 台ある
- blocker は required clear distance 内にある

## Expectations

### Target Actor

- same-lane 停止障害物を follow target として維持する
- curve geometry でも opposite static actor を blocker として観測する

### Reject / Wait

- `adjacent_rear_gap_insufficient` か等価な corridor reject で抑制される
- `lane_change_out` に入らない

### Pass / Rejoin

- collision せず stall で終わる

## Why This Matters

- 直線ではなく curve でも pass corridor blocker 判定が効くことを確認するためです。

### Summary Acceptance

- `success = false`
- `failure_reason = stalled`
- `collision_count = 0`
- `overtake_attempt_count = 0`
- `unsafe_lane_change_reject_count >= 1`

### Manifest Acceptance

- some manifest row has `overtake_reject_reason = adjacent_rear_gap_insufficient`

## Source Of Truth

- scenario matrix: [scenario_matrix.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/scenario_matrix.py)
- summary / manifest assertions: [assertions.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/assertions.py)
