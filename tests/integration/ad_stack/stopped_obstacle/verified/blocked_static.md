# Blocked Static Scenario

停止障害物はあるが、隣接 lane に静的 blocker がいて出られない scenario です。

## Status

- verified

## Run Config

- [town01_stopped_obstacle_blocked_long_expert.json](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_blocked_long_expert.json)

## Scenario Contract

- same-lane に停止障害物が 1 台
- adjacent lane front gap を静的 blocker が塞ぐ

## Expectations

### Target Actor

- `follow_target_id` は停止障害物 actor に収束する
- blocker actor を追い越し target と取り違えない

### Reject / Wait

- `adjacent_front_gap_insufficient` または等価な gap reject で抑制される
- `lane_change_out` に入らない

### Pass / Rejoin

- route-loop は stall で終わる
- collision は起こさない

## Why This Matters

- gap 不足で出ないことを保証する negative baseline です。

### Summary Acceptance

- `success = false`
- `failure_reason = stalled`
- `collision_count = 0`
- `overtake_attempt_count = 0`
- `unsafe_lane_change_reject_count >= 1`

## Source Of Truth

- scenario matrix: [scenario_matrix.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/scenario_matrix.py)
- summary / manifest assertions: [assertions.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/assertions.py)
