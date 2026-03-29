# Adjacent Lane Closed Scenario

停止障害物はあるが、隣接 lane 自体が利用不可で出られない scenario です。

## Status

- verified

## Run Config

- [town01_stopped_obstacle_adjacent_lane_closed_long_expert.json](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_adjacent_lane_closed_long_expert.json)

## Scenario Contract

- same-lane に停止障害物が 1 台
- route-aligned adjacent lane が `Driving` でないか corridor として利用不可

## Expectations

### Target Actor

- `follow_target_id` は停止障害物 actor に収束する

### Reject / Wait

- `adjacent_lane_closed` で抑制される
- gap の大小ではなく lane availability で reject される

### Pass / Rejoin

- route-loop は stall で終わる

## Why This Matters

- gap 不足と lane unavailable を分けて扱えているかを見る scenario です。

### Summary Acceptance

- `success = false`
- `failure_reason = stalled`
- `collision_count = 0`
- `overtake_attempt_count = 0`
- `unsafe_lane_change_reject_count >= 1`

## Source Of Truth

- scenario matrix: [scenario_matrix.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/scenario_matrix.py)
- summary / manifest assertions: [assertions.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/assertions.py)
