# Blocked Oncoming Scenario

対向車がいる間は出ず、通過後に追い越す scenario です。

## Status

- verified

## Run Config

- [town01_stopped_obstacle_blocked_oncoming_long_expert.json](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_blocked_oncoming_long_expert.json)

## Scenario Contract

- same-lane に停止障害物が 1 台
- adjacent lane に oncoming actor が一時的に存在する

## Expectations

### Target Actor

- `follow_target_id` は停止障害物 actor に収束する
- oncoming actor を追い越し target と取り違えない

### Reject / Wait

- 対向車が近い間は lane gap reject が立つ
- gap 解放後に `lane_change_out` へ入る

### Pass / Rejoin

- 待機後に 1 回の追い越しで完走する

## Why This Matters

- “待ってから抜く” を見る baseline です。

### Summary Acceptance

- `success = true`
- `collision_count = 0`
- `overtake_attempt_count >= 1`
- `overtake_success_count >= 1`
- `unsafe_lane_change_reject_count >= 1`

## Source Of Truth

- scenario matrix: [scenario_matrix.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/scenario_matrix.py)
- summary / manifest assertions: [assertions.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/assertions.py)
