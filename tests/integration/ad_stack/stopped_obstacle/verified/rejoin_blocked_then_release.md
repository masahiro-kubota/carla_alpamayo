# Rejoin Blocked Then Release Scenario

追い越し後、rejoin gap が一時的に不足し、解放後に戻る scenario です。

## Status

- verified

## Run Config

- [town01_stopped_obstacle_rejoin_blocked_then_release_long_expert.json](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_rejoin_blocked_then_release_long_expert.json)

## Scenario Contract

- same-lane に停止障害物が 1 台
- 元 lane 側に moving blocker がいて rejoin gap を一時的に塞ぐ

## Expectations

### Target Actor

- `follow_target_id` は停止障害物 actor に収束する

### Reject / Wait

- `lane_change_out` は成立する
- `pass_vehicle` 中、gap 不足の間は `lane_change_back` に入らない

### Pass / Rejoin

- target を抜いた後に positive な wait があり、gap 解放後に `lane_change_back` に入る

## Why This Matters

- `pass 完了` と `rejoin 開始` が gap 基準で分かれているかを見る scenario です。

### Summary Acceptance

- `success = true`
- `collision_count = 0`
- `overtake_attempt_count >= 1`
- `overtake_success_count >= 1`
- `rejoin_wait_after_target_passed_s > 0`

## Source Of Truth

- scenario matrix: [scenario_matrix.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/scenario_matrix.py)
- summary / manifest assertions: [assertions.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/assertions.py)
