# Signal Suppressed Scenario

停止障害物はあるが、近い赤信号を優先して追い越しを開始しない scenario です。

## Status

- verified

## Run Config

- [town01_stopped_obstacle_signal_suppressed_long_expert.json](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_signal_suppressed_long_expert.json)

## Scenario Contract

- same-lane に停止障害物が 1 台
- 近傍 signal が red で、suppression distance 内にある

## Expectations

### Target Actor

- `follow_target_id` は停止障害物 actor に収束する

### Reject / Wait

- `signal_suppressed` で抑制される
- `lane_change_out` に入らない

### Pass / Rejoin

- route-loop は stall で終わる

## Why This Matters

- 停止障害物より信号規制を優先することを確認します。

### Summary Acceptance

- `success = false`
- `failure_reason = stalled`
- `collision_count = 0`
- `overtake_attempt_count = 0`

## Source Of Truth

- scenario matrix: [scenario_matrix.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/scenario_matrix.py)
- summary / manifest assertions: [assertions.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/assertions.py)
