# Curve Clear Scenario

緩いカーブ上の停止障害物を clear 条件で追い越す scenario です。

## Status

- verified

## Run Config

- [town01_stopped_obstacle_curve_clear_long_expert.json](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_curve_clear_long_expert.json)

## Verified Artifacts

- [summary.json](/home/masa/carla_alpamayo/outputs/evaluate/20260329_160234_town01_curve_overtake_clear_expert_eval_b924c52bb855/summary.json)
- [manifest.jsonl](/home/masa/carla_alpamayo/outputs/evaluate/20260329_160234_town01_curve_overtake_clear_expert_eval_b924c52bb855/manifest.jsonl)

## Scenario Contract

- カーブ区間上に same-lane 停止障害物が 1 台
- adjacent lane は clear
- junction / signal は十分遠い

## Expectations

### Target Actor

- `follow_target_id` は停止障害物 actor に収束する

### Reject / Wait

- trigger 条件を満たしたら `lane_change_out`

### Pass / Rejoin

- `lane_change_out -> pass_vehicle -> lane_change_back`
- route progress は単調増加
- opposite lane native direction に引っ張られない

### Summary Acceptance

- `success = true`
- `collision_count = 0`
- `overtake_success_count >= 1`

## Verification Verdict

- `PASS`
- `success = true`
- `collision_count = 0`
- `overtake_attempt_count = 1`
- `overtake_success_count = 1`

## Why This Matters

- route-aligned lane-change plan の単調性を実 world で確認する
