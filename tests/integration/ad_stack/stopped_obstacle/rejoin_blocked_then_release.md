# Rejoin Blocked Then Release Scenario

追い越し開始はできるが、元レーンへの復帰 gap が最初は足りず、一時的に target lane を維持してから戻る scenario です。

## Status

- verified

## Run Config

- [town01_stopped_obstacle_rejoin_blocked_then_release_long_expert.json](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_rejoin_blocked_then_release_long_expert.json)

## Verified Artifacts

- [summary.json](/home/masa/carla_alpamayo/outputs/evaluate/20260329_164315_town01_northbound_overtake_long_expert_eval_69ede1ee9eca/summary.json)
- [manifest.jsonl](/home/masa/carla_alpamayo/outputs/evaluate/20260329_164315_town01_northbound_overtake_long_expert_eval_69ede1ee9eca/manifest.jsonl)
- [segment_0000.mcap](/home/masa/carla_alpamayo/outputs/evaluate/20260329_164315_town01_northbound_overtake_long_expert_eval_69ede1ee9eca/telemetry/segment_0000.mcap)

## Scenario Contract

- same-lane に停止障害物が 1 台
- opposite lane への `lane_change_out` は可能
- ただし rejoin 開始時点の origin lane rear/front gap は不足
- 数秒後に rejoin gap が解放される

## Expectations

### Target Actor

- `lead_vehicle_id` は停止障害物 actor に収束する

### Reject / Wait

- `lane_change_out` は成立する
- `pass_vehicle` 中、rejoin gap 不足のため復帰を待つ

### Pass / Rejoin

- `lane_change_out -> pass_vehicle`
- すぐには `lane_change_back` に入らない
- gap 解放後に `lane_change_back`
- `nominal_cruise / idle` に戻る

### Summary Acceptance

- `collision_count = 0`
- `overtake_attempt_count >= 1`
- `overtake_success_count >= 1`
- `first_target_passed_s` と `first_rejoin_started_s` が両方出る
- `rejoin_wait_after_target_passed_s > 0`
- `first_rejoin_rear_gap_m >= 15.0`

## Verification Verdict

- `PASS`
- `first_target_passed_s = 23.65`
- `first_rejoin_started_s = 24.25`
- `rejoin_wait_after_target_passed_s = 0.6`
- `first_rejoin_rear_gap_m = 15.244`
- manifest 上でも `target_passed=true` 後に 6 frame `pass_vehicle` を維持し、その後 `lane_change_back` に入る

## Why This Matters

- 今の baseline では「戻れるときに戻る」しか見ていない
- `pass 完了` と `rejoin 開始` を actor / gap 基準で分離できているかを確認する
