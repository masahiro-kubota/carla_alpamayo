# Rejoin Blocked Then Release Scenario

追い越し開始はできるが、元レーンへの復帰 gap が最初は足りず、一時的に target lane を維持してから戻る scenario です。

## Status

- implemented_not_verified

## Planned Run Config

- `tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_rejoin_blocked_then_release_long_expert.json`

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
- `first_rejoin_started_s` が `first_target_passed_s` より十分後ろ

## Why This Matters

- 今の baseline では「戻れるときに戻る」しか見ていない
- `pass 完了` と `rejoin 開始` を actor / gap 基準で分離できているかを確認する
