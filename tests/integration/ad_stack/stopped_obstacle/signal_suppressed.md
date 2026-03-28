# Signal Suppressed Scenario

停止障害物は前方にあるが、その先の信号条件により追い越しを開始してはいけない scenario です。

## Status

- implemented_not_verified

## Planned Run Config

- `tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_signal_suppressed_long_expert.json`

## Scenario Contract

- same-lane に停止障害物が 1 台
- adjacent lane は gap 的には成立している
- ただし red / yellow signal が suppression 距離内にある

## Expectations

### Target Actor

- `lead_vehicle_id` は停止障害物 actor に収束する

### Reject / Wait

- `overtake_reject_reason = signal_suppressed`
- `lane_change_out` に入らない
- signal が解放されるまで `car_follow` か `traffic_light_stop`

### Pass / Rejoin

- `pass_vehicle` に入らない
- `lane_change_back` に入らない

### Summary Acceptance

- `collision_count = 0`
- `overtake_attempt_count = 0`
- `unsafe_lane_change_reject_count >= 1`

## Why This Matters

- pure logic にある `signal_suppressed` を runtime で本当に踏めるかを確認する
- `gap が空いている = 出る` になっていないことを保証する
