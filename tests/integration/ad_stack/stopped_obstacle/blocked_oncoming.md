# Blocked Oncoming Scenario

前方停止障害物はあるが、隣接 lane に対向 moving vehicle が接近するため最初は出ず、通過後に追い越す baseline です。

## Run Config

- [town01_stopped_obstacle_blocked_oncoming_long_expert.json](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_blocked_oncoming_long_expert.json)

## Verified Artifacts

- [summary.json](/home/masa/carla_alpamayo/outputs/evaluate/20260329_042237_town01_northbound_overtake_long_expert_eval_cb0573c8fc8e/summary.json)
- [manifest.jsonl](/home/masa/carla_alpamayo/outputs/evaluate/20260329_042237_town01_northbound_overtake_long_expert_eval_cb0573c8fc8e/manifest.jsonl)
- [segment_0000.mcap](/home/masa/carla_alpamayo/outputs/evaluate/20260329_042237_town01_northbound_overtake_long_expert_eval_cb0573c8fc8e/telemetry/segment_0000.mcap)

## Expectations

### Target Actor

- `lead_vehicle_id` は停止障害物 actor に収束する
- oncoming actor を lead target と取り違えない

baseline 実績:

- `lead_ids = [286]`

### Reject / Wait

- 近づくまでは `lead_out_of_range`
- oncoming が近い間は `adjacent_front_gap_insufficient` または `adjacent_rear_gap_insufficient`
- 最初の overtake は reject
- 一定時間待機が入る

baseline 実績:

- frame `144`: `lead_out_of_range`
- frame `204`: `adjacent_front_gap_insufficient`
- frame `230`: `adjacent_rear_gap_insufficient`
- `unsafe_lane_change_reject_count = 62`

### Pass / Rejoin

- oncoming 通過後に `lane_change_out`
- その後 `pass_vehicle`
- 最後に `lane_change_back`
- `nominal_cruise / idle` に戻って完走

baseline 実績:

- `lane_change_out` 開始: frame `266`
- `pass_vehicle` 開始: frame `444`
- `lane_change_back` 開始: frame `572`
- `nominal_cruise / idle` 復帰: frame `610`

### Summary Acceptance

- `success = true`
- `collision_count = 0`
- `unsafe_lane_change_reject_count >= 1`
- `overtake_attempt_count >= 1`
- `overtake_success_count >= 1`

baseline 実績:

- `success = true`
- `collision_count = 0`
- `unsafe_lane_change_reject_count = 62`
- `overtake_attempt_count = 1`
- `overtake_success_count = 1`

## Verification Verdict

- `PASS`
- MCAP 上でも `lead_vehicle_id = 286` に収束し、oncoming actor `287` を target と取り違えていない
- frame `204` では same-lane obstacle `286` と left-lane oncoming `287` を同時に見て reject
- frame `266` で reject が消え、`lane_change_out` に入る
