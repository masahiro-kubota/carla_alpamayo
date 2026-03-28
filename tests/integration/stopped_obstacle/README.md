# Stopped Obstacle Scenario Suite

停止障害物回避の `CARLA` 結合テストを読むときの、期待値チェック用メモです。

目的:

- 各 scenario で何を期待するかを先に固定する
- `MCAP` / `manifest.jsonl` / `summary.json` を見たときに、成功か失敗かをすぐ判定できるようにする
- 以後の stopped-obstacle 調整で、判断の一貫性を崩さないようにする

suite location:

- run-configs:
  - [run_configs](/home/masa/carla_alpamayo/tests/integration/stopped_obstacle/run_configs)
- regression runner:
  - [run_stopped_obstacle_regressions.sh](/home/masa/carla_alpamayo/tests/integration/stopped_obstacle/run_stopped_obstacle_regressions.sh)

baseline regression set:

- [town01_stopped_obstacle_clear_long_expert.json](/home/masa/carla_alpamayo/tests/integration/stopped_obstacle/run_configs/town01_stopped_obstacle_clear_long_expert.json)
- [town01_stopped_obstacle_blocked_long_expert.json](/home/masa/carla_alpamayo/tests/integration/stopped_obstacle/run_configs/town01_stopped_obstacle_blocked_long_expert.json)
- [town01_stopped_obstacle_blocked_oncoming_long_expert.json](/home/masa/carla_alpamayo/tests/integration/stopped_obstacle/run_configs/town01_stopped_obstacle_blocked_oncoming_long_expert.json)

latest verified run set:

- clear:
  - [summary.json](/home/masa/carla_alpamayo/outputs/evaluate/20260329_042204_town01_northbound_overtake_long_expert_eval_cb0573c8fc8e/summary.json)
  - [segment_0000.mcap](/home/masa/carla_alpamayo/outputs/evaluate/20260329_042204_town01_northbound_overtake_long_expert_eval_cb0573c8fc8e/telemetry/segment_0000.mcap)
- blocked_static:
  - [summary.json](/home/masa/carla_alpamayo/outputs/evaluate/20260329_042220_town01_northbound_overtake_long_expert_eval_cb0573c8fc8e/summary.json)
  - [segment_0000.mcap](/home/masa/carla_alpamayo/outputs/evaluate/20260329_042220_town01_northbound_overtake_long_expert_eval_cb0573c8fc8e/telemetry/segment_0000.mcap)
- blocked_oncoming:
  - [summary.json](/home/masa/carla_alpamayo/outputs/evaluate/20260329_042237_town01_northbound_overtake_long_expert_eval_cb0573c8fc8e/summary.json)
  - [segment_0000.mcap](/home/masa/carla_alpamayo/outputs/evaluate/20260329_042237_town01_northbound_overtake_long_expert_eval_cb0573c8fc8e/telemetry/segment_0000.mcap)

補足:

- `lane_invasion_count` は baseline の品質ゲートに含めない
- 重要なのは
  - `collision_count = 0`
  - 期待どおりの `overtake / reject`
  - `route_completion_ratio` または意図した `stalled`
  である

## 1. Common Checks

全 scenario 共通で見るもの:

- `summary.json`
  - `success`
  - `failure_reason`
  - `collision_count`
  - `overtake_attempt_count`
  - `overtake_success_count`
  - `unsafe_lane_change_reject_count`
- `manifest.jsonl`
  - `lead_vehicle_id`
  - `overtake_reject_reason`
  - `planner_state`
  - `overtake_state`
  - `current_lane_id`
  - `target_lane_id`

共通の期待:

- 停止障害物を `lead_vehicle_id` として見ていること
- `overtake_state` は `idle -> lane_change_out -> pass_vehicle -> lane_change_back -> idle`
  の順で進むか、もしくは `idle` のまま reject されること
- `collision_count` は常に `0`

## 2. Clear

対象 artifact:

- [summary.json](/home/masa/carla_alpamayo/outputs/evaluate/20260329_042204_town01_northbound_overtake_long_expert_eval_cb0573c8fc8e/summary.json)
- [manifest.jsonl](/home/masa/carla_alpamayo/outputs/evaluate/20260329_042204_town01_northbound_overtake_long_expert_eval_cb0573c8fc8e/manifest.jsonl)

### 2.1 Target Actor Selection

期待値:

- `lead_vehicle_id` は停止障害物の actor に収束する
- `lead_vehicle_id` は run 中に別 actor へふらつかない
- `current_lane_id` は最初 `15:-1`

baseline 実績:

- `lead_ids = [275]`

### 2.2 Reject / Wait

期待値:

- 障害物が遠い間は `overtake_reject_reason = lead_out_of_range`
- trigger 距離に入ったら reject は消え、`lane_change_out` へ遷移する
- `blocked` 系のような `adjacent_front_gap_insufficient` は出ない

baseline 実績:

- reject transition:
  - frame `144`: `lead_out_of_range`

### 2.3 Pass / Rejoin

期待値:

- `lane_change_out` に入る
- その後 `pass_vehicle`
- target を抜いたあと `lane_change_back`
- 最後に `nominal_cruise / idle` に戻る
- `route_completion_ratio = 1.0`

baseline 実績:

- `lane_change_out` 開始: frame `204`
- `pass_vehicle` 開始: frame `346`
- `lane_change_back` 開始: frame `484`
- `nominal_cruise / idle` 復帰: frame `524`

### 2.4 Summary-Level Acceptance

期待値:

- `success = true`
- `collision_count = 0`
- `overtake_attempt_count >= 1`
- `overtake_success_count >= 1`

baseline 実績:

- `success = true`
- `collision_count = 0`
- `overtake_attempt_count = 1`
- `overtake_success_count = 1`

verification:

- `PASS`
- MCAP でも `lead_vehicle_id = 275` に収束
- frame `204` で `lane_change_out`
- frame `346` で `pass_vehicle`
- frame `484` で `lane_change_back`
- frame `524` で `nominal_cruise / idle` へ復帰

## 3. Blocked Static

対象 artifact:

- [summary.json](/home/masa/carla_alpamayo/outputs/evaluate/20260329_042220_town01_northbound_overtake_long_expert_eval_cb0573c8fc8e/summary.json)
- [manifest.jsonl](/home/masa/carla_alpamayo/outputs/evaluate/20260329_042220_town01_northbound_overtake_long_expert_eval_cb0573c8fc8e/manifest.jsonl)

### 3.1 Target Actor Selection

期待値:

- `lead_vehicle_id` は停止障害物の actor に収束する
- opposite lane の static blocker を overtake target にしない

baseline 実績:

- `lead_ids = [280]`

### 3.2 Reject / Wait

期待値:

- 近づくまでは `lead_out_of_range`
- trigger 距離に入っても opposite lane が塞がっているので `adjacent_front_gap_insufficient`
- 末尾側では `adjacent_rear_gap_insufficient` へ変わることはありうる
- `lane_change_out` には入らない
- `planner_state` は `car_follow`

baseline 実績:

- reject transitions:
  - frame `144`: `lead_out_of_range`
  - frame `204`: `adjacent_front_gap_insufficient`
  - frame `774`: `adjacent_rear_gap_insufficient`
- state transition:
  - frame `204`: `car_follow / idle`

### 3.3 Pass / Rejoin

期待値:

- `pass_vehicle` に入らない
- `lane_change_back` に入らない
- route 完了ではなく `stalled` で終了する

baseline 実績:

- `overtake_state` は最後まで `idle`
- `failure_reason = stalled`

### 3.4 Summary-Level Acceptance

期待値:

- `success = false`
- `failure_reason = stalled`
- `collision_count = 0`
- `overtake_attempt_count = 0`
- `unsafe_lane_change_reject_count >= 1`

baseline 実績:

- `success = false`
- `failure_reason = stalled`
- `collision_count = 0`
- `overtake_attempt_count = 0`
- `unsafe_lane_change_reject_count = 610`

verification:

- `PASS`
- MCAP でも `lead_vehicle_id = 280` を停止障害物として見ている
- frame `204` 時点で
  - same-lane obstacle `280`
  - left-lane blocker `281`
  を同時に見ている
- reject は `adjacent_front_gap_insufficient`
- `lane_change_out` に入らず、最後まで `car_follow / idle`

## 4. Blocked Oncoming

対象 artifact:

- [summary.json](/home/masa/carla_alpamayo/outputs/evaluate/20260329_042237_town01_northbound_overtake_long_expert_eval_cb0573c8fc8e/summary.json)
- [manifest.jsonl](/home/masa/carla_alpamayo/outputs/evaluate/20260329_042237_town01_northbound_overtake_long_expert_eval_cb0573c8fc8e/manifest.jsonl)

### 4.1 Target Actor Selection

期待値:

- `lead_vehicle_id` は停止障害物の actor に収束する
- oncoming actor を lead target と取り違えない

baseline 実績:

- `lead_ids = [286]`

### 4.2 Reject / Wait

期待値:

- 近づくまでは `lead_out_of_range`
- oncoming が近い間は `adjacent_front_gap_insufficient` あるいは `adjacent_rear_gap_insufficient`
- 最初の overtake は reject される
- 一定時間待機が入る

baseline 実績:

- reject transitions:
  - frame `144`: `lead_out_of_range`
  - frame `204`: `adjacent_front_gap_insufficient`
  - frame `230`: `adjacent_rear_gap_insufficient`
- `unsafe_lane_change_reject_count = 62`

### 4.3 Pass / Rejoin

期待値:

- oncoming 通過後に `lane_change_out`
- その後 `pass_vehicle`
- 最後に `lane_change_back`
- `nominal_cruise / idle` に戻って完走

baseline 実績:

- `lane_change_out` 開始: frame `266`
- `pass_vehicle` 開始: frame `444`
- `lane_change_back` 開始: frame `572`
- `nominal_cruise / idle` 復帰: frame `610`

### 4.4 Summary-Level Acceptance

期待値:

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

verification:

- `PASS`
- MCAP でも `lead_vehicle_id = 286` に収束し、oncoming actor `287` を target と取り違えていない
- frame `204` では
  - same-lane obstacle `286`
  - left-lane oncoming `287`
  を同時に見て、reject は `adjacent_front_gap_insufficient`
- frame `230` では oncoming が rear 側へ回り、reject は `adjacent_rear_gap_insufficient`
- frame `266` で reject が消え、`lane_change_out`
- その後 `pass_vehicle -> lane_change_back -> nominal_cruise / idle`

## 5. How To Use This Sheet

新しい停止障害物変更を入れたときは、まず regression を回す。

```bash
./tests/integration/stopped_obstacle/run_stopped_obstacle_regressions.sh
```

その後、各 scenario について次だけを見る。

1. `lead_vehicle_id` が期待した actor に収束しているか
2. `overtake_reject_reason` が scenario の意味に合っているか
3. `overtake_state` の遷移順が想定どおりか
4. `collision_count` が `0` か

この 4 点が揃っていれば、停止障害物回避の baseline は維持できているとみなしてよい。
