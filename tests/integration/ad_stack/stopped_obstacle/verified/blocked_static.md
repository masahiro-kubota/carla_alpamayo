# Blocked Static Scenario

前方停止障害物はあるが、隣接 lane が静止 blocker で塞がれていて出られない baseline です。

## Run Config

- [town01_stopped_obstacle_blocked_long_expert.json](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_blocked_long_expert.json)

## Verified Artifacts

- [summary.json](/home/masa/carla_alpamayo/outputs/evaluate/20260329_042220_town01_northbound_overtake_long_expert_eval_cb0573c8fc8e/summary.json)
- [manifest.jsonl](/home/masa/carla_alpamayo/outputs/evaluate/20260329_042220_town01_northbound_overtake_long_expert_eval_cb0573c8fc8e/manifest.jsonl)
- [segment_0000.mcap](/home/masa/carla_alpamayo/outputs/evaluate/20260329_042220_town01_northbound_overtake_long_expert_eval_cb0573c8fc8e/telemetry/segment_0000.mcap)

## Expectations

### Target Actor

- `follow_target_id` は停止障害物 actor に収束する
- opposite lane の static blocker を overtake target にしない

baseline 実績:

- `lead_ids = [280]`

### Reject / Wait

- 近づくまでは `lead_out_of_range`
- trigger 距離に入っても `adjacent_front_gap_insufficient`
- 末尾側では `adjacent_rear_gap_insufficient` へ変化しうる
- `lane_change_out` に入らない
- `planner_state = car_follow`

baseline 実績:

- frame `144`: `lead_out_of_range`
- frame `204`: `adjacent_front_gap_insufficient`
- frame `774`: `adjacent_rear_gap_insufficient`
- frame `204`: `car_follow / idle`

### Pass / Rejoin

- `pass_vehicle` に入らない
- `lane_change_back` に入らない
- route 完了ではなく `stalled`

baseline 実績:

- `overtake_state` は最後まで `idle`
- `failure_reason = stalled`

### Summary Acceptance

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

## Verification Verdict

- `PASS`
- MCAP 上でも `follow_target_id = 280` を停止障害物として見ている
- frame `204` で same-lane obstacle `280` と left-lane blocker `281` を同時に見ている
- 最後まで `lane_change_out` に入らない
