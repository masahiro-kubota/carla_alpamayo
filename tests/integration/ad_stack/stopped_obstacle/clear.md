# Clear Scenario

停止障害物だけが前方にいて、隣接 lane が空いている場合の baseline です。

## Run Config

- [town01_stopped_obstacle_clear_long_expert.json](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_clear_long_expert.json)

## Verified Artifacts

- [summary.json](/home/masa/carla_alpamayo/outputs/evaluate/20260329_042204_town01_northbound_overtake_long_expert_eval_cb0573c8fc8e/summary.json)
- [manifest.jsonl](/home/masa/carla_alpamayo/outputs/evaluate/20260329_042204_town01_northbound_overtake_long_expert_eval_cb0573c8fc8e/manifest.jsonl)
- [segment_0000.mcap](/home/masa/carla_alpamayo/outputs/evaluate/20260329_042204_town01_northbound_overtake_long_expert_eval_cb0573c8fc8e/telemetry/segment_0000.mcap)

## Expectations

### Target Actor

- `lead_vehicle_id` は停止障害物 actor に収束する
- run 中に別 actor へふらつかない
- `current_lane_id` は開始時 `15:-1`

baseline 実績:

- `lead_ids = [275]`

### Reject / Wait

- 障害物が遠い間は `overtake_reject_reason = lead_out_of_range`
- trigger 距離に入ったら reject は消える
- `lane_change_out` に遷移する
- `adjacent_front_gap_insufficient` は出ない

baseline 実績:

- frame `144`: `lead_out_of_range`

### Pass / Rejoin

- `lane_change_out -> pass_vehicle -> lane_change_back -> nominal_cruise`
- target を抜いたあと元 lane に戻る
- `route_completion_ratio = 1.0`

baseline 実績:

- `lane_change_out` 開始: frame `204`
- `pass_vehicle` 開始: frame `346`
- `lane_change_back` 開始: frame `484`
- `nominal_cruise / idle` 復帰: frame `524`

### Summary Acceptance

- `success = true`
- `collision_count = 0`
- `overtake_attempt_count >= 1`
- `overtake_success_count >= 1`

baseline 実績:

- `success = true`
- `collision_count = 0`
- `overtake_attempt_count = 1`
- `overtake_success_count = 1`

## Verification Verdict

- `PASS`
- MCAP 上でも `lead_vehicle_id = 275` に収束
- state 遷移は `lane_change_out -> pass_vehicle -> lane_change_back -> idle`
