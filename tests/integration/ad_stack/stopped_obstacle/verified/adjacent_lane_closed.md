# Adjacent Lane Closed Scenario

停止障害物は前方にあるが、隣接 lane 自体が使えないため追い越しを開始しない scenario です。

## Status

- verified

## Run Config

- [town01_stopped_obstacle_adjacent_lane_closed_long_expert.json](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_adjacent_lane_closed_long_expert.json)

## Verified Artifacts

- [summary.json](/home/masa/carla_alpamayo/outputs/evaluate/20260329_155831_town01_adjacent_lane_closed_corridor_expert_eval_fa9deeaa0a73/summary.json)
- [manifest.jsonl](/home/masa/carla_alpamayo/outputs/evaluate/20260329_155831_town01_adjacent_lane_closed_corridor_expert_eval_fa9deeaa0a73/manifest.jsonl)

## Scenario Contract

- same-lane に停止障害物が 1 台
- adjacent lane が `Driving` でない、または route 上で継続利用できない
- gap が空いていても lane 自体は closed

## Expectations

### Target Actor

- `follow_target_id` は停止障害物 actor に収束する

### Reject / Wait

- `overtake_reject_reason = adjacent_lane_closed`
- `lane_change_out` に入らない

### Pass / Rejoin

- `pass_vehicle` に入らない
- `lane_change_back` に入らない

### Summary Acceptance

- `collision_count = 0`
- `overtake_attempt_count = 0`
- `unsafe_lane_change_reject_count >= 1`

## Verification Verdict

- `PASS`
- `overtake_attempt_count = 0`
- `unsafe_lane_change_reject_count = 417`
- manifest 上でも `overtake_reject_reason = adjacent_lane_closed`

## Why This Matters

- gap 不足と lane invalid を分離して扱えているかを見る
- preflight validation と runtime reject の整合を確認する
