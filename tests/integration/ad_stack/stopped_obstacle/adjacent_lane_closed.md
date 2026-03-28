# Adjacent Lane Closed Scenario

停止障害物は前方にあるが、隣接 lane 自体が使えないため追い越しを開始しない scenario です。

## Status

- planned

現状メモ:

- route 候補は [town01_adjacent_lane_closed_corridor.json](/home/masa/carla_alpamayo/scenarios/routes/town01_adjacent_lane_closed_corridor.json)
- ただし停止障害物を置く exact transform はまだ未確定
- `adjacent_lane_closed` だけは CARLA 上で corridor を再確認してから environment / run-config を追加する

## Planned Run Config

- `tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_adjacent_lane_closed_long_expert.json`

## Scenario Contract

- same-lane に停止障害物が 1 台
- adjacent lane が `Driving` でない、または route 上で継続利用できない
- gap が空いていても lane 自体は closed

## Expectations

### Target Actor

- `lead_vehicle_id` は停止障害物 actor に収束する

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

## Why This Matters

- gap 不足と lane invalid を分離して扱えているかを見る
- preflight validation と runtime reject の整合を確認する
