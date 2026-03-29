# Double Stopped Clustered With Oncoming Block Scenario

近接した停止障害物 cluster があり、対向車がいる間は出ず、corridor 解放後に cluster として 1 回で抜く scenario です。

## Status

- verified

## Run Config

- [town01_stopped_obstacle_double_stopped_clustered_with_oncoming_block_long_expert.json](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_double_stopped_clustered_with_oncoming_block_long_expert.json)

## Scenario Contract

- same-lane に cluster として扱う停止障害物が 2 台以上ある
- 反対車線に oncoming actor が一時的に存在する
- corridor 解放後は cluster pass が可能になる

## Expectations

### Target Actor

- `overtake_target_kind = cluster` を維持する
- `overtake_target_member_actor_ids` は複数 actor を保持する

### Reject / Wait

- oncoming actor が近い間は `lane_change_out` に入らない
- corridor 解放後に初めて overtake を開始する

### Pass / Rejoin

- cluster に対して 1 回の `lane_change_out -> pass_vehicle -> lane_change_back` を行う

## Why This Matters

- multi-target と opposite-lane occupancy の組み合わせで state machine が崩れないことを確認するためです。

### Summary Acceptance

- `success = true`
- `collision_count = 0`
- `overtake_attempt_count >= 1`
- `overtake_success_count >= 1`
- `unsafe_lane_change_reject_count >= 1`

### Manifest Acceptance

- some manifest row has `overtake_target_kind = cluster`
- some manifest row has `overtake_target_member_actor_ids` length >= 2

## Source Of Truth

- scenario matrix: [scenario_matrix.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/scenario_matrix.py)
- summary / manifest assertions: [assertions.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/assertions.py)
