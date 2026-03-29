# Double Stopped Clustered Scenario

停止障害物が近接し、1 つの `cluster` として同時に追い越す scenario です。

## Status

- verified

## Run Config

- [town01_stopped_obstacle_double_stopped_clustered_long_expert.json](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_double_stopped_clustered_long_expert.json)

## Scenario Contract

- same-lane に停止障害物が 2 台以上
- 相互 gap は `cluster_merge_gap_m` 以下
- adjacent lane は clear

## Expectations

### Target Actor

- `overtake_target_kind = cluster` を維持する
- `overtake_target_member_actor_ids` は複数 actor を保持する

### Reject / Wait

- cluster 途中で別 target へ切り替えず corridor を維持する

### Pass / Rejoin

- cluster 全体に対して 1 回の `lane_change_out -> pass_vehicle -> lane_change_back` で終える

## Why This Matters

- 複数停止車両の同時追い越しを state machine 肥大化なしで扱えるかを見る scenario です。

### Summary Acceptance

- `success = true`
- `collision_count = 0`
- `overtake_attempt_count >= 1`
- `overtake_success_count >= 1`

### Manifest Acceptance

- some manifest row has `overtake_target_kind = cluster`
- some manifest row has `overtake_target_member_actor_ids` length >= 2

## Source Of Truth

- scenario matrix: [scenario_matrix.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/scenario_matrix.py)
- summary / manifest assertions: [assertions.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/assertions.py)
