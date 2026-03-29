# Double Stopped Separated Scenario

停止障害物が離れて 2 台あり、1 台目を抜いて rejoin した後に 2 台目を再取得する scenario です。

## Status

- verified

## Run Config

- [town01_stopped_obstacle_double_stopped_separated_long_expert.json](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_double_stopped_separated_long_expert.json)

## Scenario Contract

- same-lane に停止障害物が 2 台
- 相互 gap は `cluster_merge_gap_m` より十分大きい

## Expectations

### Target Actor

- 1 台目を追い越し target として取得し、その後 2 台目へ切り替わる

### Reject / Wait

- 2 台目のために cluster 扱いへ崩れない

### Pass / Rejoin

- 1 台目で `lane_change_out -> pass_vehicle -> lane_change_back`
- その後 2 台目に対して再度同じ flow が発生する

## Why This Matters

- multi-obstacle を single-target の積み重ねとして扱えることを確認します。

### Summary Acceptance

- `success = true`
- `collision_count = 0`
- `overtake_attempt_count >= 2`
- `overtake_success_count >= 2`

### Manifest Acceptance

- `overtake_target_actor_id` has at least 2 unique non-null values

## Source Of Truth

- scenario matrix: [scenario_matrix.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/scenario_matrix.py)
- summary / manifest assertions: [assertions.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/assertions.py)
