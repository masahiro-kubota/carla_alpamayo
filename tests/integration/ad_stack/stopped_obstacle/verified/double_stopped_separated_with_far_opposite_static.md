# Double Stopped Separated With Far Opposite Static Scenario

離れた 2 台の停止障害物を順番に抜くが、反対車線の遠方 static actor は blocker と誤判定しない scenario です。

## Status

- verified

## Run Config

- [town01_stopped_obstacle_double_stopped_separated_with_far_opposite_static_long_expert.json](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_double_stopped_separated_with_far_opposite_static_long_expert.json)

## Scenario Contract

- same-lane に separated 停止障害物が 2 台ある
- 反対車線に far static actor が 1 台いる
- far static actor は required clear distance 外にある

## Expectations

### Target Actor

- 1 台目を抜いたあと 2 台目へ `follow_target_id` が切り替わる
- far opposite static actor を active target / blocker と取り違えない

### Reject / Wait

- false positive reject なしで 2 回の overtake flow に入る

### Pass / Rejoin

- 1 台目で rejoin し、その後 2 台目に対して再度 `lane_change_out -> pass_vehicle -> lane_change_back` を行う

## Why This Matters

- separated multi-target でも irrelevant actor による誤抑制がないことを確認するためです。

### Summary Acceptance

- `success = true`
- `collision_count = 0`
- `overtake_attempt_count >= 2`
- `overtake_success_count >= 2`

### Manifest Acceptance

- `overtake_target_actor_id` has at least 2 unique non-null values
- no manifest row has `overtake_reject_reason = adjacent_front_gap_insufficient`

## Source Of Truth

- scenario matrix: [scenario_matrix.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/scenario_matrix.py)
- summary / manifest assertions: [assertions.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/assertions.py)
