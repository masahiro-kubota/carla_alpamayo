# Clear Scenario

停止障害物だけが前方にいて、隣接 lane が空いている場合の baseline です。

## Status

- verified

## Run Config

- [town01_stopped_obstacle_clear_long_expert.json](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_clear_long_expert.json)

## Scenario Contract

- same-lane に停止障害物が 1 台
- adjacent lane は clear
- route 完走まで十分な corridor がある

## Expectations

### Target Actor

- `follow_target_id` は停止障害物 actor に収束する
- run 中に別 actor へふらつかない

### Reject / Wait

- trigger 距離に入るまでは `target_out_of_range` で抑制される
- trigger 後は `lane_change_out` に遷移する

### Pass / Rejoin

- `lane_change_out -> pass_vehicle -> lane_change_back -> nominal_cruise`
- target を抜いたあと元 lane に戻る

## Why This Matters

- 停止障害物回避の最小 baseline です。

### Summary Acceptance

- `success = true`
- `collision_count = 0`
- `overtake_attempt_count >= 1`
- `overtake_success_count >= 1`

## Source Of Truth

- scenario matrix: [scenario_matrix.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/scenario_matrix.py)
- summary / manifest assertions: [assertions.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/assertions.py)
