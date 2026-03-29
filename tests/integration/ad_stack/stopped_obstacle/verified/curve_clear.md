# Curve Clear Scenario

緩いカーブ上の停止障害物を clear 条件で追い越す scenario です。

## Status

- verified

## Run Config

- [town01_stopped_obstacle_curve_clear_long_expert.json](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_curve_clear_long_expert.json)

## Scenario Contract

- route 進行先の curve 上に停止障害物がある
- adjacent lane は clear

## Expectations

### Target Actor

- same-lane でまだ見えていなくても route-aligned target として観測できる

### Reject / Wait

- curve 進入前後で target acquisition が崩れない

### Pass / Rejoin

- カーブでも `lane_change_out -> pass_vehicle -> lane_change_back` が成立する

## Why This Matters

- 直線 corridor だけに過学習していないことを確認します。

### Summary Acceptance

- `success = true`
- `collision_count = 0`
- `overtake_attempt_count >= 1`
- `overtake_success_count >= 1`

## Source Of Truth

- scenario matrix: [scenario_matrix.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/scenario_matrix.py)
- summary / manifest assertions: [assertions.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/assertions.py)
