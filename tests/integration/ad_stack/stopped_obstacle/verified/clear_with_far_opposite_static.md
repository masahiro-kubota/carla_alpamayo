# Clear With Far Opposite Static Scenario

反対車線に静止車両はいるが、pass corridor の required clear distance 外なので通常どおり追い越してよい scenario です。

## Status

- verified

## Run Config

- [town01_stopped_obstacle_clear_with_far_opposite_static_long_expert.json](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_clear_with_far_opposite_static_long_expert.json)

## Scenario Contract

- same-lane に停止障害物が 1 台
- 反対車線に static actor が 1 台いる
- その actor は pass corridor の required clear distance 外にある

## Expectations

### Target Actor

- `follow_target_id` は same-lane の停止障害物 actor に収束する
- far opposite static actor を blocker target と取り違えない

### Reject / Wait

- `adjacent_front_gap_insufficient` を誤検出しない
- opposite static actor がいても `lane_change_out` を抑制しない

### Pass / Rejoin

- `lane_change_out -> pass_vehicle -> lane_change_back` が通常どおり成立する

## Why This Matters

- Town01 の片側1車線で false positive reject を抑える代表ケースです。

### Summary Acceptance

- `success = true`
- `collision_count = 0`
- `average_speed_kmh >= 18.0`
- `overtake_attempt_count >= 1`
- `overtake_success_count >= 1`

### Manifest Acceptance

- no manifest row has `overtake_reject_reason = adjacent_front_gap_insufficient`
- some manifest row with `overtake_state = lane_change_out` has `target_speed_kmh >= 30.0`

## Source Of Truth

- scenario matrix: [scenario_matrix.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/scenario_matrix.py)
- summary / manifest assertions: [assertions.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/assertions.py)
