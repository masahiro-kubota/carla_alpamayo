# Curve Clear Scenario

緩いカーブ上の停止障害物を clear 条件で追い越す scenario です。

## Status

- planned

## Planned Run Config

- `tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_curve_clear_long_expert.json`

## Scenario Contract

- カーブ区間上に same-lane 停止障害物が 1 台
- adjacent lane は clear
- junction / signal は十分遠い

## Expectations

### Target Actor

- `lead_vehicle_id` は停止障害物 actor に収束する

### Reject / Wait

- trigger 条件を満たしたら `lane_change_out`

### Pass / Rejoin

- `lane_change_out -> pass_vehicle -> lane_change_back`
- route progress は単調増加
- opposite lane native direction に引っ張られない

### Summary Acceptance

- `success = true`
- `collision_count = 0`
- `overtake_success_count >= 1`

## Why This Matters

- route-aligned lane-change plan の単調性を実 world で確認する
