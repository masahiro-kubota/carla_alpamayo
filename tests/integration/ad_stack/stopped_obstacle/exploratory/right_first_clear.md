# Right-First Clear Scenario

左右どちらも成立する状況で、`preferred_direction = right_first` を runtime で確認する scenario です。

## Status

- deferred

## Planned Run Config

- `tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_right_first_clear_long_expert.json`

## Scenario Contract

- left / right の両方が `Driving`
- front / rear gap が両側とも十分
- `preferred_direction = right_first`

## Expectations

### Target Actor

- `follow_target_id` は停止障害物 actor に収束する

### Reject / Wait

- reject は解消される
- `target_lane_id` は右 lane を選ぶ

### Pass / Rejoin

- 右側へ `lane_change_out`
- `pass_vehicle -> lane_change_back`

### Summary Acceptance

- `collision_count = 0`
- `overtake_success_count >= 1`

## Why Deferred

- Town01 で自然に両側成立する直線区間を用意する難度が高い
- pure logic の単体テスト価値のほうが先に大きい
