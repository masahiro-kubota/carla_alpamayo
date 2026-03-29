# Right-First Clear Scenario

左右どちらも成立する状況で、`preferred_direction = right_first` を runtime で確認する scenario です。

## Status

- deferred

## Planned Run Config

- `/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_right_first_clear_long_expert.json`

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

## Why This Matters

- Town01 で自然な両側成立 corridor が見つかったら昇格する候補です。

### Summary Acceptance

- `collision_count = 0`
- `overtake_success_count >= 1`

## Why Deferred

- Town01 で自然な両側成立 corridor が見つかったら昇格する候補です。
