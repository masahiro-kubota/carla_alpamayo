# Blocked Static Near Scenario

same-lane の停止障害物が近距離にあり、反対車線の static blocker も効いているため、安全に減速停止することを確認する scenario です。

## Status

- deferred

## Planned Run Config

- `/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_blocked_near_long_expert.json`

## Scenario Contract

- same-lane に停止障害物が 1 台あり、初期前方距離はおおむね `30-40m`
- adjacent lane front gap を static blocker が required clear distance 内で塞ぐ
- false positive ではなく、自然な blocked case として停止判断へ入る

## Expectations

### Target Actor

- `follow_target_id` は same-lane の停止障害物 actor に収束する
- adjacent lane の blocker actor を追い越し target と取り違えない

### Reject / Wait

- `adjacent_front_gap_insufficient` または `adjacent_lane_closed` で抑制される
- `lane_change_out` に入らない

### Pass / Rejoin

- 停止車両の手前で安全に減速停止する
- route-loop は `stalled` で終わり、collision は起こさない

## Why This Matters

- 今の `blocked_static` は far placement baseline なので、より自然な近距離 blocked case を別に持つ価値があります。

### Summary Acceptance

- `collision_count = 0`
- `overtake_success_count >= 1`

## Why Deferred

- 今の `blocked_static` は far placement baseline なので、より自然な近距離 blocked case を別に持つ価値があります。
