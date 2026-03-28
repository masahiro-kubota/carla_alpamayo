# Double Stopped Separated Scenario

same-lane に停止障害物が 2 台あるが、相互 gap が十分大きく、1 台目を抜いたあと一度 rejoin する scenario です。

## Status

- planned

## Planned Run Config

- `tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_double_stopped_separated_long_expert.json`

## Scenario Contract

- same-lane に停止障害物が 2 台
- 1 台目と 2 台目の縦距離は `cluster_merge_gap_m` より大きい
- adjacent lane は clear
- 1 台目通過後に origin lane へ安全に戻れる gap がある

## Expectations

### Target Actor

- まず 1 台目を `lead_vehicle_id` / `overtake_target_actor_id` として見る
- 1 台目通過後、origin lane 復帰後に 2 台目へ切り替わる
- target actor が 1 台目と 2 台目の間で不用意にふらつかない

### Reject / Wait

- 1 台目に対する追い越し開始は成立する
- 2 台目に対する再取得後も、必要なら改めて reject / wait が出てよい

### Pass / Rejoin

- `lane_change_out -> pass_vehicle -> lane_change_back` を 1 台目に対して完了する
- origin lane へ戻ったあと、2 台目に対して新しい overtake event を開始できる
- `pass_vehicle` 中に 2 台目を同時 corridor として保持しない

### Summary Acceptance

- `collision_count = 0`
- `overtake_attempt_count >= 1`
- 1 台目通過後に `lead_vehicle_id` または `overtake_target_actor_id` が 2 台目へ切り替わる
- actor 切替の前後で state 遷移が説明可能

## Why This Matters

- `single_actor` 設計のままで複数停止車両に対応できる境界を確認する
- 早戻りと target reacquisition の contract を明確にする
