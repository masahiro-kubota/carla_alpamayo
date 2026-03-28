# Double Stopped Obstacle Scenario

同一路上に停止障害物が 2 台連続で存在し、target actor の切り替えと復帰タイミングを確認する scenario です。

## Status

- implemented_not_verified

## Planned Run Config

- `tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_double_stopped_long_expert.json`

## Scenario Contract

- same-lane に停止障害物が 2 台
- 1 台目と 2 台目の縦距離は、同一 overtake corridor として扱うか迷う程度に近い
- adjacent lane は clear

## Expectations

### Target Actor

- まず 1 台目を `lead_vehicle_id` として見る
- 1 台目通過後に 2 台目へ自然に切り替わる
- target actor が不用意にふらつかない

### Reject / Wait

- 追い越し開始自体は成立する

### Pass / Rejoin

- 1 台目通過直後に早戻りしない
- 2 台目も含めて corridor を処理できるならそのまま維持
- そうでなければ一度戻って再度出るが、いずれにせよ actor 切替は一貫している

### Summary Acceptance

- `collision_count = 0`
- `overtake_attempt_count >= 1`
- `overtake_target_actor_id` の切替が説明可能

## Why This Matters

- `target actor` 選択と pass 完了判定の robustness を見る
