# Double Stopped Clustered Scenario

same-lane に停止障害物が 2 台以上あるが、相互 gap が小さく、1 つの `obstacle cluster` として同時に追い越す scenario です。

## Status

- planned

## Planned Run Config

- `tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_double_stopped_clustered_long_expert.json`

## Scenario Contract

- same-lane に停止障害物が 2 台以上
- 相互 gap は `cluster_merge_gap_m` 以下
- adjacent lane は clear
- cluster 全体を抜き切るまで target lane を維持する余地がある

## Expectations

### Target Actor

- runtime は `single_actor` ではなく `overtake_target_kind = cluster` を持つ
- `overtake_target_member_actor_ids` は cluster member を安定に保持する
- member の一時的な見え隠れで corridor 解釈が揺れない

### Reject / Wait

- cluster 先頭に対して追い越し開始が成立する
- cluster 中途で 2 台目のために新しい reject / wait へ戻らない

### Pass / Rejoin

- `lane_change_out -> pass_vehicle -> lane_change_back` を cluster 全体に対して 1 回で行う
- cluster tail を抜く前に不用意に rejoin しない
- actor 個別の切替は必須ではなく、cluster tail 基準で pass 完了する

### Summary Acceptance

- `collision_count = 0`
- `overtake_attempt_count >= 1`
- `overtake_target_kind = cluster`
- `overtake_target_member_actor_ids` が 2 台以上
- `first_rejoin_started_s` は cluster tail 通過後で説明可能

## Why This Matters

- 複数停止車両の同時追い越しを、state machine を肥大化させずに実現する本命 scenario
- `single actor` と `obstacle cluster` の境界を検証する
