# Temporary Target Occlusion Scenario

追い越し対象 actor が一時的に tracking から消えても、即 `passed` 扱いしないことを runtime で確認する scenario です。

## Status

- deferred

## Planned Run Config

- `tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_temporary_target_occlusion_long_expert.json`

## Scenario Contract

- same-lane 停止障害物が一時的に tracked object から消える条件を作る
- actor 自体は存在し続ける

## Expectations

### Target Actor

- `target_actor_last_seen_s` が保持される
- actor 欠落だけで target が切り替わらない

### Reject / Wait

- `pass_vehicle` 中に target 見失いが起きても即 `lane_change_back` しない

### Pass / Rejoin

- visibility timeout 前は `target_passed = false`
- reacquisition か、actor が本当に後方へ回った時点でのみ pass 完了

### Summary Acceptance

- `collision_count = 0`
- `overtake_success_count >= 1`

## Why Deferred

- deterministic に欠落条件を作るのが難しい
- unit test のほうが先に主戦場になる
