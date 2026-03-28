# Near Junction Preflight Reject Scenario

停止障害物はあるが junction / signal が近すぎるため、scenario validation の時点で baseline 対象外にする scenario です。

## Status

- implemented_not_verified

## Planned Run Config

- `tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_near_junction_preflight_reject_long_expert.json`

## Scenario Contract

- same-lane に停止障害物が 1 台
- 近傍に junction または signal があり、overtake corridor として unsafe

## Expectations

### Preflight

- `scenario_validation.valid = false`
- reason に `junction_nearby` または `signal_nearby`

### Runtime

- baseline regression runner には含めない
- 実行する場合でも scenario が invalid と分かる

### Summary Acceptance

- `summary.scenario_validation` が充実している

## Why This Matters

- 「走らせない」ことも integration suite の契約に入れる
