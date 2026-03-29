# Near Junction Preflight Reject Scenario

停止障害物はあるが junction / signal が近すぎるため、scenario validation の時点で baseline 対象外にする scenario です。

## Status

- verified

## Run Config

- [town01_stopped_obstacle_near_junction_preflight_reject_long_expert.json](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_near_junction_preflight_reject_long_expert.json)

## Verified Artifacts

- [summary.json](/home/masa/carla_alpamayo/outputs/evaluate/20260329_074401_town01_signal_conflict_short_expert_eval_133397cc9280/summary.json)

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

## Verification Verdict

- `PASS`
- `failure_reason = scenario_preflight_invalid`
- `scenario_validation.valid = false`
- `scenario_validation.errors = [\"junction_nearby\"]`

## Why This Matters

- 「走らせない」ことも integration suite の契約に入れる
