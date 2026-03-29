# Near Junction Preflight Reject Scenario

停止障害物はあるが junction / signal が近すぎるため、preflight で invalid にする scenario です。

## Status

- verified

## Inspection Command

```bash
uv run python /media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/inspect_scenarios.py --allow-invalid /media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_near_junction_preflight_reject_long_expert.json
```

## Scenario Contract

- same-lane に停止障害物が 1 台
- 近傍に junction または signal があり、overtake corridor として unsafe

## Expectations

### Preflight

- `scenario_validation.valid = false`
- `junction_nearby` または `signal_nearby` を含む

## Source Of Truth

- scenario matrix: [scenario_matrix.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/scenario_matrix.py)
- inspector contract: [overtake_scenario_contract.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/_shared/overtake_scenario_contract.py)
