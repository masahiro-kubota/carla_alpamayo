# Stopped Obstacle Integration Suite

停止障害物回避の `CARLA` 結合シナリオテスト資産です。

この `README` は index だけを持ちます。各 scenario の期待値は個別 markdown を見ます。

## Suite Assets

- run-configs:
  - [run_configs](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs)
- regression runner:
  - [run_stopped_obstacle_regressions.sh](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_stopped_obstacle_regressions.sh)
- scenario inspector:
  - [inspect_scenarios.py](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/inspect_scenarios.py)
- next steps:
  - [NEXT_STEPS.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/NEXT_STEPS.md)
- refactor plan:
  - [REFACTOR_PLAN.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/REFACTOR_PLAN.md)

## Verified Scenarios

- [clear.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/clear.md)
- [blocked_static.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/blocked_static.md)
- [blocked_oncoming.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/blocked_oncoming.md)
- [double_stopped_separated.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/double_stopped_separated.md)
- [double_stopped_clustered.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/double_stopped_clustered.md)
- [signal_suppressed.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/signal_suppressed.md)
- [near_junction_preflight_reject.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/near_junction_preflight_reject.md)
- [adjacent_lane_closed.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/adjacent_lane_closed.md)
- [curve_clear.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/curve_clear.md)
- [rejoin_blocked_then_release.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/rejoin_blocked_then_release.md)

この 10 本が現在の verified integration set です。

## Deferred / Exploratory Scenarios

- [right_first_clear.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/right_first_clear.md)
- [temporary_target_occlusion.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/temporary_target_occlusion.md)

## Related Design Docs

- [STOPPED_OBSTACLE_LOGIC_DESIGN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_LOGIC_DESIGN.md)
- [STOPPED_OBSTACLE_TEST_DESIGN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_TEST_DESIGN.md)
- [STOPPED_OBSTACLE_SCENARIO_CONTRACT_DESIGN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_SCENARIO_CONTRACT_DESIGN.md)
- [STOPPED_OBSTACLE_SCENARIO_CONTRACT_TEST_DESIGN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_SCENARIO_CONTRACT_TEST_DESIGN.md)

## Regression Set

- [town01_stopped_obstacle_clear_long_expert.json](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_clear_long_expert.json)
- [town01_stopped_obstacle_blocked_long_expert.json](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_blocked_long_expert.json)
- [town01_stopped_obstacle_blocked_oncoming_long_expert.json](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_blocked_oncoming_long_expert.json)
- [town01_stopped_obstacle_double_stopped_separated_long_expert.json](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_double_stopped_separated_long_expert.json)
- [town01_stopped_obstacle_double_stopped_clustered_long_expert.json](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_double_stopped_clustered_long_expert.json)
- [town01_stopped_obstacle_signal_suppressed_long_expert.json](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_signal_suppressed_long_expert.json)
- [town01_stopped_obstacle_near_junction_preflight_reject_long_expert.json](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_near_junction_preflight_reject_long_expert.json)
- [town01_stopped_obstacle_adjacent_lane_closed_long_expert.json](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_adjacent_lane_closed_long_expert.json)
- [town01_stopped_obstacle_curve_clear_long_expert.json](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_curve_clear_long_expert.json)
- [town01_stopped_obstacle_rejoin_blocked_then_release_long_expert.json](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_rejoin_blocked_then_release_long_expert.json)

## Usage

```bash
./tests/integration/ad_stack/stopped_obstacle/run_stopped_obstacle_regressions.sh
```

```bash
python tests/integration/ad_stack/stopped_obstacle/inspect_scenarios.py \
  tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_clear_long_expert.json
```
