# Stopped Obstacle Integration Suite

停止障害物回避の `CARLA` 結合シナリオテスト資産です。

この `README` は `scenario_matrix.py` から生成されています。suite の正本は matrix と shared assertion です。

## Suite Assets

- run-configs:
  - [run_configs](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs)
- regression runner:
  - [run_stopped_obstacle_regressions.sh](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_stopped_obstacle_regressions.sh)
- scenario inspector:
  - [inspect_scenarios.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/inspect_scenarios.py)
- scenario matrix:
  - [scenario_matrix.py](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/scenario_matrix.py)

## Verified Scenarios

- [clear.md](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/verified/clear.md)
- [blocked_static.md](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/verified/blocked_static.md)
- [blocked_oncoming.md](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/verified/blocked_oncoming.md)
- [double_stopped_separated.md](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/verified/double_stopped_separated.md)
- [double_stopped_clustered.md](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/verified/double_stopped_clustered.md)
- [signal_suppressed.md](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/verified/signal_suppressed.md)
- [adjacent_lane_closed.md](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/verified/adjacent_lane_closed.md)
- [curve_clear.md](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/verified/curve_clear.md)
- [rejoin_blocked_then_release.md](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/verified/rejoin_blocked_then_release.md)

この 9 本が現在の verified route-loop integration set です。

## Inspect-Only Scenarios

- [near_junction_preflight_reject.md](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/inspect_only/near_junction_preflight_reject.md)

この group は route-loop 成否ではなく inspector contract で検証します。

## Deferred / Exploratory Scenarios

- [right_first_clear.md](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/exploratory/right_first_clear.md)
- [temporary_target_occlusion.md](/media/masa/ssd_data/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/exploratory/temporary_target_occlusion.md)

## Related Design Docs

- [STOPPED_OBSTACLE_LOGIC_DESIGN.md](/media/masa/ssd_data/carla_alpamayo/docs/STOPPED_OBSTACLE_LOGIC_DESIGN.md)
- [STOPPED_OBSTACLE_TEST_DESIGN.md](/media/masa/ssd_data/carla_alpamayo/docs/STOPPED_OBSTACLE_TEST_DESIGN.md)
- [STOPPED_OBSTACLE_SCENARIO_CONTRACT_DESIGN.md](/media/masa/ssd_data/carla_alpamayo/docs/STOPPED_OBSTACLE_SCENARIO_CONTRACT_DESIGN.md)
- [STOPPED_OBSTACLE_SCENARIO_CONTRACT_TEST_DESIGN.md](/media/masa/ssd_data/carla_alpamayo/docs/STOPPED_OBSTACLE_SCENARIO_CONTRACT_TEST_DESIGN.md)
- [NEXT_STEPS.md](/media/masa/ssd_data/carla_alpamayo/docs/stopped_obstacle/NEXT_STEPS.md)
- [REFACTOR_PLAN.md](/media/masa/ssd_data/carla_alpamayo/docs/stopped_obstacle/REFACTOR_PLAN.md)

## Usage

```bash
./tests/integration/ad_stack/stopped_obstacle/run_stopped_obstacle_regressions.sh
```

`near_junction_preflight_reject` は route-loop summary ではなく inspector contract で検証します。

```bash
python tests/integration/ad_stack/stopped_obstacle/inspect_scenarios.py \
  tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_clear_long_expert.json
```
