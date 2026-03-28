# Stopped Obstacle Integration Suite

停止障害物回避の `CARLA` 結合シナリオテスト資産です。

この `README` は index だけを持ちます。各 scenario の期待値は個別 markdown を見ます。

## Suite Assets

- run-configs:
  - [run_configs](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs)
- regression runner:
  - [run_stopped_obstacle_regressions.sh](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_stopped_obstacle_regressions.sh)

## Verified Baseline Scenarios

- [clear.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/clear.md)
- [blocked_static.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/blocked_static.md)
- [blocked_oncoming.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/blocked_oncoming.md)

この 3 本だけが現在の executable baseline です。

## Planned Integration Scenarios

- [signal_suppressed.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/signal_suppressed.md)
- [rejoin_blocked_then_release.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/rejoin_blocked_then_release.md)
- [adjacent_lane_closed.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/adjacent_lane_closed.md)
- [double_stopped_obstacle.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/double_stopped_obstacle.md)
- [curve_clear.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/curve_clear.md)
- [near_junction_preflight_reject.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/near_junction_preflight_reject.md)

## Deferred / Exploratory Scenarios

- [right_first_clear.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/right_first_clear.md)
- [temporary_target_occlusion.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/temporary_target_occlusion.md)

## Related Design Docs

- [STOPPED_OBSTACLE_LOGIC_DESIGN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_LOGIC_DESIGN.md)
- [STOPPED_OBSTACLE_TEST_DESIGN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_TEST_DESIGN.md)

## Baseline Regression Set

- [town01_stopped_obstacle_clear_long_expert.json](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_clear_long_expert.json)
- [town01_stopped_obstacle_blocked_long_expert.json](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_blocked_long_expert.json)
- [town01_stopped_obstacle_blocked_oncoming_long_expert.json](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_blocked_oncoming_long_expert.json)

## Usage

```bash
./tests/integration/ad_stack/stopped_obstacle/run_stopped_obstacle_regressions.sh
```
