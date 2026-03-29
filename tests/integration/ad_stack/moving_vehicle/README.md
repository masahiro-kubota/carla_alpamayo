# Moving Vehicle Integration Suite

前走車両の追い越しを対象にした `CARLA` 結合シナリオテスト資産の置き場です。

現時点では pure policy / unit test の準備までで、integration scenario は未実装です。

この directory の役割:

- future run-config の正本
- scenario expectation markdown
- regression runner

想定構成:

- `run_configs/`
- scenario ごとの expectation markdown
- `run_moving_vehicle_regressions.sh`

関連:

- [OVERTAKE_ARCHITECTURE_REFACTOR_PLAN.md](/home/masa/carla_alpamayo/docs/OVERTAKE_ARCHITECTURE_REFACTOR_PLAN.md)
- [README.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/README.md)
