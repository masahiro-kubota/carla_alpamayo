# Overtake Architecture Current Issues

この文書は、`stopped obstacle` 向け refactor を進めた後の「いま本当に残っている architectural concern だけ」を整理したものです。解消済みの項目はここには残さず、未解決の論点だけを置きます。

関連:

- [PLANNING_CONTROLLER_ARCHITECTURE.md](/home/masa/carla_alpamayo/docs/PLANNING_CONTROLLER_ARCHITECTURE.md)
- [PLANNING_CONTROLLER_TARGET_ARCHITECTURE.md](/home/masa/carla_alpamayo/docs/PLANNING_CONTROLLER_TARGET_ARCHITECTURE.md)
- [PLANNING_CONTROLLER_REFACTOR_PLAN.md](/home/masa/carla_alpamayo/docs/PLANNING_CONTROLLER_REFACTOR_PLAN.md)
- [OVERTAKE_ARCHITECTURE_REFACTOR_PLAN.md](/home/masa/carla_alpamayo/docs/OVERTAKE_ARCHITECTURE_REFACTOR_PLAN.md)
- [README.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/README.md)

## 1. いま残っている問題

現時点で、`stopped obstacle` の refactor を止める architectural blocker はありません。

いま残っているのは、次の `moving lead vehicle overtake` を入れる段階で改めて設計すべき論点です。

- moving target 専用 policy の導入
  - 現状は `motion_profile` と generic target contract まで整っている
  - ただし
    - slow lead をいつ overtake candidate にするか
    - pass 中に target が加速した時どう abort するか
    - moving target の rejoin clear distance をどう定義するか
    はまだ未設計
- moving suite 用 scenario matrix の追加
  - stopped-obstacle suite の harness / docs / assertions は再利用できる
  - ただし `moving_vehicle/` 用の scenario contract と expectation matrix はまだ存在しない

## 2. 優先順位

次に architectural work を進めるなら次です。

1. moving target policy の domain / application contract を書く
2. moving integration suite の scenario matrix と generated docs を作る
3. stopped-obstacle suite と moving suite の共通 acceptance をさらに `_shared` に寄せる

## 3. いま言えること

停止障害物回避については、

- production / tests の境界
- target acceptance seam
- generic parser と feature contract の境界
- policy seam
- suite matrix
- application entry
- telemetry の `base + extra_fields` 分離
- planning debug の declarative contract
- generated suite docs
- shared integration harness

はかなり改善しています。

なので次にやるべきことは、

- stopped-obstacle をもう一段いじること

ではなく、

- `moving lead vehicle` を入れても崩れない target policy と integration suite を追加すること

です。
