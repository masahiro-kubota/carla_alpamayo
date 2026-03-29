# Overtake Architecture Current Issues

この文書は、`stopped obstacle` 向け refactor を進めた後の「いま本当に残っている architectural concern だけ」を整理したものです。

関連:

- [OVERTAKE_ARCHITECTURE_REFACTOR_PLAN.md](/home/masa/carla_alpamayo/docs/OVERTAKE_ARCHITECTURE_REFACTOR_PLAN.md)
- [README.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/README.md)

## 1. 今回までで解消したもの

次は、もう current issue ではありません。

- target accept / reject が stopped-specific に application service へ埋め込まれていた問題
  - いまは [target_acceptance_policy.py](/home/masa/carla_alpamayo/ad_stack/overtake/policies/target_acceptance_policy.py) と [stopped_target_acceptance_policy.py](/home/masa/carla_alpamayo/ad_stack/overtake/policies/stopped_target_acceptance_policy.py) に切り出している
  - [decision_service.py](/home/masa/carla_alpamayo/ad_stack/overtake/application/decision_service.py) は generic lane-gap / signal / target-lane decision に戻っている
- [OvertakeContext](/home/masa/carla_alpamayo/ad_stack/overtake/domain/models.py) が `stopped_speed_threshold_mps` を持っていた問題
  - threshold は candidate extraction 側へ閉じた
- `EpisodeRecord` が overtake field を明示列挙して mapper と hand-sync していた問題
  - いまは [EpisodeRecord](/home/masa/carla_alpamayo/libs/schemas/episode_schema.py) が generic base fields と `extra_fields` を持つ
  - overtake manifest projection は [telemetry_mapper.py](/home/masa/carla_alpamayo/ad_stack/overtake/infrastructure/carla/telemetry_mapper.py) に集約した
- stopped-obstacle suite の acceptance が `assertions.py` に厚くベタ書きされていた問題
  - いまは [scenario_matrix.py](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/scenario_matrix.py) が suite の正本で、[run_suite.py](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_suite.py) と [assertions.py](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/assertions.py) がこれを共有する
- MCAP JSON schema と telemetry projection の宣言点が分散していた問題
  - いまは [telemetry_contract.py](/home/masa/carla_alpamayo/ad_stack/overtake/domain/telemetry_contract.py) が `planning_debug` field spec の正本
  - [telemetry_mapper.py](/home/masa/carla_alpamayo/ad_stack/overtake/infrastructure/carla/telemetry_mapper.py) の manifest projection / MCAP payload と [mcap_route_log.py](/home/masa/carla_alpamayo/libs/schemas/mcap_route_log.py) の JSON schema が同じ spec を使う
- `follow target` / `overtake target` contract が stopped boolean に寄りすぎていた問題
  - いまは [models.py](/home/masa/carla_alpamayo/ad_stack/overtake/domain/models.py) が `motion_profile` を first-class に持つ
  - stopped policy は `motion_profile == "stopped"` を使うだけになり、moving target を入れる余地を残している
- stopped-obstacle suite の verified markdown が suite matrix と手同期だった問題
  - いまは [doc_builder.py](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/doc_builder.py) が [scenario_matrix.py](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/scenario_matrix.py) から `README` と scenario docs を生成する

## 2. いま残っている問題

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

## 3. 優先順位

次に architectural work を進めるなら次です。

1. moving target policy の domain / application contract を書く
2. moving integration suite の scenario matrix と generated docs を作る
3. stopped-obstacle suite と moving suite の共通 acceptance をさらに `_shared` に寄せる

## 4. いま言えること

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
