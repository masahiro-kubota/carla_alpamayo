# Overtake Architecture Current Issues

この文書は、`stopped obstacle` 向け refactor を進めた後の「いま残っている architectural concern だけ」を整理したものです。

関連:

- [OVERTAKE_ARCHITECTURE_REFACTOR_PLAN.md](/home/masa/carla_alpamayo/docs/OVERTAKE_ARCHITECTURE_REFACTOR_PLAN.md)
- [README.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/README.md)

## 1. 直近で解消したもの

次は、もう current issue ではありません。

- target accept / reject が stopped-specific に application service へ埋め込まれていた問題
  - いまは [target_acceptance_policy.py](/home/masa/carla_alpamayo/ad_stack/overtake/policies/target_acceptance_policy.py) と [stopped_target_acceptance_policy.py](/home/masa/carla_alpamayo/ad_stack/overtake/policies/stopped_target_acceptance_policy.py) に切り出している
  - [decision_service.py](/home/masa/carla_alpamayo/ad_stack/overtake/application/decision_service.py) は generic lane-gap / signal / target-lane decision に戻っている
- [OvertakeContext](/home/masa/carla_alpamayo/ad_stack/overtake/domain/models.py) が `stopped_speed_threshold_mps` を持っていた問題
  - threshold は candidate extraction 側へ閉じた
- `EpisodeRecord` が overtake field を明示列挙して mapper と hand-sync していた問題
  - いまは [EpisodeRecord](/home/masa/carla_alpamayo/libs/schemas/episode_schema.py) が generic base fields と `extra_fields` を持つ
  - overtake manifest projection は [build_episode_record_extra_fields](/home/masa/carla_alpamayo/ad_stack/overtake/infrastructure/carla/telemetry_mapper.py) に集約した
- stopped-obstacle suite の acceptance が `assertions.py` に厚くベタ書きされていた問題
  - いまは [scenario_matrix.py](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/scenario_matrix.py) が suite の正本で、[run_suite.py](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_suite.py) と [assertions.py](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/assertions.py) がこれを共有する

## 2. いま残っている問題

### 2.1 MCAP JSON schema と telemetry projection の同期はまだ完全自動ではない

[mcap_route_log.py](/home/masa/carla_alpamayo/libs/schemas/mcap_route_log.py) の JSON schema は、依然として hand-maintained です。

manifest projection の重複は減りましたが、moving overtake を入れるなら将来的には

- planning debug field spec
- MCAP JSON schema
- `/ego/planning_debug` payload

をもう一段 declarative に寄せたいです。

### 2.2 `follow target` と `overtake target` の contract は moving overtake をまだ十分に表現しない

[models.py](/home/masa/carla_alpamayo/ad_stack/overtake/domain/models.py) の

- [OvertakeLeadSnapshot](/home/masa/carla_alpamayo/ad_stack/overtake/domain/models.py)
- [OvertakeTargetSnapshot](/home/masa/carla_alpamayo/ad_stack/overtake/domain/models.py)
- [OvertakeTargetTelemetry](/home/masa/carla_alpamayo/ad_stack/overtake/domain/models.py)

は stopped obstacle をうまく表現できる一方で、

- moving target の pass / abort 条件
- moving target の速度変化
- target が slow lead から non-overtakeable lead へ戻るケース

をまだ first-class に持っていません。

今の seam は moving 実装の入口としては十分ですが、moving suite を本格実装する段階で contract をもう一度見直す必要があります。

### 2.3 integration suite の verified markdown はまだ手で同期している

[scenario_matrix.py](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/scenario_matrix.py) に run-config と acceptance は寄りましたが、

- `verified/*.md`
- `inspect_only/*.md`
- `exploratory/*.md`

はまだ手更新です。

これは current blocker ではありませんが、suite を増やすなら

- scenario matrix
- generated suite index
- verified note のテンプレート

の境界をもう少し整理したいです。

## 3. 優先順位

moving overtake 前に優先して直すなら次です。

1. moving target contract を domain / telemetry に足す
2. MCAP schema と planning-debug projection の宣言点を減らす
3. verified scenario docs を matrix ともう少し近づける

## 4. いま言えること

停止障害物回避については、

- production / tests の境界
- target acceptance seam
- generic parser と feature contract の境界
- policy seam
- suite matrix
- application entry
- telemetry の `base + extra_fields` 分離
- shared integration harness

はかなり改善しています。

なので次にやるべきことは、

- stopped-obstacle をもう一段いじること

ではなく、

- `moving lead vehicle` を入れても崩れない target contract と telemetry contract を仕上げること

です。
