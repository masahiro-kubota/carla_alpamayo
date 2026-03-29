# Overtake Architecture Current Issues

この文書は、`stopped obstacle` 向け refactor を進めた後の「まだ残っている問題だけ」を整理したものです。

関連:

- [OVERTAKE_ARCHITECTURE_REFACTOR_PLAN.md](/home/masa/carla_alpamayo/docs/OVERTAKE_ARCHITECTURE_REFACTOR_PLAN.md)
- [README.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/README.md)

## 1. もう解消したもの

次は、少なくとも「いまの current issue」ではありません。

- `simulation/environment_config.py` が feature parser を持っていた問題
  - いまは `overtake_scenario` を raw dict のまま返す
- production package が scenario contract / preflight contract を持っていた問題
  - いまは [overtake_scenario_contract.py](/home/masa/carla_alpamayo/tests/integration/ad_stack/_shared/overtake_scenario_contract.py) に寄っている
- [run.py](/home/masa/carla_alpamayo/ad_stack/run.py) が stopped-obstacle 専用 preflight と summary field を知っていた問題
  - いまは generic route-loop lifecycle に戻している
- production domain に integration scenario 名が残っていた問題
  - `ScenarioKind` は production domain から削除した
- [snapshot_builder.py](/home/masa/carla_alpamayo/ad_stack/overtake/infrastructure/carla/snapshot_builder.py) が concrete stopped-target policy を固定していた問題
  - いまは `target_policy` を注入する
- [expert_basic_agent.py](/home/masa/carla_alpamayo/ad_stack/agents/expert_basic_agent.py) に route-loop step の高水準分岐が残っていた問題
  - いまは [step_service.py](/home/masa/carla_alpamayo/ad_stack/overtake/application/step_service.py) が application 入口になっている
- candidate builder が stopped vehicle 前提だった問題
  - いまは [build_target_candidates](/home/masa/carla_alpamayo/ad_stack/overtake/infrastructure/carla/snapshot_builder.py) が generic candidate extraction を返し、policy が stopped target へ正規化する
- `planning_debug` payload が 1 枚の flat dict だった問題
  - いまは `core` と `target` に分かれ、payload も nested contract になっている
- telemetry の public naming が `lead_vehicle_*` と `overtake_target_*` で混在していた問題
  - いまは public telemetry を `follow_target_*` と `overtake_target_*` に分けている
- integration harness が suite ごとの shell orchestration に偏っていた問題
  - いまは [carla_harness.py](/home/masa/carla_alpamayo/tests/integration/ad_stack/_shared/carla_harness.py) と [run_suite.py](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_suite.py) に寄っている
- suite declaration が module 直書きだった問題
  - いまは [suite_runner.py](/home/masa/carla_alpamayo/tests/integration/ad_stack/_shared/suite_runner.py) の declarative spec を使う
- suite root に進行メモと実働資産が混在していた問題
  - 進行メモは [docs/stopped_obstacle](/home/masa/carla_alpamayo/docs/stopped_obstacle) に移した
- suite docs の verified / inspect-only / exploratory が同居していた問題
  - いまは directory を分けた
- 未参照 legacy asset が残っていた問題
  - 旧 short env と旧 double-stopped run-config は削除した
- `snapshot_builder.py` に extractor / projector / assembler が混在していた問題
  - いまは `candidate_extractor.py`, `route_projection.py`, `scene_assembler.py` に分割した
- [run.py](/home/masa/carla_alpamayo/ad_stack/run.py) の recording loop が manifest / MCAP の field 展開を重複して持っていた問題
  - いまは [RouteLoopFrameTelemetryRequest](/home/masa/carla_alpamayo/ad_stack/overtake/infrastructure/carla/telemetry_mapper.py) と `build_frame_telemetry(...)` に寄せている
- stopped target policy の選択が agent 側で固定されていた問題
  - いまは [api.py](/home/masa/carla_alpamayo/ad_stack/api.py) から `target_policy` を注入している

## 2. いま残っている問題

### 2.1 `OvertakeContext` と decision service はまだ stopped-oriented

[models.py](/home/masa/carla_alpamayo/ad_stack/overtake/domain/models.py) と [decision_service.py](/home/masa/carla_alpamayo/ad_stack/overtake/application/decision_service.py) は、今も

- `stopped_speed_threshold_mps`
- `active_target.is_stopped`
- stopped obstacle 前提の reject reason

に寄っています。

moving overtake を入れるなら、

- stopped / moving で分かれる target accept 条件
- pass / abort 条件
- follow target が moving のときの制限速度

を policy extension として外せるようにする必要があります。

### 2.2 MCAP schema と EpisodeRecord が still hand-maintained

[mcap_route_log.py](/home/masa/carla_alpamayo/libs/schemas/mcap_route_log.py) と [episode_schema.py](/home/masa/carla_alpamayo/libs/schemas/episode_schema.py) は、今も

- field 定義
- JSON schema
- telemetry mapper

の 3 箇所で人手同期しています。

今回の refactor で naming は整理されましたが、moving overtake を入れると field が増えるので、

- domain telemetry DTO
- manifest projection
- MCAP projection / schema

の生成責務をもう少し一元化したいです。

### 2.3 suite assertions は shared harness に比べると still per-suite module が厚い

[tests/integration/ad_stack/_shared](/home/masa/carla_alpamayo/tests/integration/ad_stack/_shared) には harness と declarative suite spec が入りました。

ただし stopped-obstacle suite ではまだ

- [assertions.py](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/assertions.py)
- verified scenario markdown

に scenario 固有 acceptance が厚く残っています。

これは間違いではありませんが、moving suite を足すなら

- reusable acceptance primitives
- scenario matrix から assertion へ写像する小さな DSL

を持ったほうが diff が小さくなります。

## 3. 優先順位

moving overtake 前に優先して直すなら次です。

1. stopped-oriented な decision vocabulary を policy extension 側へ押し出す
2. telemetry schema / projection の同期点を減らす
3. suite assertions を scenario matrix 駆動に近づける

## 4. いま言えること

停止障害物回避については、

- production / tests の境界
- generic parser と feature contract の境界
- policy seam
- suite root と planning memo / scenario docs の境界
- application entry
- telemetry の `core + target` 分離
- shared integration harness

はかなり改善しています。

なので次にやるべきことは、

- 「壊れているところを場当たりで直す」

ではなく、

- `moving lead vehicle` を入れても崩れない application / adapter 境界を仕上げる

ことです。
