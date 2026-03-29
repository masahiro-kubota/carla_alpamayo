# Overtake Architecture Refactor Plan

停止障害物回避を起点に実装してきた `overtake` 周りを、今後の「前走車両の追い越し」まで含めて継続開発できる形へ寄せるための整理メモです。

この文書の目的:

- 現在のアーキテクチャとディレクトリ構成の問題点を明文化する
- `stopped obstacle` だけに最適化された実装から、`moving lead vehicle` まで扱える構成へ寄せる
- 今後の責務分離と正本ディレクトリを固定する
- リファクタの優先順位を決める

関連:

- [OVERTAKE_EXPERT_REQUIREMENTS.md](/home/masa/carla_alpamayo/docs/OVERTAKE_EXPERT_REQUIREMENTS.md)
- [STOPPED_OBSTACLE_LOGIC_DESIGN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_LOGIC_DESIGN.md)
- [STOPPED_OBSTACLE_TEST_DESIGN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_TEST_DESIGN.md)
- [STOPPED_OBSTACLE_SCENARIO_CONTRACT_DESIGN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_SCENARIO_CONTRACT_DESIGN.md)
- [README.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/README.md)

## 1. いまの問題

### 1.1 `expert_basic_agent.py` に責務が寄りすぎている

現在の [expert_basic_agent.py](/home/masa/carla_alpamayo/ad_stack/agents/expert_basic_agent.py) は、少なくとも次の責務を同時に持っています。

- world state の参照
- lead / blocker / lane gap の抽出
- overtake の状態機械
- lane-change path の生成
- longitudinal / lateral の挙動決定
- debug 情報の蓄積

この状態だと、

- 停止障害物用の例外処理が増える
- moving vehicle 用の条件を足すと分岐がさらに増える
- pure logic と CARLA adapter の境界が曖昧になる

結果として、ロジックを直したいのか、観測条件を直したいのか、経路生成を直したいのかが読み取りづらい。

### 1.2 `stopped obstacle` 専用の概念が feature 全体の概念になっていない

現在の pure logic は `ad_stack/overtake/domain|application|policies` に分割されたが、なお naming の一部が停止障害物に寄りすぎています。

たとえば今後の moving vehicle 追い越しでは、次の共通概念が必要です。

- active overtake target
- single actor / cluster
- trigger / reject
- pass complete
- rejoin gate
- visible corridor / oncoming risk

停止障害物だけを前提にした naming のままだと、

- `stopped obstacle` と `slow lead vehicle` で別実装が生える
- 同じ `lane_change_out -> pass_vehicle -> lane_change_back` を二重管理しやすい

### 1.3 scenario authoring / validation / integration が分かれているが、正本がまだ弱い

今は改善されてきたものの、依然として情報が複数箇所に分かれています。

- scenario asset
  - `scenarios/environments/...`
  - `scenarios/routes/...`
- run-config
  - `tests/integration/ad_stack/stopped_obstacle/run_configs/...`
- expectation
  - `tests/integration/ad_stack/stopped_obstacle/*.md`
- validation / inspector
  - [preflight_validation.py](/home/masa/carla_alpamayo/ad_stack/overtake/infrastructure/carla/preflight_validation.py)
  - [inspect_stopped_obstacle_scenario.py](/home/masa/carla_alpamayo/simulation/pipelines/inspect_stopped_obstacle_scenario.py)

この分離自体は正しいが、

- feature 固有の scenario contract
- 汎用 runtime infrastructure
- テスト資産

の境界をさらに明確にしないと、moving vehicle 用 suite を足したときにまた散りやすい。

### 1.4 telemetry / summary が feature 拡張に対して平面的

manifest / summary / MCAP にはかなり情報を入れているが、feature ごとに field を継ぎ足している状態です。

たとえば今後は、

- stopped obstacle
- slow moving lead vehicle
- multi-target overtake
- signal-suppressed overtake

を同じ枠で見たい。

そのときに必要なのは、

- target の型
- target member actor ids
- reject reason
- pass / rejoin 進捗
- visible corridor / required clear distance

のような「overtake telemetry の共通 schema」であり、停止障害物用の ad-hoc field 追加ではない。

さらに、clean architecture の観点では

- domain / application が「どんな debug DTO を出すか」
- infrastructure / interface が「それをどう MCAP / manifest / summary に写像するか」

を分けるべきです。

つまり、telemetry schema の正本は pure な contract であって、MCAP 向けの具体 serialization ではない。

### 1.5 moving vehicle 追い越しを足すと、今のままでは duplicate しやすい

今後やりたいのは「前を走る車両の追い越し」です。

ただしそこで必要なものの多くは、停止障害物回避と同じです。

- 追い越し対象の選定
- adjacent lane の gap 評価
- signal / junction / curve suppression
- pass 完了判定
- rejoin 判定

違うのは主にこれです。

- target が停止か低速か
- target の将来位置をどれだけ conservative に見るか
- required clear distance の計算

したがって、停止障害物ロジックをそのまま拡張するのではなく、

- `共通 overtake core`
- `stopped target policy`
- `moving target policy`

に分けるべきです。

### 1.6 application layer がまだ明示されていない

現行案は `core / policies / adapters` で十分近いが、clean architecture としては

- domain
- application
- infrastructure
- interface

の層を明示したほうが依存方向を固定しやすい。

特に `state machine` や `decision service` は pure ではあるが domain object そのものではなく、application/use-case 層として置いたほうが整理しやすい。

## 2. リファクタ後に目指す形

### 2.1 大原則

- pure logic は `CARLA` 型を持たない
- adapter は `SceneState` から pure logic 用 snapshot を作るだけに寄せる
- scenario validation は feature 固有 contract として保つ
- integration suite は `tests/` を正本にする
- `stopped obstacle` と `moving lead vehicle` は、同じ overtake core の上に載せる
- state 名、telemetry、decision contract は共通化する
- state machine 本体は「可能な限り共有」を原則にし、feature 差分が強い部分は policy か guard として分離する

### 2.2 将来の feature 分解

### `domain`

責務:

- active target 正規化
- pass / rejoin の純粋判定規則
- target / lane / corridor の pure model

### `application`

責務:

- overtake state machine
- reject / wait / pass / rejoin 判定の orchestrate
- route-aligned lane-change progression
- debug / decision DTO の出力
- ports を介した adapter 依存の抽象化

### `target policy`

責務:

- 何を overtake target とみなすか
- stopped / moving / cluster の分類
- target ごとの trigger 条件
- required clear distance の差分

制約:

- pure strategy として保つ
- `domain` の model / DTO だけを見る
- `application` を import しない

想定 policy:

- `stopped_target_policy`
- `moving_target_policy`

### `carla adapter`

責務:

- CARLA world から target candidates を作る
- lane gap / route corridor / signal / junction 情報を集める
- pure logic decision を waypoint / control へ落とす

### `interface layer`

責務:

- `expert_basic_agent.py` の orchestration
- `run.py` / `run_route_loop.py` の entrypoint
- MCAP / manifest / summary への具体 serialization

### `scenario suite`

責務:

- run-config
- contract
- expectation
- regression runner

注記:

- route / environment JSON 自体の正本は `scenarios/` に残す
- `scenario suite` はそれらをどう組み合わせて検証するかの正本を指す

### 2.3 依存方向ルール

ディレクトリを分けるだけでは不十分なので、依存方向を先に固定する。

- `ad_stack/overtake/domain`
  - `CARLA`, `simulation`, `tests` を import しない
- `ad_stack/overtake/application`
  - `domain` には依存してよい
  - `CARLA`, `simulation` には依存しない
- `ad_stack/overtake/policies`
  - `domain` には依存してよい
  - `application`, `CARLA`, `simulation` には依存しない
- `ad_stack/overtake/infrastructure/carla`
  - `CARLA` に触ってよい唯一の層
  - `domain / application / policies` への変換責務を持つ
- `ad_stack/agents/expert_basic_agent.py`
  - overtake の判断本体を持たない
  - infrastructure adapter と control execution の orchestration に寄せる

追加ルール:

- integration scenario contract validation は production package に入れない
- `tests/integration/...` と inspector tooling で持つ
- production package に残す validation は runtime judgment に必要な pure rule だけにする

要するに、`CARLA` 依存を adapter へ閉じ込め、それ以外は pure module として保つ。

## 3. 推奨ディレクトリ構成

### 3.1 `ad_stack/`

`ad_stack/` は feature 実装の正本にする。

推奨:

```text
ad_stack/
  agents/
    expert_basic_agent.py
  overtake/
    domain/
      models.py
      target.py
      pass_rules.py
    application/
      state_machine.py
      decision_service.py
      lane_change_planner.py
      contracts.py
    policies/
      stopped_target_policy.py
      moving_target_policy.py  # 将来追加
    infrastructure/
      carla/
        snapshot_builder.py
        route_projection.py
        telemetry_mapper.py
```

現在の stopped-obstacle 実装はすでに上記の層へ分割済みで、以後はこの構成を維持したまま整理を続ける。

### この構成にしたい理由

- `state machine` を 1 箇所に閉じ込められる
- `stopped` と `moving` の違いを policy に押し込められる
- adapter 変更が pure logic に漏れにくい
- telemetry serialization を domain/application から切り離せる

### 3.2 `simulation/`

`simulation/` は generic infrastructure だけに寄せる。

残すもの:

- [run_route_loop.py](/home/masa/carla_alpamayo/simulation/pipelines/run_route_loop.py)
- [environment_config.py](/home/masa/carla_alpamayo/simulation/environment_config.py)
- 汎用 inspector 基盤
- generic pipeline / CLI

置かないもの:

- feature 固有 expectation
- feature 固有 regression runner
- feature 固有の設計メモ

つまり `simulation/` は「共通の実行基盤」であり、「overtake feature の正本」ではない。

### 3.3 `tests/`

`tests/` は feature test suite の正本にする。

推奨:

```text
tests/
  unit/
    ad_stack/
      overtake/
  integration/
    ad_stack/
      stopped_obstacle/
      moving_vehicle/
```

現状の [tests/integration/ad_stack/stopped_obstacle](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle) は方向として正しい。

今後はここに並列で

- `moving_vehicle/`

を追加できるようにしたい。

ここでは `tests/integration/ad_stack/` を feature suite の root とみなす。

- `tests/integration/ad_stack/stopped_obstacle/`
- `tests/integration/ad_stack/moving_vehicle/`

のように横に並べるほうが、現状の stopped-obstacle suite と整合する。

加えて、suite 間で共通化したいものは feature root 直下に shared 層を置く。

例:

```text
tests/integration/ad_stack/
  _shared/
    carla_runner.sh
    summary_assertions.py
    manifest_assertions.py
```

ここに置くもの:

- CARLA 起動停止
- summary / manifest の共通 assertion
- inspector 呼び出し helper

こうしておくと、`stopped_obstacle` と `moving_vehicle` で runner や確認ロジックを二重実装しにくい。

### 3.4 `scenarios/`

`scenarios/` は runtime が直接読む共有 asset の置き場として維持する。

残すもの:

- `scenarios/routes/...`
- `scenarios/environments/...`

考え方:

- `tests/` は test suite の正本
- `scenarios/` は route / environment asset の正本

つまり、run-config や expectation は `tests/` に寄せるが、route / environment JSON 自体は `scenarios/` に残す。

### 3.5 `docs/`

`docs/` には設計書だけを置く。

残すべきもの:

- 要件
- アーキテクチャ方針
- state machine 設計
- test design
- パラメータ決定の理由

置かないもの:

- run-config inventory
- verified artifact への逐次リンク集

それらは `tests/integration/.../README.md` 側へ寄せる。

## 4. moving vehicle 追い越しを見据えて今決めておくべきこと

### 4.1 `target` の共通表現

停止車両と moving vehicle を別物として持ちすぎない。

最低限共通化したい項目:

- `kind`
  - `single_actor`
  - `cluster`
- `motion_kind`
  - `stopped`
  - `slow_moving`
- `primary_actor_id`
- `member_actor_ids`
- `lane_id`
- `entry_distance_m`
- `exit_distance_m`
- `speed_mps`

これがあれば、

- stopped target
- moving target
- stopped cluster
- moving cluster

を同じ state machine で扱いやすい。

### 4.2 `required_clear_distance` を stopped 専用にしない

moving vehicle 追い越しで必要になるのは、

- ego speed
- target speed
- oncoming corridor
- lane change out / return distance
- safety margin

です。

したがって `required_clear_distance` は feature 共通の概念として持ち、

- stopped target では簡略式
- moving target では relative speed を使う式

に分けるのがよい。

### 4.3 `pass_complete` と `rejoin_gate` を target type 非依存にする

今後も共通化したいのはここです。

- `pass_complete`
  - target entry/exit を十分抜けたか
- `rejoin_gate`
  - origin lane front/rear gap
  - signal / junction / curve suppression

これらを target type から切り離しておくと、moving vehicle 追加時も state machine を増やさずに済む。

### 4.4 `visible corridor` は feature core に寄せる

moving vehicle 追い越しでは、停止障害物よりも

- oncoming visibility
- occlusion
- curvature

が重要になる。

これは `stopped obstacle` 専用ロジックに埋め込まず、overtake core の safety gate として持つべきです。

## 5. 直近のディレクトリ構成上の問題点

現時点で特に気になるもの:

- [simulation/tests](/home/masa/carla_alpamayo/simulation/tests)
  - ほぼ使っていない中途半端な場所になっている
- [preflight_validation.py](/home/masa/carla_alpamayo/ad_stack/overtake/infrastructure/carla/preflight_validation.py)
  - `run.py` / inspector が使う feature 固有 preflight の CARLA adapter
- [run.py](/home/masa/carla_alpamayo/ad_stack/run.py)
  - telemetry 組み立てと scenario preflight の feature 固有処理がまだ多い
- [expert_basic_agent.py](/home/masa/carla_alpamayo/ad_stack/agents/expert_basic_agent.py)
  - adapter / decision / execution が混ざっている

## 6. 段階的リファクタ計画

### Phase 1: 正本の整理

- `tests/integration/ad_stack/stopped_obstacle/` を suite の正本として維持
- `simulation/tests/` は将来的に空にする
- `docs/` と `tests/` の役割を明確にする

### Phase 2: stopped-obstacle logic の分割

- stopped-obstacle の pure rule を
  - domain
  - application
  - policies
 へ維持したまま、stopped 固有 naming を generic 化する

### Phase 3: CARLA adapter の明示化

- `expert_basic_agent.py` から
  - target extraction
  - lane gap extraction
  - route corridor extraction
  を infrastructure adapter module へ出す

### Phase 4: moving vehicle overtake の pure design 追加

- `moving_target_policy.py` を追加
- unit test を stopped と同じ層で先に作る
- integration suite は `tests/integration/ad_stack/moving_vehicle/` を新設

### Phase 5: 共通 telemetry へ移行

- domain/application は共通 debug DTO を出す
- manifest / summary / MCAP への写像は infrastructure / interface に寄せる
- stopped 専用 field 名を減らす

## 7. Done の定義

この refactor が完了したとみなせる条件は次です。

- `expert_basic_agent.py` が overtake state machine の本体を持たない
- `stopped obstacle` と `moving lead vehicle` が
  - 共通の state vocabulary
  - 共通の telemetry contract
  - 可能な限り共有された core transition
  を使う
- `simulation/` に feature 固有 suite 資産が残らない
- `tests/` 側に `stopped_obstacle/` と `moving_vehicle/` suite が並ぶ
- CARLA なし unit / contract / preflight test で大半を落とせる
- CARLA integration は scenario acceptance の確認だけに近づく

## 8. 当面の優先順位

優先度順:

1. `expert_basic_agent.py` の adapter / decision 分離
2. stopped-obstacle pure module の分割
3. `scenario_validation` の feature module 化
4. overtake telemetry schema の共通化
5. moving vehicle overtake の pure design 追加

いま無理にやらないもの:

- 全 scenario の一括 rename
- Town01 以外への同時展開
- learned policy との I/O まで含めた大改修
