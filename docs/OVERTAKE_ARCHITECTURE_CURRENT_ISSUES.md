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
- suite root に進行メモと実働資産が混在していた問題
  - 進行メモは [docs/stopped_obstacle](/home/masa/carla_alpamayo/docs/stopped_obstacle) に移した
- 未参照 legacy asset が残っていた問題
  - 旧 short env と旧 double-stopped run-config は削除した

## 2. いま残っている問題

### 2.1 `expert_basic_agent.py` に application orchestration がまだ残る

[expert_basic_agent.py](/home/masa/carla_alpamayo/ad_stack/agents/expert_basic_agent.py) は以前よりかなり薄いですが、まだ

- traffic light / follow / overtake の mode 切り替え
- application service の呼び出し順
- route-loop step 内の高水準分岐

を持っています。

これは stopped-obstacle だけなら許容できますが、`moving lead vehicle` を足すと

- target policy の切り替え
- moving target 固有の abort 条件
- conservative future estimate

が入り、再び agent が太りやすいです。

必要なのは、

- `OvertakeUseCase`
- `OvertakeFacade`
- もしくは `StepDecisionService`

のような明示的な application 入口です。

### 2.2 target policy seam はできたが、candidate builder はまだ stopped 前提

[snapshot_builder.py](/home/masa/carla_alpamayo/ad_stack/overtake/infrastructure/carla/snapshot_builder.py) は `target_policy` を受け取るようになりました。

ただし現在の adapter にはまだ

- `build_same_lane_stopped_targets()`
- `build_route_aligned_stopped_targets()`

があり、raw candidate の時点で stopped vehicle 前提です。

moving overtake を入れるなら、さらに

- raw candidate extraction
- target normalization / clustering
- target selection

を 3 段に分ける必要があります。

つまり次の seam は

- `TargetCandidateBuilder`
- `TargetPolicy`
- `TargetSelectionService`

です。

### 2.3 telemetry DTO は共通化されたが、payload 契約はまだ feature 寄り

[telemetry_mapper.py](/home/masa/carla_alpamayo/ad_stack/overtake/infrastructure/carla/telemetry_mapper.py) と domain DTO 化で改善はしました。

ただ、`planning_debug` にはまだ

- stopped-obstacle を中心にした field
- integration suite で重要だった field

がそのまま残っています。

moving overtake を足すなら、

- feature 共通の core telemetry
- target-specific extension

に分けないと、payload が平面的に増え続けます。

### 2.4 integration harness はまだ suite ごとの shell orchestration が主

[tests/integration/ad_stack/_shared](/home/masa/carla_alpamayo/tests/integration/ad_stack/_shared) に assertion helper はあります。

ただし、実行自体はまだ

- suite ごとの shell runner
- suite ごとの inspect 呼び出し順
- suite ごとの CARLA 起動停止ルール

が中心です。

moving suite を足すと、

- CARLA lifecycle
- inspect contract
- summary / manifest assertion

をもう一段共通化した `integration harness` が欲しくなります。

### 2.5 suite docs は整理されたが、長期的には `verified` と `exploratory` をさらに分けたい

現在の [README.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/README.md) は前より整理されています。

ただし今後 scenario が増えると、

- verified baseline
- verified but inspect-only
- exploratory / deferred

を別 directory か別 index で分けたくなる可能性があります。

今すぐの問題ではないですが、moving suite を追加する段階では考慮したほうがよいです。

## 3. 優先順位

moving overtake 前に優先して直すなら次です。

1. `expert_basic_agent.py` の orchestration を application facade に寄せる
2. candidate builder を stopped 前提から外す
3. telemetry を `core + extension` に分ける
4. integration harness を `_shared` に寄せる

## 4. いま言えること

停止障害物回避については、

- production / tests の境界
- generic parser と feature contract の境界
- policy seam
- suite root と planning memo の境界

はかなり改善しています。

なので次にやるべきことは、

- 「壊れているところを場当たりで直す」

ではなく、

- `moving lead vehicle` を入れても崩れない application / adapter 境界を仕上げる

ことです。
