# Overtake Architecture Current Issues

この文書は、`stopped obstacle` 向け refactor を一通り進めた後の「現時点の問題点」を整理したものです。

目的:

- いまの overtake 実装のどこが改善済みで、どこがまだ弱いかを分けて書く
- 今後 `moving lead vehicle` の追い越しを実装するときに、先に直すべき点を明確にする
- ディレクトリ構成と依存方向の観点で、どこに歪みが残っているかを明文化する

関連:

- [OVERTAKE_ARCHITECTURE_REFACTOR_PLAN.md](/home/masa/carla_alpamayo/docs/OVERTAKE_ARCHITECTURE_REFACTOR_PLAN.md)
- [STOPPED_OBSTACLE_LOGIC_DESIGN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_LOGIC_DESIGN.md)
- [README.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/README.md)

## 1. 先に言えること

現状は、初期状態よりかなり良いです。

- `ad_stack/overtake/` に `domain / application / policies / infrastructure / validation` がある
- `tests/integration/ad_stack/stopped_obstacle/` に suite の正本がある
- `simulation/` から suite 専用 inspector は外れた
- execution contract と suite assertion は `_shared` に寄った
- `stopped obstacle` の integration suite は成立している

つまり、もう「全部が `expert_basic_agent.py` に埋まっている」状態ではありません。

ただし、`moving lead vehicle` を追加する前提で見ると、まだ architecture と directory structure に弱い点があります。

## 2. いま残っている問題

### 2.1 target policy の注入点がまだ弱い

いまの [snapshot_builder.py](/home/masa/carla_alpamayo/ad_stack/overtake/infrastructure/carla/snapshot_builder.py) は、直接

- `build_stopped_obstacle_targets()`

を呼んでいます。

つまり、

- target 候補の抽出
- target policy の選択
- stopped-specific な target 正規化

が adapter の中で半分固定されています。

このまま `moving vehicle` を入れると、次のどちらかになりやすいです。

- `snapshot_builder.py` の中で `if stopped ... elif moving ...` が増える
- stopped 用 builder と moving 用 builder が二重化する

必要なのは、

- `TargetPolicy`
- `TargetSelectionService`

のような seam を先に明示して、

- adapter は raw candidate を集める
- policy は target を決める

に分けることです。

### 2.2 scenario contract が production package に漏れている

今は [scenario_config.py](/home/masa/carla_alpamayo/ad_stack/overtake/validation/scenario_config.py) と [preflight_validation.py](/home/masa/carla_alpamayo/ad_stack/overtake/infrastructure/carla/preflight_validation.py) が production 側にあります。

ここにある情報はかなり test-suite 寄りです。

- `scenario_kind`
- `nearest_signal_distance_m`
- `nearest_junction_distance_m`
- `route_aligned_adjacent_lane_available`

これらは runtime の一般機能というより、

- scenario authoring
- preflight contract
- integration verification

のための情報です。

このまま moving suite を足すと、

- production package が test scenario metadata を知り続ける
- feature を増やすたびに `validation/` が suite-specific config を抱える

ことになります。

理想は、

- production に残すのは pure な runtime rule だけ
- suite 固有の scenario contract は `tests/` 側の tooling に寄せる

です。

### 2.3 `simulation/environment_config.py` が generic parser と feature extension を両方持っている

今の [environment_config.py](/home/masa/carla_alpamayo/simulation/environment_config.py) は、

- generic な environment parser
- `overtake_scenario` の parse

を両方持っています。

しかも `simulation` が

- `ad_stack.overtake.validation`

を import しています。

これは依存方向として少し不自然です。

`simulation` を generic infrastructure とみなすなら、

- feature packageに依存しない

ほうが clean です。

整理の方向は 2 つあります。

1. `simulation/environment_config.py` を完全に generic に戻す  
   - `overtake_scenario` は raw dict のまま返す
   - feature 側で parse する

2. 逆に overtake 用 config loader を feature 側へ寄せる  
   - `simulation` には generic parser だけ残す

今のままだと、`simulation` が feature extension の入口になっています。

### 2.4 `ad_stack/run.py` がまだ generic route-loop と overtake feature の境界に立ちすぎている

[run.py](/home/masa/carla_alpamayo/ad_stack/run.py) は以前よりかなり薄くなりましたが、まだ

- overtake preflight
- overtake telemetry
- overtake-specific summary/manifest field

の orchestration を知っています。

ここが問題になるのは、

- `expert` 以外の policy
- future moving overtake
- overtake を持たない route-loop run

を同じ entrypoint で扱うときです。

今後の方向としては、

- `run.py` は generic route-loop lifecycle に専念
- feature-specific instrumentation は feature adapter 側の `collector` / `hooks` に寄せる

ほうが自然です。

### 2.5 `expert_basic_agent.py` はまだ interface と use-case の両方を少し持っている

現在の [expert_basic_agent.py](/home/masa/carla_alpamayo/ad_stack/agents/expert_basic_agent.py) はかなり薄くなりましたが、まだ

- route-loop step ごとの orchestration
- light / follow / overtake の mode 切り替え
- application service 呼び出し順

を一手に持っています。

これは stopped-obstacle だけなら許容できますが、moving vehicle を入れると

- target policy の切り替え
- moving target 固有の abort rule
- future-position conservative estimate

が入り、再び agent が肥大化しやすいです。

必要なのは、

- `OvertakeUseCase`
- `OvertakeFacade`

のような application 側の明示的な入口です。

agent は

- world state を読む
- service を呼ぶ
- control を適用する

だけに寄せたほうがよいです。

### 2.6 production domain に integration scenario 名が残っている

[models.py](/home/masa/carla_alpamayo/ad_stack/overtake/domain/models.py) には `ScenarioKind` として

- `blocked_static`
- `curve_clear`
- `near_junction_preflight_reject`
- `double_stopped_clustered`

のような integration suite 名が残っています。

これは production domain の責務としては強すぎます。

domain が知るべきなのは本来、

- target type
- suppress reason
- lane availability
- risk category

のような一般概念であって、

- test scenario の名前

ではありません。

この enum が残っていると、moving suite を足したときに production code が test naming に引っ張られます。

### 2.7 suite root に planning / progress / expectations が混在している

[tests/integration/ad_stack/stopped_obstacle](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle) には

- scenario expectation markdown
- runner
- inspector
- `NEXT_STEPS.md`
- `REFACTOR_PLAN.md`

が同居しています。

機能としては問題ありませんが、長期的には

- suite の正本
- 進行管理メモ

が同じ階層にあるので、探索コストが上がります。

moving suite を足すなら、

- `README.md`
- scenario docs
- `run_configs/`
- `runner`

のような実働資産と、

- `REFACTOR_PLAN.md`
- `NEXT_STEPS.md`

のような進行メモを少し分けたほうがよいです。

### 2.8 legacy asset がまだ少し残っている

後方互換を気にしない前提なら、未参照の旧 asset はまだ削れる可能性があります。

例:

- [town01_stopped_obstacle_double_stopped_long_expert.json](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_double_stopped_long_expert.json)
- [town01_stopped_obstacle_blocked_45s.json](/home/masa/carla_alpamayo/scenarios/environments/town01_stopped_obstacle_blocked_45s.json)
- [town01_stopped_obstacle_clear_45s.json](/home/masa/carla_alpamayo/scenarios/environments/town01_stopped_obstacle_clear_45s.json)

これらが本当に不要なら、moving suite を始める前に消したほうが directory はかなり見やすくなります。

## 3. moving lead vehicle 追い越しを始める前にやるべきこと

優先度順に並べると、次が自然です。

1. `snapshot_builder` から stopped policy の直呼びを外す  
   - target candidate 抽出と target policy 適用を分ける

2. scenario contract を production package からさらに剥がす  
   - suite-specific metadata は `tests/` 側へ寄せる

3. `simulation/environment_config.py` の依存方向を整理する  
   - generic parser と feature parser を分ける

4. `expert_basic_agent.py` の application 入口を明示する  
   - `OvertakeUseCase` か `Facade` を置く

5. legacy asset と suite root の整理をする  
   - moving suite を足したときの探索コストを下げる

## 4. ひとことで言うと

今の構成は、

- stopped-obstacle の refactor はかなり進んだ
- でも moving vehicle を入れるには、まだ
  - policy seam
  - scenario contract の置き場
  - generic infrastructure と feature extension の境界

が甘い

という状態です。

つまり、次の本命は「moving target policy を足すこと」そのものではなく、

- moving を足しても duplicated branch に戻らないように
- policy と contract の置き場をもう一段 clean にすること

です。
