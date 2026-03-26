# AD Stack Architecture

最終目標は、`CARLA` 上で次を満たす online AD stack を持つことです。

- NPC vehicle を含む交通環境で走る
- 信号状態に従って停止・発進する
- 先行車追従、追い越し、回避を扱う
- 学習済み policy を一部または全部に差し替えられる

## ディレクトリ責務

- `ad_stack/`
  - online 実行時の agent / planner / controller / safety guard
- `scenarios/`
  - route, NPC profile, 交通セットアップ, eval suite 定義
- `evaluation/`
  - scenario runner, metrics, suite summary
- `data_collection/`
  - expert や simulator を使った収集
- `learning/`
  - dataset, train, inference
- `libs/`
  - CARLA 接続や schema などの共通部品

## 実行時アーキテクチャ

`runtime -> world_model -> planning -> control -> safety -> command`

1. `runtime`
   - `CARLA` client, actor spawn, sensor bridge
2. `world_model`
   - ego, NPC, traffic light, route progress を `SceneState` に集約
3. `planning.behavior`
   - `lane_follow`, `follow_vehicle`, `stop_for_red_light`, `overtake`, `avoid_obstacle`
4. `planning.motion`
   - target speed と target steer に落とす
5. `control`
   - steer / throttle / brake を生成
6. `safety`
   - emergency brake や hard veto をかける

## 既存コードとのつなぎ方

短期的には、既存の `BasicAgent` expert を `ad_stack/agents/expert_basic_agent.py` で包みます。
これで `data_collection` や `evaluation` は「CARLA 付属 agent を直接呼ぶ」のではなく「repo 内の agent interface を呼ぶ」形に寄せられます。

中期的には、`learning` 側の learned lateral policy を `ad_stack/agents/learned_lateral_agent.py` に差し込み、
最終的には `ad_stack/agents/full_stack_agent.py` の planner/controller/safety 置換まで進めます。

## 段階的な移行順

1. `BasicAgent` wrapper を使うように既存収集・評価 code を寄せる
2. `scenarios/routes/` に online 用 route 定義を置く
3. `ObservationBuilder` で `SceneState` を共通化する
4. rule-based `FullStackAgent` で信号停止と先行車追従を作る
5. 追い越し・回避を `behavior planner` に追加する
6. 学習済み module を planner / controller / policy に段階投入する
