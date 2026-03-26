# AD Stack Architecture

最終目標は、`CARLA` 上で次を満たす online AD stack を持つことです。

- NPC vehicle を含む交通環境で走る
- 信号状態に従って停止・発進する
- 先行車追従、追い越し、回避を扱う
- 学習済み policy を一部または全部に差し替えられる

## ディレクトリ責務

- `ad_stack/`
  - online 実行時の agent adapter / observation builder / scene model
- `scenarios/`
  - route, NPC profile, 交通セットアップ, eval suite 定義
- `evaluation/`
  - closed-loop simulator evaluation, interactive drive, suite summary
- `data_collection/`
  - expert や simulator を使った収集
- `learning/`
  - dataset, train, offline eval, inference
- `libs/`
  - CARLA 接続や schema などの共通部品

## 実行時アーキテクチャ

現在の最小実装は `runtime -> world_model -> agents -> command` です。

1. `runtime`
   - `SceneState` を組み立てる薄い adapter
2. `world_model`
   - ego, route progress, learned-policy metadata を `SceneState` に集約
3. `agents`
   - `BasicAgent` adapter と learned lateral agent adapter を提供する

## 既存コードとのつなぎ方

短期的には、既存の `BasicAgent` expert を `ad_stack/agents/expert_basic_agent.py` で包みます。
これで `data_collection` や `evaluation` は「CARLA 付属 agent を直接呼ぶ」のではなく「repo 内の agent interface を呼ぶ」形に寄せられます。

中期的には、`learning` 側の learned lateral policy を `ad_stack/agents/learned_lateral_agent.py` に差し込みます。
planner / controller / safety を本格的に持つ full stack は、必要になった段階で別途追加します。

## 段階的な移行順

1. `BasicAgent` wrapper を使うように既存収集・評価 code を寄せる
2. `ObservationBuilder` で `SceneState` を共通化する
3. learned lateral policy を `ad_stack` 経由で closed-loop evaluator に載せる
4. 必要になったら planner / controller / safety を別モジュールとして追加する
