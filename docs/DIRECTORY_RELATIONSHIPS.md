# Directory Relationships

このドキュメントは、project root 直下のうち主にソースコードを持つディレクトリが、いまどう依存しているかをまとめたものです。

重要:

- ここでは `docs/`, `data/`, `outputs/` のような非ソースコード中心のディレクトリは図から省いています
- `scenarios/` はまだ現行の主要実行フローには組み込んでいないので、図から外しています

## 1. 現在の実装依存

```mermaid
flowchart LR
  data_collection["data_collection<br/>expert collection entrypoints"] -->|collect_route_loop builds SceneState / calls agents| ad_stack["ad_stack<br/>online agent interface / adapters"]
  evaluation["evaluation<br/>closed-loop eval / interactive drive"] -->|calls ad_stack agents / inference bridge| ad_stack

  ad_stack -->|call learned inference runtime| learning

  data_collection -->|shared helpers| libs["libs<br/>CARLA helper / schema / path"]
  learning -->|shared helpers| libs
  ad_stack -->|shared helpers| libs
  evaluation -->|shared helpers| libs
```

現状のポイント:

- `data_collection/` は route-based expert collector だけを持つ
  - `collect_route_loop` は `ad_stack.agents.ExpertBasicAgent` を使う
- `evaluation/` は `ad_stack` だけを見る
  - `evaluate_pilotnet_loop` は `ad_stack.agents.LearnedLateralAgent` を使う
  - `interactive_command_drive` は `ad_stack` の inference bridge を使う
- `ad_stack/` は learned lateral policy の推論時に `learning/` の inference runtime を呼ぶ
- `learning/` は offline train code と inference runtime を持つ
- `evaluation/` は `CARLA` closed-loop evaluator と、manual command の interactive drive を持つ

## 2. 現在の責務分担

- `data_collection/`
  - `CARLA` world を進める
  - sensor と ego 状態から `SceneState` を組み立てる
  - `ad_stack` の expert agent を呼ぶ
  - `EpisodeRecord` と画像を保存する
- `learning/`
  - dataset, model, train code を持つ
  - checkpoint load / preprocess / infer の runtime を持つ
- `ad_stack/`
  - `SceneState -> ControlDecision` の interface を提供する
  - `BasicAgent` adapter と learned lateral agent adapter を持つ
  - 必要に応じて `learning/` の inference runtime を呼ぶ
- `evaluation/`
  - closed-loop evaluator では `ad_stack` の agent interface を呼ぶ
  - interactive drive では `ad_stack` の inference bridge を呼ぶ
- `libs/`
  - route config, `CARLA` PythonAPI 接続補助, project root 解決, schema を持つ

## 3. ディレクトリ間インターフェース

### `data_collection/` -> `ad_stack/`

- 入力:
  - `ad_stack.world_model.scene_state.SceneState`
- 出力:
  - `ad_stack.agents.base.ControlDecision`
  - その中の `VehicleCommand`

実際の流れ:

- `data_collection/` が `ObservationBuilder` で `SceneState` を作る
- `ExpertBasicAgent.step(scene_state)` を呼ぶ
- 返ってきた `VehicleCommand` を `carla.VehicleControl` に変換して適用する

### `evaluation/` -> `ad_stack/`

- これは `evaluate_pilotnet_loop` に当てはまる
- `evaluation/pipelines/` の closed-loop evaluator が `SceneState` を作る
- `LearnedLateralAgent.step(scene_state)` を呼ぶ
- longitudinal は `ExpertBasicAgent`、lateral は learned policy で合成する

### `ad_stack/` -> `learning/`

- `ad_stack` は `learning.libs.ml.PilotNetInferenceRuntime` を呼ぶ
- `evaluation/` は `learning` を直接 import しない
- `interactive_command_drive` も `ad_stack.inference` 経由で runtime を読む
- 現在の入力は `SceneState.metadata` 経由で渡している
  - `front_rgb_history`
  - `command`
  - `route_point`

これは現在動いている interface だが、まだ暫定的です。将来的に厳密化するなら、`SceneState.metadata` ではなく typed な learned-observation 構造に切り出したほうがよいです。

## 4. 主要な公開面

### `ad_stack` が提供するもの

- `ad_stack.agents.base.AutonomyAgent`
  - `step(scene_state) -> ControlDecision`
- `ad_stack.agents.base.VehicleCommand`
  - `steer`, `throttle`, `brake`
- `ad_stack.world_model.scene_state.SceneState`
  - ego / route / metadata / optional world objects をまとめた状態
- `ad_stack.agents.ExpertBasicAgent`
  - `CARLA BasicAgent` の adapter
- `ad_stack.agents.LearnedLateralAgent`
  - learned steer policy を online loop に載せる agent

### `learning` が提供するもの

- model 定義
- checkpoint load / preprocess / infer の runtime
- `learning.libs.ml.PilotNetInferenceRuntime`

### `libs` が提供するもの

- route config / planned route
- `CARLA` PythonAPI への接続補助
- project root 解決
- JSONL schema
