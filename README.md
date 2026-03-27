# carla_alpamayo

`CARLA` 上で

- expert route-loop 実行
- `PilotNet` 系の学習
- 学習済み checkpoint の closed-loop 評価
- 手動 command 付き interactive 試走

を回すための最小リポジトリです。

現在の実行アーキテクチャは [docs/DIRECTORY_RELATIONSHIPS.md](docs/DIRECTORY_RELATIONSHIPS.md) にまとめています。`simulation/` は薄い CLI wrapper で、実行本体は `ad_stack.run(request)` に集約しています。
信号遵守と overtaking を含む将来の privileged expert policy 要件は [docs/EXPERT_POLICY_REQUIREMENTS.md](docs/EXPERT_POLICY_REQUIREMENTS.md) にまとめています。
具体的な内部設計は [docs/EXPERT_POLICY_DESIGN.md](docs/EXPERT_POLICY_DESIGN.md) にまとめています。

## 前提

- `uv` を使えること
- `CARLA 0.9.16` を `~/sim/carla-0.9.16` で参照できること
- Python 3.10 以上

## セットアップ

```bash
cd /home/masa/carla_alpamayo
uv sync
```

`carla` wheel は `pyproject.toml` から参照します。`CARLA` の配置を変えたら wheel path も更新してください。

## CARLA 起動

```bash
cd ~/sim/carla-0.9.16
export DISPLAY=:1
./CarlaUE4.sh -quality-level=Low -RenderOffScreen -nosound
```

## Expert Route Loop

fixed loop の expert 実行:

```bash
cd /home/masa/carla_alpamayo
./simulation/scripts/run_expert_town01.sh
```

内部では `simulation.pipelines.run_route_loop` が `RunRequest(mode="evaluate", policy.kind="expert", ...)` を作り、`ad_stack.run(...)` を呼びます。
route は `scenarios/routes/*.json`、周辺環境と stopping 条件は `scenarios/environments/*.json`、ego 側の expert policy 閾値は `ad_stack/configs/expert/*.json` で管理します。
`run_expert_town01.sh` は fixed wrapper で、任意の追加 CLI 引数は受けません。切り替えは `CARLA_ROUTE_CONFIG`, `CARLA_ENVIRONMENT_CONFIG`, `CARLA_EXPERT_CONFIG` などの環境変数で行います。

主な出力:

- manifest: `outputs/evaluate/<run_id>/manifest.jsonl`
- summary: `outputs/evaluate/<run_id>/summary.json`
- video: `outputs/evaluate/<run_id>/front_rgb.mp4`
- mcap: `outputs/evaluate/<run_id>/telemetry.mcap`
- raw argv: `outputs/evaluate/<run_id>/cli_args.json`
- resolved request: `outputs/evaluate/<run_id>/run_request.json`

## 学習

`PilotNet` 系の学習:

```bash
cd /home/masa/carla_alpamayo
./learning/scripts/run_train_pilotnet.sh
```

`front RGB + speed + command -> steer` を試すときは:

```bash
cd /home/masa/carla_alpamayo
PILOTNET_COMMAND_CONDITIONING=embedding ./learning/scripts/run_train_pilotnet.sh
```

train artifact は `outputs/train/<run_id>/{config.json,summary.json,best.pt}` に出ます。

## 評価

closed-loop 評価:

```bash
cd /home/masa/carla_alpamayo
PYTHONPATH="" uv run python -m simulation.pipelines.run_route_loop \
  --checkpoint outputs/train/<train_run>/best.pt \
  --route-config scenarios/routes/town01_pilotnet_loop.json \
  --expert-config ad_stack/configs/expert/default.json
```

`simulation.pipelines.run_route_loop` は clean git worktree が必要です。出力先は `outputs/evaluate/<date>_<time>_<memo>_<commit>/` です。
default では `telemetry.mcap` を出力し、front camera の JPEG、ego pose / control / route progress、Foxglove `SceneUpdate` の static route / lane centerline を記録します。地図は default で town 全体を出し、重い場合だけ `--mcap-map-scope near_route` で route 近傍に絞れます。不要なら `--no-record-mcap` を使います。`front_rgb/` の連番 PNG は常設せず、`--record-video` のときだけ一時フレームから `front_rgb.mp4` を生成します。
default の front camera は `1280x720`、artifact 記録レートは `10Hz` です。必要なら `--camera-width`, `--camera-height`, `--record-hz` で上書きできます。
route-loop 実行時の raw argv は `cli_args.json`、解決済み request は `run_request.json` として出力ディレクトリに保存します。
expert policy の設定値は `summary.json` に `expert_config_path` と `expert_config` として残ります。

フロントカメラをライブ表示しながら評価したいときは `DISPLAY` を設定して `--show-front-camera` を付けます。

```bash
cd /home/masa/carla_alpamayo
export DISPLAY=:1
PYTHONPATH="" uv run python -m simulation.pipelines.run_route_loop \
  --policy-kind expert \
  --route-config scenarios/routes/town01_signal_short.json \
  --environment-config scenarios/environments/town01_signal_resume_phase2.json \
  --expert-config ad_stack/configs/expert/no_overtake.json \
  --show-front-camera
```

interactive 試走:

```bash
cd /home/masa/carla_alpamayo
export DISPLAY=:1
PYTHONPATH="" uv run python -m simulation.pipelines.interactive_command_drive
```

操作:

- `w`: `lanefollow`
- `a`: `left`
- `s`: `straight`
- `d`: `right`
- `q`: quit

## いまの責務分割

- `simulation/`
  - collect / evaluate / interactive 用 CLI
  - `RunRequest` を組み立てる
- `ad_stack/`
  - `run(request)` を提供する single entrypoint
  - CARLA world setup, actor/sensor lifecycle, simulation loop, artifact 出力を持つ
  - `configs/expert/*.json` に ego expert policy の閾値を置く
- `learning/`
  - `PilotNet` の model / train / inference runtime
- `libs/`
  - route helper, schema, project helper

## リポジトリ構成

```text
carla_alpamayo/
  ad_stack/
    configs/
      expert/
  data/
    manifests/
  docs/
    AD_STACK_SINGLE_ENTRYPOINT_PLAN.md
    DIRECTORY_RELATIONSHIPS.md
  learning/
    libs/
    pipelines/
    scripts/
  libs/
    carla_utils/
    project.py
    schemas/
    utils/
  outputs/
    collect/
    evaluate/
    train/
  simulation/
    pipelines/
    scripts/
  scenarios/
    environments/
    npc_profiles/
    routes/
```
