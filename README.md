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

## 並列収集

このマシンでは、`60s` の expert route-loop を `policy.ignore_traffic_lights=true` かつ `artifacts.record_video=false` 相当の設定で実測した結果、収集用途の実用上限は `3並列` でした。

- `1並列`: 約 `2.67x realtime`
- `2並列`: 約 `4.42x realtime`
- `3並列`: 約 `4.85x realtime`
- `4並列`: 約 `3.30x realtime`

そのため、長時間の収集は `3並列` を推奨します。`4並列` では `RTX 4070 Ti 12GB` の GPU 使用率と VRAM 使用率がほぼ飽和して、aggregate throughput が逆に落ちました。

独立した CARLA server を増やすときは、`RPC port` をずらすだけでなく `-carla-streaming-port=0` を付けます。今回の環境では次の port family が安定しました。

- `2000` / `2002`
- `2004` / `2006`
- `2008` / `2010`

起動例:

```bash
cd ~/sim/carla-0.9.16
export DISPLAY=:1

./CarlaUE4.sh -quality-level=Low -RenderOffScreen -nosound
./CarlaUE4.sh -quality-level=Low -RenderOffScreen -nosound -carla-rpc-port=2004 -carla-streaming-port=0
./CarlaUE4.sh -quality-level=Low -RenderOffScreen -nosound -carla-rpc-port=2008 -carla-streaming-port=0
```

それぞれに対して、run config JSON 内の `runtime.port` を `2000`, `2004`, `2008` に振り分けます。

## Expert Route Loop

route-loop 実行は JSON-only です。CLI override や環境変数 override は使わず、設定はすべて `simulation/run_configs/*.json` に書きます。

単発の expert 実行:

```bash
cd /home/masa/carla_alpamayo
PYTHONPATH="" uv run python -m simulation.pipelines.run_route_loop \
  simulation/run_configs/town01_perimeter_cw_expert.json
```

wrapper を使うなら:

```bash
cd /home/masa/carla_alpamayo
./simulation/scripts/run_expert_town01.sh simulation/run_configs/town01_perimeter_cw_expert.json
```

内部では `simulation.pipelines.run_route_loop` が run config JSON から `RunRequest(mode="evaluate", policy.kind=..., ...)` を作り、`ad_stack.run(...)` を呼びます。
route は `scenarios/routes/*.json`、周辺環境と stopping 条件は `scenarios/environments/*.json`、ego 側の expert policy 閾値は `ad_stack/configs/expert/*.json` で管理します。

主な出力:

- manifest: `outputs/evaluate/<run_id>/manifest.jsonl`
- summary: `outputs/evaluate/<run_id>/summary.json`
- video: `outputs/evaluate/<run_id>/front_rgb.mp4`
- mcap: `outputs/evaluate/<run_id>/telemetry/segment_0000.mcap`
- raw argv: `outputs/evaluate/<run_id>/cli_args.json`
- input run config: `outputs/evaluate/<run_id>/run_config.json`
- resolved request: `outputs/evaluate/<run_id>/run_request.json`

3 並列で回すときは、config JSON を 3 つ並べるだけです。

```bash
cd /home/masa/carla_alpamayo
PYTHONPATH="" uv run python -m simulation.pipelines.run_route_loop \
  simulation/run_configs/town01_perimeter_cw_collection.json \
  simulation/run_configs/town01_intersection_weave_cw_collection.json \
  simulation/run_configs/town01_intersection_weave_ccw_collection.json
```

この場合、元コマンドと各 worker の config/log/exit code は `outputs/launcher_runs/<run_id>/launcher_manifest.json` に残ります。

## Route Preview

`scenarios/routes/*.json` は anchor index の列だけだと人間には分かりにくいので、route JSON と同名の preview PNG を `scenarios/routes/previews/` に併せて管理します。

route preview を全部更新するときは:

```bash
cd /home/masa/carla_alpamayo
PYTHONPATH="" uv run python -m simulation.pipelines.plot_route_map --all
```

個別 route だけ更新するときは:

```bash
cd /home/masa/carla_alpamayo
PYTHONPATH="" uv run python -m simulation.pipelines.plot_route_map \
  --route-config scenarios/routes/town01_perimeter_cw.json
```

出力先は `scenarios/routes/previews/<route_name>.png` です。

Town 全体の top-down map asset を更新するときは:

```bash
cd /home/masa/carla_alpamayo
PYTHONPATH="" uv run python -m simulation.pipelines.render_town_topdown_asset --town Town01
```

出力先は `scenarios/maps/town01_topdown.png` と `scenarios/maps/town01_topdown.json` です。
この `town01_topdown.png` はレーン図ではなく、Town 全体を真上から撮った RGB の俯瞰画像です。

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

closed-loop 評価も同じで、checkpoint を含む learned 用 run config JSON を 1 つ渡します。

`simulation.pipelines.run_route_loop` は clean git worktree が必要です。出力先は `outputs/evaluate/<date>_<time>_<memo>_<commit>/` です。
default では `telemetry/segment_0000.mcap` から始まる segmented MCAP を出力し、front camera の JPEG、`/ego/state`, `/ego/control`, `/ego/planning`, `/tf`、Foxglove `SceneUpdate` の static route / lane centerline を記録します。Town ごとの top-down asset が `scenarios/maps/` にある場合は `/map/topdown/compressed` と `/map/topdown/camera_info` も各 segment 先頭に入ります。segment 一覧は `telemetry/index.json` に残り、default の split 間隔は `600s` です。必要なら run config JSON の `artifacts.mcap_segment_seconds` で変えられます。地図は default で town 全体を出し、重い場合だけ `artifacts.mcap_map_scope` を `near_route` にします。不要なら `artifacts.record_mcap` を `false` にします。`front_rgb/` の連番 PNG は常設せず、`artifacts.record_video=true` のときだけ一時フレームから `front_rgb.mp4` を生成します。
default の front camera は `1280x720`、artifact 記録レートは `10Hz` です。必要なら run config JSON の `runtime.camera_width`, `runtime.camera_height`, `artifacts.record_hz` を変えます。
route-loop 実行時の raw argv は `cli_args.json`、入力 run config は `run_config.json`、解決済み request は `run_request.json` として出力ディレクトリに保存します。
expert policy の設定値は `summary.json` に `expert_config_path` と `expert_config` として残ります。

フロントカメラをライブ表示したいときは、run config JSON の `preview.show_front_camera` を `true` にした上で `DISPLAY` を設定します。

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
