# carla_alpamayo

`CARLA` 上で

- expert データ収集
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

## 収集

fixed loop の expert 収集:

```bash
cd /home/masa/carla_alpamayo
./simulation/scripts/run_collect_town01.sh
```

内部では `simulation.pipelines.run_route_loop` が `RunRequest(mode="collect", ...)` を作り、`ad_stack.run(...)` を呼びます。

主な出力:

- manifest: `data/manifests/episodes/<episode_id>.jsonl`
- 画像: `outputs/collect/<episode_id>/front_rgb/*.png`
- summary: `outputs/collect/<episode_id>/summary.json`
- video: `outputs/collect/<episode_id>/front_rgb.mp4`
- mcap: `outputs/collect/<episode_id>/telemetry.mcap`

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
  --mode evaluate \
  --checkpoint outputs/train/<train_run>/best.pt \
  --route-config scenarios/routes/town01_pilotnet_loop.json
```

`simulation.pipelines.run_route_loop` は `collect` / `evaluate` の両方で clean git worktree が必要です。`evaluate` の出力先は `outputs/evaluate/<route>_<timestamp>_<commit>/` です。
default では `telemetry.mcap` も出力し、`front_rgb` の JPEG と ego pose / control / route progress を記録します。不要なら `--no-record-mcap` を使います。

フロントカメラをライブ表示しながら評価したいときは `DISPLAY` を設定して `--show-front-camera` を付けます。

```bash
cd /home/masa/carla_alpamayo
export DISPLAY=:1
PYTHONPATH="" uv run python -m simulation.pipelines.run_route_loop \
  --mode evaluate \
  --policy-kind expert \
  --route-config scenarios/routes/town01_signal_short.json \
  --traffic-setup scenarios/traffic_setups/town01_signal_resume_phase2.json \
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
- `learning/`
  - `PilotNet` の model / train / inference runtime
- `libs/`
  - route helper, schema, project helper

## リポジトリ構成

```text
carla_alpamayo/
  ad_stack/
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
    routes/
```
