# carla_alpamayo

`CARLA` を擬似フリート兼シミュレータとして使い、収集 -> 整形 -> 検索 -> 学習 -> 再学習のループを個人で回すための作業リポジトリです。

詳細な構想は [docs/PERSONAL_PROJECT_PLAN.md](docs/PERSONAL_PROJECT_PLAN.md) にまとめています。このルートでは、まず `front RGB + speed + command` を収集する最小パイプラインから始めます。

## 前提

- `uv` を使えること
- `CARLA 0.9.16` を `~/sim/carla-0.9.16` で参照できること
- Python 3.10 以上

今回の環境では、本体は容量都合で `/media/masa/ssd_data/sim/carla-0.9.16` に置き、`/home/masa/sim/carla-0.9.16` は symlink にしています。
Python ライブラリは `uv` で管理します。lockfile は `uv.lock`、仮想環境は `uv sync` が作る `.venv` を使います。

## クイックスタート

1. 依存を同期する

```bash
cd /media/masa/ssd_data/carla_alpamayo
uv sync
```

`carla` wheel は `pyproject.toml` から参照します。`CARLA` の配置を変えたら、その wheel パスに対して `uv add /path/to/carla-....whl` をやり直します。

2. `CARLA` を起動する

```bash
cd ~/sim/carla-0.9.16
export DISPLAY=:1
./CarlaUE4.sh -quality-level=Low -RenderOffScreen -nosound
```

3. 最小収集を走らせる

```bash
cd /media/masa/ssd_data/carla_alpamayo
./scripts/run_collect_town01.sh
```

このラッパーは `uv run` で project の lock 済み環境を使います。`PYTHONPATH` は明示的に空にして、ROS 環境が混ざらないようにしています。

4. 固定ループ expert 収集を走らせる

```bash
cd /media/masa/ssd_data/carla_alpamayo
./scripts/run_collect_town01_loop.sh
```

この run は `configs/routes/town01_pilotnet_loop.json` の loop を `BasicAgent + PID` で 1 周させ、front camera と frame manifest に加えて `summary.json` と `front_rgb.mp4` を出します。既定値は `320x180`, `30 km/h`, `max_seconds=600` です。

4.5. junction 用の open route を収集する

```bash
cd /media/masa/ssd_data/carla_alpamayo
./scripts/run_collect_town01_route.sh configs/routes/town01_right_focus_se.json
```

追加収集の route 一覧と plot は [docs/TOWN01_JUNCTION_COLLECTION_PLAN.md](docs/TOWN01_JUNCTION_COLLECTION_PLAN.md) にまとめています。

5. 既存 episode から動画を再生成する

```bash
cd /media/masa/ssd_data/carla_alpamayo
PYTHONPATH="" uv run python -m pipelines.collect.render_episode_video \
  --episode-dir outputs/collect/<episode_id>
```

6. `PilotNet風` の lateral policy を学習する

```bash
cd /media/masa/ssd_data/carla_alpamayo
./scripts/run_train_pilotnet.sh
```

`front RGB + speed + command -> steer` を試すときは、`PILOTNET_COMMAND_CONDITIONING=embedding` を付けます。

```bash
cd /media/masa/ssd_data/carla_alpamayo
PILOTNET_COMMAND_CONDITIONING=embedding ./scripts/run_train_pilotnet.sh
```

7. 学習済み steer policy を fixed loop で評価する

```bash
cd /media/masa/ssd_data/carla_alpamayo
./scripts/run_evaluate_pilotnet_loop.sh outputs/train/<train_run>/best.pt
```

重要:

- 現時点の fixed-loop run はまだ camera-to-steer の E2E 推論ではありません
- `steer/throttle/brake` は `BasicAgent.run_step()` が返す expert control です
- front camera は学習用の観測と teacher label を記録するために使っています

## いま入っているもの

- `pipelines.collect.minimal_collect`
  - ランダム spawn の ego vehicle を立てる
  - front RGB camera を取り付ける
  - `Traffic Manager` の autopilot で走らせる
  - frame ごとに PNG と JSONL manifest を出す
- `libs.schemas.episode_schema`
  - 最低限の frame schema を定義する
- `pipelines.collect.collect_route_loop`
  - `Town01` の固定 route を planner で完走させる
  - front RGB と frame manifest を出す
  - success criteria に沿った summary JSON と MP4 を出す
- `pipelines.collect.render_episode_video`
  - 既存 episode の PNG 列から MP4 を再生成する
- `pipelines.train.train_pilotnet`
  - `front RGB + speed (+ command) -> steer` の `PilotNet風` モデルを学習する
- `pipelines.evaluate.evaluate_pilotnet_loop`
  - learned steer を fixed loop で closed-loop 評価する
  - `command` 条件付き checkpoint もそのまま評価できる

## 出力先

- frame manifest: `data/manifests/episodes/<episode_id>.jsonl`
- 画像: `outputs/collect/<episode_id>/front_rgb/*.png`
- fixed-loop summary: `outputs/collect/<episode_id>/summary.json`
- fixed-loop video: `outputs/collect/<episode_id>/front_rgb.mp4`

2026-03-21 の確認結果:

- `Town01`
- `640x360`
- `400 frames`
- 約 `20` 秒
- 出力例: `outputs/collect/town01_20260321_022519/front_rgb/`

## 制約

- fixed loop の route planning は `Town01` だけ実装済み
- `collision` と `lane_invasion` はセンサーイベントがあった frame だけ反映する
- verified baseline の steering は learned E2E ではなく planner/PID expert 制御

## Town01 Baseline

固定ループの最初の成功条件は [docs/TOWN01_PILOTNET_ROUTE.md](docs/TOWN01_PILOTNET_ROUTE.md) にまとめています。現時点の定義は次です。

- map: `Town01`
- route: `configs/routes/town01_pilotnet_loop.json`
- target task: `front RGB + speed -> steer`
- success: `route_completion_ratio >= 0.99`, `collision_count == 0`, `max_stationary_seconds < 10`, `distance_to_goal_m <= 10`

2026-03-21 の確認では、`30 km/h`、`320x180` の fixed-loop run が `508.75 s` で成功しています。

## Camera E2E

camera 画像からの learned steer 実験も 1 本回しています。結果は [docs/TOWN01_CAMERA_E2E_RESULTS.md](docs/TOWN01_CAMERA_E2E_RESULTS.md) にまとめました。

要点:

- `front RGB + speed -> steer` の baseline は学習済み
- camera-only baseline は `600 s` ノーコリジョンだが `route_completion_ratio = 0.1547`
- `front RGB + speed + command -> steer` も追加収集なしで 1 回試した
- command-conditioned retry は `route_completion_ratio = 0.0797`, `41.5 s`, `collision`
- 現時点の判断は「architecture は正しいが、追加収集なしでは改善しない」

## Junction Collection

次の追加収集計画は [docs/TOWN01_JUNCTION_COLLECTION_PLAN.md](docs/TOWN01_JUNCTION_COLLECTION_PLAN.md) にまとめています。

要点:

- `right` と `straight` を優先して増やす
- `Town01` の短い open route を 7 本使う
- 最初の 1 パックで `right 1500+`, `straight 1800+` を狙う

## リポジトリ構成

```text
carla_alpamayo/
  README.md
  docs/
    PERSONAL_PROJECT_PLAN.md
  configs/
    collect/
  libs/
    schemas/
  pipelines/
    collect/
  scripts/
  data/
    manifests/
  outputs/
```
