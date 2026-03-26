# carla_alpamayo

`CARLA` を擬似フリート兼シミュレータとして使い、収集 -> 整形 -> 検索 -> 学習 -> 再学習のループを個人で回すための作業リポジトリです。

詳細な構想は [docs/PERSONAL_PROJECT_PLAN.md](docs/PERSONAL_PROJECT_PLAN.md) にまとめています。このルートでは、まず `front RGB + speed + command` を収集する最小パイプラインから始めます。
将来の NPC 含み full-stack 自動運転向けの設計は [docs/AD_STACK_ARCHITECTURE.md](docs/AD_STACK_ARCHITECTURE.md) に切り出しています。

現在の mainline 方針:

- planner 由来の連続 guidance 入力は使わない
- つまり `target-point` は今後の本筋実験では使わない
- mainline の許可入力は `front RGB` の時系列 stack + `speed`、必要なら `+ command` まで
- 既存の `target-point` 実験結果は参考用に残すが、accepted baseline には数えない

## 前提

- `uv` を使えること
- `CARLA 0.9.16` を `~/sim/carla-0.9.16` で参照できること
- Python 3.10 以上

今回の環境では、本体は容量都合で `/media/masa/ssd_data/sim/carla-0.9.16` に置き、`/home/masa/sim/carla-0.9.16` は symlink にしています。
Python ライブラリは `uv` で管理します。lockfile は `uv.lock`、仮想環境は `uv sync` が作る `.venv` を使います。

レイアウト再編後の受け入れ条件と smoke test 手順は [docs/MIGRATION_ACCEPTANCE_CRITERIA.md](docs/MIGRATION_ACCEPTANCE_CRITERIA.md) にまとめています。

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

3. 固定ループ expert 収集を走らせる

```bash
cd /media/masa/ssd_data/carla_alpamayo
./data_collection/scripts/run_collect_town01.sh
```

この run は `data_collection/configs/routes/town01_pilotnet_loop.json` の loop を `BasicAgent + PID` で 1 周させ、front camera と frame manifest に加えて `summary.json` と `front_rgb.mp4` を出します。既定値は `320x180`, `30 km/h`, `max_seconds=600` です。custom route を使うときは `--route-config path/to/route.json` か `CARLA_ROUTE_CONFIG` を渡します。

4. `PilotNet風` の lateral policy を学習する

```bash
cd /media/masa/ssd_data/carla_alpamayo
./learning/scripts/run_train_pilotnet.sh
```

`front RGB + speed + command -> steer` を試すときは、`PILOTNET_COMMAND_CONDITIONING=embedding` を付けます。

```bash
cd /media/masa/ssd_data/carla_alpamayo
PILOTNET_COMMAND_CONDITIONING=embedding ./learning/scripts/run_train_pilotnet.sh
```

episode 単位 split を使うときは `PILOTNET_SPLIT_MODE=episode` を付けます。複数 dataset を混ぜるときは `--manifest-glob` を複数回渡せます。

5. 学習済み steer policy を fixed loop で closed-loop 評価する

```bash
cd /media/masa/ssd_data/carla_alpamayo
PYTHONPATH="" uv run python -m evaluation.pipelines.evaluate_pilotnet_loop \
  --checkpoint outputs/train/<train_run>/best.pt \
  --route-config data_collection/configs/routes/town01_pilotnet_loop.json
```

評価は `git diff` が空の clean worktree でのみ実行できます。出力先は `outputs/evaluate/<route>_<timestamp>_<commit>/` の形で、実行時刻と commit id を含む一意なディレクトリ名になります。

6. 学習済み steer policy を手動 command で動かす

```bash
cd /media/masa/ssd_data/carla_alpamayo
export DISPLAY=:1
PYTHONPATH="" uv run python -m evaluation.pipelines.interactive_command_drive
```

既定では `outputs/train/pilotnet_best/best.pt` を使います。操作は terminal 上で `w=lanefollow`, `a=left`, `s=straight`, `d=right`, `q=quit` です。command は切り替えるまで保持されます。`DISPLAY` があると front camera の live preview window も出ます。既定の preview は `2x` 拡大で、さらに大きくしたいときは `--preview-scale 3` のように指定できます。

重要:

- `collect_*` 系は expert データ収集であり、camera-to-steer の E2E 推論ではありません
- 学習済み policy の評価では `steer` だけを learned policy に置き換えます
- `throttle/brake` は `BasicAgent` の longitudinal control を使います
- つまり現状の learned policy は full-control ではなく lateral-only E2E です

## いま入っているもの

- `libs.schemas.episode_schema`
  - 最低限の frame schema を定義する
- `data_collection.pipelines.collect.collect_route_loop`
  - `Town01` の固定 route を planner で完走させる
  - front RGB と frame manifest を出す
  - success criteria に沿った summary JSON と MP4 を出す
- `learning.pipelines.train.train_pilotnet`
  - `front RGB + speed (+ command) -> steer` の `PilotNet風` モデルを学習する
- `evaluation.pipelines.evaluate_pilotnet_loop`
  - learned steer を fixed loop で closed-loop 評価する
  - `command` 条件付き checkpoint もそのまま評価できる
- `evaluation.pipelines.interactive_command_drive`
  - learned steer を manual command と一緒に `CARLA` 上で試す

## 出力先

- frame manifest: `data/manifests/episodes/<episode_id>.jsonl`
- 画像: `outputs/collect/<episode_id>/front_rgb/*.png`
- fixed-loop summary: `outputs/collect/<episode_id>/summary.json`
- fixed-loop video: `outputs/collect/<episode_id>/front_rgb.mp4`
- train artifacts: `outputs/train/<run_id>/{config.json,summary.json,best.pt}`
- evaluate artifacts: `outputs/evaluate/<episode_id>/{summary.json,manifest.jsonl}`

fixed-loop expert collector の baseline 設定:

- `Town01`
- `320x180`
- `30 km/h`
- `max_seconds = 600`

## 制約

- fixed loop の route planning は `Town01` だけ実装済み
- `collision` と `lane_invasion` はセンサーイベントがあった frame だけ反映する
- verified baseline の steering は learned E2E ではなく planner/PID expert 制御

## Town01 Baseline

固定ループの最初の成功条件は [docs/TOWN01_PILOTNET_ROUTE.md](docs/TOWN01_PILOTNET_ROUTE.md) にまとめています。現時点の定義は次です。

- map: `Town01`
- route: `data_collection/configs/routes/town01_pilotnet_loop.json`
- target task: `front RGB + speed -> steer`
- success: `route_completion_ratio >= 0.99`, `collision_count == 0`, `max_stationary_seconds < 10`, `distance_to_goal_m <= 10`

2026-03-21 の確認では、`30 km/h`、`320x180` の fixed-loop run が `508.75 s` で成功しています。

fixed loop は baseline として維持しますが、次の main goal は [docs/TOWN01_INTERSECTION_GOAL.md](docs/TOWN01_INTERSECTION_GOAL.md) に切り替えています。つまり、今後の Town01 mainline は「特定 loop を通す」ではなく、「Town01 の任意交差点で valid movement を command 付きで通せるか」を基準に見ます。

## Camera E2E

camera 画像からの learned steer 実験も 1 本回しています。結果は [docs/TOWN01_CAMERA_E2E_RESULTS.md](docs/TOWN01_CAMERA_E2E_RESULTS.md) にまとめました。
再現手順と、学習に使った data recipe / correction loop の整理は [docs/TOWN01_MAINLINE_REPRO.md](docs/TOWN01_MAINLINE_REPRO.md) にまとめています。

要点:

- `front RGB + speed -> steer` の baseline は学習済み
- camera-only baseline は `600 s` ノーコリジョンだが `route_completion_ratio = 0.1547`
- `front RGB + speed + command -> steer` も追加収集なしで 1 回試した
- command-conditioned retry は `route_completion_ratio = 0.0797`, `41.5 s`, `collision`
- `target-point` を足した route-conditioned 実験では完走できたが、これは mainline には採用しない
- accepted mainline は `frame_stack=3` の `front RGB + speed + command -> steer`
- 現在保持している baseline checkpoint は `outputs/train/pilotnet_best/best.pt`
- fixed loop の成功 run は `outputs/evaluate/town01_pilotnet_loop_pilotnet_eval_20260321_232707/summary.json`
- その run は `route_completion_ratio = 0.9991`, `collision_count = 0`, `elapsed_seconds = 507.95`, `video_path` あり
- accepted run では fixed loop に加えて補助 route と correction windows も使った
- 補助 route の historical 内訳は [docs/TOWN01_MAINLINE_REPRO.md](docs/TOWN01_MAINLINE_REPRO.md) に残している
- 現在の minimal repo では、収集用 route config は baseline loop だけ保持している

## リポジトリ構成

直下ディレクトリ間の依存関係は [docs/DIRECTORY_RELATIONSHIPS.md](docs/DIRECTORY_RELATIONSHIPS.md) に分けてあります。

```text
carla_alpamayo/
  README.md
  pyproject.toml
  uv.lock
  ad_stack/
    agents/
    configs/
    control/
    planning/
    runtime/
    safety/
    scripts/
    world_model/
  data/
    manifests/
      corrections/
      episodes/
  data_collection/
    configs/
    pipelines/
    scripts/
  docs/
    DIRECTORY_RELATIONSHIPS.md
    AD_STACK_ARCHITECTURE.md
  learning/
    libs/
    pipelines/
    scripts/
  evaluation/
    pipelines/
  libs/
    carla_utils/
    project.py
    schemas/
    utils/
  outputs/
    collect/
    evaluate/
    evaluate_suites/
    train/
  scenarios/
    eval_suites/
    npc_profiles/
    routes/
    traffic_setups/
```
