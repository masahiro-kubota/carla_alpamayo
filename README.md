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

3. 最小収集を走らせる

```bash
cd /media/masa/ssd_data/carla_alpamayo
./data_collection/scripts/run_collect_town01.sh
```

このラッパーは `uv run` で project の lock 済み環境を使います。`PYTHONPATH` は明示的に空にして、ROS 環境が混ざらないようにしています。

4. 固定ループ expert 収集を走らせる

```bash
cd /media/masa/ssd_data/carla_alpamayo
./data_collection/scripts/run_collect_town01_loop.sh
```

この run は `data_collection/configs/routes/town01_pilotnet_loop.json` の loop を `BasicAgent + PID` で 1 周させ、front camera と frame manifest に加えて `summary.json` と `front_rgb.mp4` を出します。既定値は `320x180`, `30 km/h`, `max_seconds=600` です。

4.5. junction 用の open route を収集する

```bash
cd /media/masa/ssd_data/carla_alpamayo
./data_collection/scripts/run_collect_town01_route.sh data_collection/configs/routes/town01_right_focus_se.json
```

追加収集の route 一覧と plot は [docs/TOWN01_JUNCTION_COLLECTION_PLAN.md](docs/TOWN01_JUNCTION_COLLECTION_PLAN.md) にまとめています。

first pack をまとめて流すときは batch wrapper を使います。

```bash
cd /media/masa/ssd_data/carla_alpamayo
./data_collection/scripts/run_collect_town01_junction_pack.sh
```

5. 既存 episode から動画を再生成する

```bash
cd /media/masa/ssd_data/carla_alpamayo
PYTHONPATH="" uv run python -m data_collection.pipelines.collect.render_episode_video \
  --episode-dir outputs/collect/<episode_id>
```

6. `PilotNet風` の lateral policy を学習する

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

7. 学習済み steer policy を recorded eval data で offline 評価する

```bash
cd /media/masa/ssd_data/carla_alpamayo
./learning/scripts/run_evaluate_pilotnet_dataset.sh \
  --checkpoint outputs/train/<train_run>/best.pt \
  --manifest-glob 'data/manifests/episodes/*.jsonl'
```

これは `CARLA` closed-loop ではなく、保存済み manifest に対して `MAE / RMSE` を見る offline 評価です。

8. 学習済み steer policy を fixed loop で closed-loop 評価する

```bash
cd /media/masa/ssd_data/carla_alpamayo
./evaluation/scripts/run_evaluate_pilotnet_loop.sh outputs/train/<train_run>/best.pt
```

評価は `git diff` が空の clean worktree でのみ実行できます。出力先は `outputs/evaluate/<route>_<timestamp>_<commit>/` の形で、実行時刻と commit id を含む一意なディレクトリ名になります。

9. 学習済み steer policy を手動 command で動かす

```bash
cd /media/masa/ssd_data/carla_alpamayo
export DISPLAY=:1
./evaluation/scripts/run_interactive_town01_command_drive.sh
```

既定では `outputs/train/pilotnet_best/best.pt` を使います。操作は terminal 上で `w=lanefollow`, `a=left`, `s=straight`, `d=right`, `q=quit` です。command は切り替えるまで保持されます。`DISPLAY` があると front camera の live preview window も出ます。既定の preview は `2x` 拡大で、さらに大きくしたいときは `--preview-scale 3` のように指定できます。

重要:

- `collect_*` 系は expert データ収集であり、camera-to-steer の E2E 推論ではありません
- 学習済み policy の評価では `steer` だけを learned policy に置き換えます
- `throttle/brake` は `BasicAgent` の longitudinal control を使います
- つまり現状の learned policy は full-control ではなく lateral-only E2E です

## いま入っているもの

- `data_collection.pipelines.collect.minimal_collect`
  - ランダム spawn の ego vehicle を立てる
  - front RGB camera を取り付ける
  - `Traffic Manager` の autopilot で走らせる
  - frame ごとに PNG と JSONL manifest を出す
- `libs.schemas.episode_schema`
  - 最低限の frame schema を定義する
- `data_collection.pipelines.collect.collect_route_loop`
  - `Town01` の固定 route を planner で完走させる
  - front RGB と frame manifest を出す
  - success criteria に沿った summary JSON と MP4 を出す
- `data_collection.pipelines.collect.render_episode_video`
  - 既存 episode の PNG 列から MP4 を再生成する
- `learning.pipelines.train.train_pilotnet`
  - `front RGB + speed (+ command) -> steer` の `PilotNet風` モデルを学習する
- `learning.pipelines.evaluate.evaluate_pilotnet_dataset`
  - saved manifest を使って offline に `MAE / RMSE` を評価する
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
- offline eval summary: 任意の `--summary-output`
- evaluate artifacts: `outputs/evaluate/<episode_id>/{summary.json,manifest.jsonl}`
- evaluate suite summary: `outputs/evaluate_suites/<suite_name>.json`

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
- route: `data_collection/configs/routes/town01_pilotnet_loop.json`
- target task: `front RGB + speed -> steer`
- success: `route_completion_ratio >= 0.99`, `collision_count == 0`, `max_stationary_seconds < 10`, `distance_to_goal_m <= 10`

2026-03-21 の確認では、`30 km/h`、`320x180` の fixed-loop run が `508.75 s` で成功しています。

fixed loop は baseline として維持しますが、次の main goal は [docs/TOWN01_INTERSECTION_GOAL.md](docs/TOWN01_INTERSECTION_GOAL.md) に切り替えています。つまり、今後の Town01 mainline は「特定 loop を通す」ではなく、「Town01 の任意交差点で valid movement を command 付きで通せるか」を基準に見ます。
movement inventory と生成済み route suite は [docs/TOWN01_MOVEMENT_COVERAGE.md](docs/TOWN01_MOVEMENT_COVERAGE.md) にまとめています。
eval route suite を checkpoint に流すときは `./evaluation/scripts/run_evaluate_town01_movement_suite.sh` を使います。

## Camera E2E

camera 画像からの learned steer 実験も 1 本回しています。結果は [docs/TOWN01_CAMERA_E2E_RESULTS.md](docs/TOWN01_CAMERA_E2E_RESULTS.md) にまとめました。
再現手順と、学習に使った data recipe / correction loop の整理は [docs/TOWN01_MAINLINE_REPRO.md](docs/TOWN01_MAINLINE_REPRO.md) にまとめています。
accepted run 当時の manifest 一覧をそのまま replay したいときは `./learning/scripts/run_train_town01_mainline_exact.sh` を使います。

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

## Junction Collection

次の追加収集計画は [docs/TOWN01_JUNCTION_COLLECTION_PLAN.md](docs/TOWN01_JUNCTION_COLLECTION_PLAN.md) にまとめています。

要点:

- `right` と `straight` を優先して増やす
- `Town01` の短い open route を 7 本使う
- first pack `31 / 31` は成功し、実測で `right = 1225`, `straight = 2113` まで増えた
- それでも fixed-loop 改善には直結しなかったので、次は evaluator / temporal context / left data を見る
- その後、full-loop の後半 failure を再現する upper-band route を 2 本追加した
- `town01_right_focus_upper_band_mid` と `town01_right_focus_upper_band_long` を各 `20` 本回し、`right = 7048`, `straight = 12138` まで増やした
- この拡張と correction windows を使った `frame_stack=3` branched policy が、`target-point` なしで fixed loop 完走に到達した

その次の high-curvature `lanefollow` pack は [docs/TOWN01_CURVE_COLLECTION_PLAN.md](docs/TOWN01_CURVE_COLLECTION_PLAN.md) にまとめています。

要点:

- loop 上の強い exit curve を 7 本の short route に分けて `35 / 35` 成功で収集した
- clean expert の `lanefollow` で `|steer| >= 0.5` は `90 -> 203`, `|steer| >= 0.7` は `19 -> 36` まで増えた
- この pack を route-target conditioned policy に混ぜることで fixed loop 完走まで届いた

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
    scripts/
    metrics/
    reports/
    runners/
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
