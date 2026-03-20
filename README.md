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

## いま入っているもの

- `pipelines.collect.minimal_collect`
  - ランダム spawn の ego vehicle を立てる
  - front RGB camera を取り付ける
  - `Traffic Manager` の autopilot で走らせる
  - frame ごとに PNG と JSONL manifest を出す
- `libs.schemas.episode_schema`
  - 最低限の frame schema を定義する

## 出力先

- frame manifest: `data/manifests/episodes/<episode_id>.jsonl`
- 画像: `outputs/collect/<episode_id>/front_rgb/*.png`

2026-03-21 の確認結果:

- `Town01`
- `640x360`
- `400 frames`
- 約 `20` 秒
- 出力例: `outputs/collect/town01_20260321_022519/front_rgb/`

## 制約

- high-level command は今は `lane_follow` 固定
- route planning はまだ未実装
- `collision` と `lane_invasion` はセンサーイベントがあった frame だけ反映する

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
