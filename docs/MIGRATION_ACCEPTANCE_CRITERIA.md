# Migration Acceptance Criteria

学習系とデータ収集系のディレクトリ再編は、`learning/` と `data_collection/` に移したあとで次の条件を満たしたときに「移行完了」とみなします。

重要:

- ここでいう成功は「性能が良いこと」ではなく、「移行後レイアウトで主要コマンドが壊れていないこと」です
- 評価 run の `success=true/false` は走行タスクの成否なので、移行成否の判定には使いません
- 移行判定では「起動できる」「必要 artifact が出る」「新しい path を参照する」を見ます

## 1. 静的受け入れ条件

次がすべて通ること:

```bash
cd /media/masa/ssd_data/carla_alpamayo
PYTHONPATH="" uv run python -m data_collection.pipelines.collect.collect_route_loop --help
PYTHONPATH="" uv run python -m learning.pipelines.train.train_pilotnet --help
PYTHONPATH="" uv run python -m evaluation.pipelines.evaluate_pilotnet_loop --help
PYTHONPATH="" uv run python -m evaluation.pipelines.interactive_command_drive --help
PYTHONPATH="" uv run python -m compileall libs learning evaluation data_collection
```

判定:

- import error が出ない
- `data_collection/configs/routes/...` を既定値として解決できる
- `outputs/` と `data/manifests/` への path 解決が新構成で壊れていない

## 2. データ収集の smoke 条件

`CARLA` 起動済み環境で次が通ること:

```bash
cd /media/masa/ssd_data/carla_alpamayo
./data_collection/scripts/run_collect_town01.sh \
  --max-seconds 15 \
  --no-record-video
```

判定:

- `data/manifests/episodes/<episode_id>.jsonl` が生成される
- `outputs/collect/<episode_id>/front_rgb/*.png` が生成される
- `outputs/collect/<episode_id>/summary.json` が生成される
- summary の `route_config_path` が `data_collection/configs/routes/...` を指す

## 3. 学習の smoke 条件

既存 manifest を使って次が通ること:

```bash
cd /media/masa/ssd_data/carla_alpamayo
./learning/scripts/run_train_pilotnet.sh \
  --epochs 1 \
  --batch-size 8 \
  --num-workers 0
```

判定:

- `outputs/train/<run_id>/config.json` が生成される
- `outputs/train/<run_id>/summary.json` が生成される
- `outputs/train/<run_id>/best.pt` が生成される
- `config.json` 内の manifest path 群が旧 `pipelines/` や旧 `configs/routes/` に依存していない

## 4. 評価の smoke 条件

既存 checkpoint を使って次が通ること:

```bash
cd /media/masa/ssd_data/carla_alpamayo
PYTHONPATH="" uv run python -m evaluation.pipelines.evaluate_pilotnet_loop \
  --checkpoint outputs/train/<train_run>/best.pt \
  --route-config data_collection/configs/routes/town01_pilotnet_loop.json \
  --max-seconds 15 \
  --no-record-video
```

判定:

- `outputs/evaluate/<episode_id>/summary.json` が生成される
- `outputs/evaluate/<episode_id>/manifest.jsonl` が生成される
- summary の `route_config_path` が `data_collection/configs/routes/...` を指す
- summary の `model_checkpoint_path` が期待した checkpoint を指す

## 最終判定

次をすべて満たせば移行完了:

1. 静的受け入れ条件がすべて通る
2. データ収集 smoke が通る
3. 学習 smoke が通る
4. 評価 smoke が通る
