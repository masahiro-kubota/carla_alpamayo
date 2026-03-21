# Town01 Mainline Reproduction

`target-point` を使わない accepted mainline の再現手順です。

対象:

- task: `front RGB[t-2:t] + speed + command -> steer`
- route: `configs/routes/town01_pilotnet_loop.json`
- longitudinal control: `BasicAgent` / PID
- accepted checkpoint: `outputs/train/pilotnet_branch_fs3_20260321_231852/best.pt`
- accepted evaluation: `outputs/evaluate/town01_pilotnet_loop_pilotnet_eval_20260321_232707/summary.json`

## Preconditions

- workspace: `/home/masa/carla_alpamayo`
- `uv sync` 済み
- `CARLA 0.9.16` が `~/sim/carla-0.9.16` から参照できる
- GUI を使う場合は `export DISPLAY=:1`

## Data Recipe

accepted mainline は、`clean expert episodes` と `correction windows` を混ぜて学習しています。

accepted checkpoint の training summary:

- checkpoint: `outputs/train/pilotnet_branch_fs3_20260321_231852/best.pt`
- frame count after loader filtering: `161742`
- duration: `~134.8 min @ 20 Hz`
- target steer: `expert_steer`, fallback `steer`
- split mode: `episode`

clean expert episodes の主な出どころ:

- fixed loop expert: `town01_pilotnet_loop_*`
- counter-clockwise 補助 expert: `town01_pilotnet_loop_ccw_*`
- junction / turn 補強: `town01_right_focus_*`, `town01_straight_focus_*`, `town01_left_focus_*`
- high-curvature `lanefollow` 補強: `town01_curve_focus_*`
- full loop 後半 failure を潰す upper-band 補強:
  - `configs/routes/town01_right_focus_upper_band_mid.json`
  - `configs/routes/town01_right_focus_upper_band_long.json`

accepted mainline に入っている correction windows:

- `data/manifests/corrections/town01_eval_200434_lower_left_window.jsonl`
- `data/manifests/corrections/town01_eval_202110_bottom_right_window.jsonl`
- `data/manifests/corrections/town01_eval_203551_bottom_route_window.jsonl`
- `data/manifests/corrections/town01_eval_203114_bottom_right_window.jsonl`
- `data/manifests/corrections/town01_eval_210230_bottom_route_window.jsonl`
- `data/manifests/corrections/town01_eval_210244_bottom_right_window.jsonl`
- `data/manifests/corrections/town01_eval_210816_nw_right_window.jsonl`
- `data/manifests/corrections/town01_eval_230633_upper_band_mid_window.jsonl`
- `data/manifests/corrections/town01_eval_230644_upper_band_long_window.jsonl`

この correction set は `9` files, `407` frames, 約 `20.4` 秒です。量としては小さいですが、fixed loop 上の失敗文脈を局所的に埋める役割でした。

## Was This DAgger?

結論:

- full DAgger ではありません
- ただし、`DAgger 的な offline correction / dataset aggregation` は使っています

full DAgger ではない理由:

- learned policy rollout 中に expert override していない
- policy mixing をしていない
- 収集ループ全体が自動の interactive relabeling になっていない

今回やったこと:

1. learned checkpoint を fixed loop や short route で closed-loop 評価する
2. その rollout manifest に predicted `steer` と planner expert の `expert_steer` を両方保存する
3. failure 直前の短い window を `data/manifests/corrections/*.jsonl` として切り出す
4. clean expert episodes とその correction windows をまとめて再学習する

`expert_steer` は、learned-policy 評価中も `BasicAgent` を動かしているので、その lateral output をラベルとして保存しています。コード上は [pipelines/evaluate/evaluate_pilotnet_loop.py](../pipelines/evaluate/evaluate_pilotnet_loop.py) の `EpisodeRecord(... expert_steer=longitudinal_control.steer, ...)` です。schema 側は [libs/schemas/episode_schema.py](../libs/schemas/episode_schema.py) に `expert_steer` field があります。

つまり、表現としては `DAgger-like`, ただし実装としては `offline correction replay` と呼ぶのが正確です。

## 1. Fastest Check

既存の accepted checkpoint をそのまま評価し直す手順です。

```bash
cd /home/masa/carla_alpamayo
uv sync

cd ~/sim/carla-0.9.16
export DISPLAY=:1
./CarlaUE4.sh -quality-level=Low -RenderOffScreen -nosound
```

別 terminal:

```bash
cd /home/masa/carla_alpamayo
./scripts/run_evaluate_town01_mainline.sh
```

期待値:

- `route_completion_ratio >= 0.99`
- `collision_count = 0`
- `elapsed_seconds ~= 508`
- video path が `outputs/evaluate/.../front_rgb.mp4`

2026-03-21 の accepted run:

- summary: `outputs/evaluate/town01_pilotnet_loop_pilotnet_eval_20260321_232707/summary.json`
- video: `outputs/evaluate/town01_pilotnet_loop_pilotnet_eval_20260321_232707/front_rgb.mp4`

## 2. Retrain From Existing Manifests

現在の workspace にある expert episodes と correction manifests を使って、accepted mainline 設定を再学習する手順です。

```bash
cd /home/masa/carla_alpamayo
./scripts/run_train_town01_mainline.sh
```

この script は accepted recipe の hyperparameter を固定します。

- split mode: `episode`
- frame stack: `3`
- command conditioning: `branch`
- target steer: `expert_steer`, fallback `steer`
- learning rate: `1e-5`
- weight decay: `1e-4`
- init checkpoint: `outputs/train/pilotnet_branch_fs3_20260321_210500/best.pt`

入力データについて:

- script は現在の workspace にある mainline 用 manifest 群を pattern で拾います
- したがって、accepted run 以降に manifest を増やしていると、学習集合は増える可能性があります
- accepted run 当時の正確な manifest 一覧は `outputs/train/pilotnet_branch_fs3_20260321_231852/config.json` を参照してください

accepted training run の例:

- summary: `outputs/train/pilotnet_branch_fs3_20260321_231852/summary.json`
- frame count: `161742`
- best val loss: `0.005968`

## 3. Evaluate A Freshly Trained Checkpoint

学習後にできた run directory を評価します。

```bash
cd /home/masa/carla_alpamayo
./scripts/run_evaluate_town01_mainline.sh outputs/train/<train_run>/best.pt
```

## 4. If You Need To Rebuild The Added Data

今回の mainline success に直結した追加データは `upper-band` route です。

- route config: `configs/routes/town01_right_focus_upper_band_mid.json`
- route config: `configs/routes/town01_right_focus_upper_band_long.json`
- plot: `docs/assets/town01_junction_routes/town01_right_focus_upper_band_mid.png`
- plot: `docs/assets/town01_junction_routes/town01_right_focus_upper_band_long.png`

1 本ずつ取り直す例:

```bash
cd /home/masa/carla_alpamayo
./scripts/run_collect_town01_route.sh configs/routes/town01_right_focus_upper_band_mid.json --seed 700 --no-record-video
./scripts/run_collect_town01_route.sh configs/routes/town01_right_focus_upper_band_long.json --seed 900 --no-record-video
```

## Notes

- これは full-control E2E ではなく lateral-only E2E です
- `target-point` は使いません
- 再現性の主軸は `front RGB` 時系列 `3` 枚 + `speed` + `command`
- accepted mainline の correction loop は full DAgger ではなく、offline correction replay です
- `Town01` fixed loop の success criteria は [TOWN01_PILOTNET_ROUTE.md](./TOWN01_PILOTNET_ROUTE.md) を参照
