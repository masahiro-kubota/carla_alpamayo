# Town01 Camera E2E Results

2026-03-21 時点の `PilotNet風` lateral-only E2E 実験メモ。

## Setup

- task: `front RGB + speed (+ optional command) -> steer`
- route intent: planner が保持
- longitudinal: `BasicAgent` / PID
- route: `configs/routes/town01_pilotnet_loop.json`
- weather: `ClearNoon`

重要:

- これは full control の E2E ではなく、**lateral-only E2E**
- `steer` だけを learned policy に置き換え、`throttle/brake` は planner 側を使う

## Data Collection

使った manifest:

- `data/manifests/episodes/town01_pilotnet_loop_20260321_030414.jsonl`
- `data/manifests/episodes/town01_pilotnet_loop_20260321_030441.jsonl`
- `data/manifests/episodes/town01_pilotnet_loop_20260321_030547.jsonl`
- `data/manifests/episodes/town01_pilotnet_loop_20260321_031241.jsonl`
- `data/manifests/episodes/town01_pilotnet_loop_ccw_20260321_031921.jsonl`

集計:

- 総 frame 数: `14949`
- うち full lap success expert: `town01_pilotnet_loop_20260321_030547`
- CCW expert は `65.6 s` で collision。turn 例としてだけ使う

### Junction Pack Expansion

追加収集:

- `docs/TOWN01_JUNCTION_COLLECTION_PLAN.md` の first pack を `31 / 31` 成功で回した
- 新規 route は `right_focus_*` と `straight_focus_*`

追加後の command counts:

- `lanefollow = 23694`
- `left = 600`
- `straight = 2113`
- `right = 1225`

## Training

### Baseline A: Camera + Speed

train output:

- `outputs/train/pilotnet_20260321_031957/`

best checkpoint:

- `outputs/train/pilotnet_20260321_031957/best.pt`

best summary:

- best epoch: `7`
- best val loss: `0.015362`
- device: `cuda`

### Baseline B: Camera + Speed + Command

train output:

- `outputs/train/pilotnet_cmd_20260321_045713/`

best checkpoint:

- `outputs/train/pilotnet_cmd_20260321_045713/best.pt`

best summary:

- best epoch: `10`
- best val loss: `0.015184`
- device: `cuda`
- command conditioning: `embedding`
- command rebalancing: `enabled`

command frame counts:

- `lanefollow = 13603`
- `left = 600`
- `straight = 529`
- `right = 219`

### Baseline C: Camera + Speed + Command + Junction Pack

train output:

- `outputs/train/pilotnet_cmd_20260321_051543/`

best checkpoint:

- `outputs/train/pilotnet_cmd_20260321_051543/best.pt`

best summary:

- best epoch: `8`
- best val loss: `0.013738`
- device: `cuda`
- split mode: `episode`
- command rebalancing: `enabled`

### Baseline D: Camera + Speed + Command + Junction Pack, No Rebalance

train output:

- `outputs/train/pilotnet_cmd_20260321_051903/`

best checkpoint:

- `outputs/train/pilotnet_cmd_20260321_051903/best.pt`

best summary:

- best epoch: `6`
- best val loss: `0.011287`
- device: `cuda`
- split mode: `episode`
- command rebalancing: `disabled`

## Closed-Loop Experiment

### Experiment A: Camera + Speed

evaluation output:

- `outputs/evaluate/town01_pilotnet_loop_pilotnet_eval_20260321_032151/summary.json`
- `outputs/evaluate/town01_pilotnet_loop_pilotnet_eval_20260321_032151/front_rgb.mp4`

結果:

- `policy_type = learned_lateral_policy`
- `is_camera_e2e_policy = true`
- `elapsed_seconds = 600.0`
- `collision_count = 0`
- `lane_invasion_count = 72`
- `route_completion_ratio = 0.1547`
- `failure_reason = max_seconds_exceeded`

### Experiment B: Camera + Speed + Command

evaluation output:

- `outputs/evaluate/town01_pilotnet_loop_pilotnet_eval_20260321_045939/summary.json`
- `outputs/evaluate/town01_pilotnet_loop_pilotnet_eval_20260321_045939/front_rgb.mp4`

結果:

- `policy_type = learned_lateral_policy`
- `model_name = pilotnet_commanded`
- `is_camera_e2e_policy = true`
- `elapsed_seconds = 41.5`
- `collision_count = 1`
- `lane_invasion_count = 7`
- `route_completion_ratio = 0.0797`
- `failure_reason = collision`

### Experiment C: Junction Pack + Rebalanced Command Model

evaluation output:

- `outputs/evaluate/town01_pilotnet_loop_pilotnet_eval_20260321_051826/summary.json`
- `outputs/evaluate/town01_pilotnet_loop_pilotnet_eval_20260321_051826/front_rgb.mp4`

結果:

- `policy_type = learned_lateral_policy`
- `model_name = pilotnet_commanded`
- `elapsed_seconds = 41.05`
- `collision_count = 2`
- `lane_invasion_count = 5`
- `route_completion_ratio = 0.0788`
- `failure_reason = collision`

### Experiment D: Junction Pack + No-Rebalance Command Model

evaluation output:

- `outputs/evaluate/town01_pilotnet_loop_pilotnet_eval_20260321_052143/summary.json`
- `outputs/evaluate/town01_pilotnet_loop_pilotnet_eval_20260321_052143/front_rgb.mp4`

結果:

- `policy_type = learned_lateral_policy`
- `model_name = pilotnet_commanded`
- `elapsed_seconds = 41.5`
- `collision_count = 1`
- `lane_invasion_count = 7`
- `route_completion_ratio = 0.0797`
- `failure_reason = collision`

## Interpretation

この時点では、**追加収集なしでも、`right/straight` を増やしたあとでも fixed loop 完走には届いていない**。

観察:

- 最初の実装では evaluator の camera 解像度が学習時とずれていて、`7.45 s` で collision した
- 学習時と同じ `320x180` にそろえると、`600 s` ノーコリジョンまでは走れた
- ただし route completion は `15.47%` で止まり、planner が期待する loop には乗り切れていない
- `command` を入れた再学習は val loss では大きく悪化していない
- それでも closed-loop では `15.47% -> 7.97%` に下がり、`41.5 s` で collision した
- つまり、このデータ量と分布では `command` を入れるだけでは十分ではない
- `right/straight` を増やしたあとでも closed-loop は `0.0788-0.0797` に張り付いた
- `rebalance_commands` を切っても結果はほぼ同じで、重み付けだけが原因ではない

一番自然な解釈:

- camera-only baseline の失敗要因はやはり junction の route intent 不足だった
- ただし `right` と `straight` を増やしただけでは足りず、`left` の不足か、単発 frame-based architecture 自体の限界が残っている
- `split_mode = episode` と `rebalance off` でも fixed-loop completion は改善しない
- open-loop の val loss は下がっても、closed-loop の改善を保証しない

## Next

- `front RGB + speed + command -> steer` は維持するが、次は model だけをいじらない
- まず evaluator で collision 位置と最初の failure 区間を可視化する
- そのうえで `left` junction 例を追加するか、temporal context を入れる
- 再学習後の主評価指標は引き続き closed-loop `route_completion_ratio`
