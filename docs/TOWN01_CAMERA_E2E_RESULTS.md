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

## Interpretation

この時点では、**追加収集なしで command conditioning を足しても fixed loop 完走には届かなかった**。

観察:

- 最初の実装では evaluator の camera 解像度が学習時とずれていて、`7.45 s` で collision した
- 学習時と同じ `320x180` にそろえると、`600 s` ノーコリジョンまでは走れた
- ただし route completion は `15.47%` で止まり、planner が期待する loop には乗り切れていない
- `command` を入れた再学習は val loss では大きく悪化していない
- それでも closed-loop では `15.47% -> 7.97%` に下がり、`41.5 s` で collision した
- つまり、このデータ量と分布では `command` を入れるだけでは十分ではない

一番自然な解釈:

- camera-only baseline の失敗要因はやはり junction の route intent 不足だった
- ただし `command` を architecture に足しても、`right` と `straight` の教師例が少なすぎて分岐で安定しない
- frame 単位のランダム split で見える val loss は、closed-loop の改善を保証しない

## Next

- `front RGB + speed + command -> steer` はこのまま維持する
- 追加収集は `right` と `straight` の junction 例を優先する
- 最低ラインとして `right` を `219 -> 1000-1500 frames` に増やす
- 再学習後の主評価指標は val loss ではなく closed-loop `route_completion_ratio`
- 同一路線で改善しなければ、次に evaluator と route design を見直す
