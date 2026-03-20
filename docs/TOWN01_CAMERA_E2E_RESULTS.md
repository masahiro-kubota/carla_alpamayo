# Town01 Camera E2E Results

2026-03-21 時点の `PilotNet風 + speed -> steer` 実験メモ。

## Setup

- task: `front RGB + speed -> steer`
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

train output:

- `outputs/train/pilotnet_20260321_031957/`

best checkpoint:

- `outputs/train/pilotnet_20260321_031957/best.pt`

best summary:

- best epoch: `7`
- best val loss: `0.015362`
- device: `cuda`

## Closed-Loop Experiment

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

## Interpretation

この first baseline では、**camera + speed だけでは fixed loop を 1 周するには不十分**だった。

観察:

- 最初の実装では evaluator の camera 解像度が学習時とずれていて、`7.45 s` で collision した
- 学習時と同じ `320x180` にそろえると、`600 s` ノーコリジョンまでは走れた
- ただし route completion は `15.47%` で止まり、planner が期待する loop には乗り切れていない

一番自然な解釈:

- junction 付近で **route intent が画像だけでは足りない**
- 以前に想定していた通り、次段階で `command` を入れる必要がある

## Next

- `front RGB + speed + command -> steer` に拡張する
- まずは same route で closed-loop completion を上げる
- その後に `Town03` や unseen route へ広げる
