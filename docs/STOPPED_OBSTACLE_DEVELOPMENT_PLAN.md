# Stopped Obstacle Development Plan

`Town01` 上で停止障害物の回避を開発するための、実装順と確認項目のメモです。

目的:

- 停止車両の回避を `deterministic` な `CARLA` scenario で繰り返し開発できるようにする
- `MCAP` / manifest / summary から、失敗理由と判断過程を追えるようにする
- 最初は単純な直線・見通し良好ケースだけで成立させる

関連資料:

- [VLA_SCENARIO_SCOPE.md](/home/masa/carla_alpamayo/docs/VLA_SCENARIO_SCOPE.md)
- [OVERTAKE_EXPERT_REQUIREMENTS.md](/home/masa/carla_alpamayo/docs/OVERTAKE_EXPERT_REQUIREMENTS.md)
- [TOWN01_OVERTAKE_PARAMETER_DECISIONS.md](/home/masa/carla_alpamayo/docs/TOWN01_OVERTAKE_PARAMETER_DECISIONS.md)

## 1. まず成立させるシナリオ

最初に対象とするのは、次の 2 本です。

- `clear`:
  - ego 前方の同一 lane 上に停止車両が 1 台いる
  - 反対 lane は空いている
  - expert は lane change で回避して元 lane に戻る
- `blocked`:
  - ego 前方の同一 lane 上に停止車両が 1 台いる
  - 反対 lane に接近車両がいる
  - expert は無理に出ず、停止して待つ

対象 route:

- [town01_road12_overtake_short.json](/home/masa/carla_alpamayo/scenarios/routes/town01_road12_overtake_short.json)
- 必要なら北向き版も追加:
  - [town01_northbound_overtake_short.json](/home/masa/carla_alpamayo/scenarios/routes/town01_northbound_overtake_short.json)

## 2. 環境として必要なもの

### 2.1 deterministic environment

停止障害物開発用 environment は、少なくとも次を固定する。

- ego route
- 停止障害物の spawn index
- 対向 blocker の spawn index
- weather
- max seconds
- traffic light condition

現時点の候補:

- [town01_stopped_obstacle_clear_45s.json](/home/masa/carla_alpamayo/scenarios/environments/town01_stopped_obstacle_clear_45s.json)
- [town01_stopped_obstacle_blocked_45s.json](/home/masa/carla_alpamayo/scenarios/environments/town01_stopped_obstacle_blocked_45s.json)

### 2.2 stopped obstacle profile

停止障害物は `autopilot off` で固定し、速度 `0 km/h` を保つ。

- [stopped_obstacle_profile_v1.json](/home/masa/carla_alpamayo/scenarios/npc_profiles/stopped_obstacle_profile_v1.json)

### 2.3 run-config

シミュレーション実行は shell wrapper ではなく JSON run-config で固定する。

必要な run-config:

- `town01_stopped_obstacle_clear_expert`
- `town01_stopped_obstacle_blocked_expert`

含めるべき設定:

- `policy.kind = expert`
- `record_mcap = true`
- `record_video = false`
- `mcap_segment_seconds`
- `ignore_traffic_lights = false`
- `ignore_vehicles = false`

## 3. MCAP に残したい情報

停止障害物回避のデバッグで最低限必要なのは、次の 4 層です。

### 3.1 ego の状態

- `/ego/state`
- `/ego/control`
- `/ego/planning`
- `/ego/planning_debug`

確認したいもの:

- 現在速度
- target speed
- planner state
- overtake state
- target lane
- emergency stop
- reject reason

### 3.2 ego から見た周辺車両

- `/world/tracked_vehicles`

確認したいもの:

- actor id
- same / left / right lane relation
- lane id
- longitudinal distance
- lateral distance
- relative speed
- ahead / behind

### 3.3 world 上の NPC 状態

- `/world/npc_vehicles`
- `/world/npc_markers`

確認したいもの:

- 停止障害物が本当に ego lane 上にいるか
- blocker が対向 lane を走っているか
- pose / size / lane id

### 3.4 traffic light / route 文脈

- `/world/traffic_lights`
- `/map/scene`
- `/ego/marker`

確認したいもの:

- signal が suppression 要因になっていないか
- route がどこを通る想定か

## 4. manifest / summary に欲しい情報

後で grep しやすいように、manifest / summary でも次を見たい。

- `lead_vehicle_id`
- `lead_vehicle_distance_m`
- `lead_vehicle_speed_mps`
- `lead_vehicle_relative_speed_mps`
- `current_lane_id`
- `route_target_lane_id`
- `overtake_direction`
- `overtake_target_lane_id`
- `overtake_reject_reason`
- `traffic_light_actor_id`
- `traffic_light_distance_m`
- `traffic_light_stop_line_distance_m`
- `min_ttc`

## 5. シミュレーションで確認したいこと

停止障害物回避で最初に確認したいのは次の 3 つです。

1. 停止障害物が本当に ego の進路上にいるか
2. 回避開始距離と lane change 開始タイミングが妥当か
3. 回避後に元 lane に自然に復帰できるか

`blocked` シナリオでは追加で次も見る。

4. 反対 lane blocker を見て reject できるか
5. 危険時に無理な lane change を始めないか

## 6. 実装順

順番は固定する。

### P0: 環境と計測を固める

- stopped-obstacle run-config を作る
- `MCAP` の debug topic を配線する
- manifest / summary に必要な field を揃える
- clear / blocked の smoke test を通す

### P1: clear シナリオを成立させる

- 停止障害物に対して回避開始できる
- lane change out できる
- 安全に追い抜ける
- 元 lane に戻れる

### P2: blocked シナリオを成立させる

- blocker を見て overtake reject する
- 停止して待つ
- blocker 通過後に安全なら再開する

### P3: パラメータを詰める

- trigger distance
- front / rear gap
- resume gap
- reject reason
- target speed / hold distance

## 7. 受け入れ基準

### AC-1 clear

- 停止障害物を回避して route を継続できる
- collision 0
- unsafe lane change 0
- lane change 後に元 lane へ戻る

### AC-2 blocked

- 停止障害物がいても、危険な対向車がある間は出ない
- collision 0
- 無理な overtake attempt 0

### AC-3 logs

- MCAP だけ見て、停止障害物回避の判断過程を追える
- manifest だけ見ても、accept / reject の理由を grep できる

## 8. 直近でやること

- [ ] stopped-obstacle 用 run-config を追加する
- [ ] `/world/tracked_vehicles` を runtime から `MCAP` へ配線する
- [ ] `/world/traffic_lights` を runtime から `MCAP` へ配線する
- [ ] planning_debug の完全重複 field を整理する
- [ ] clear / blocked を通常 entrypoint で smoke test する
- [ ] その結果を見て、停止障害物の追い越しロジックを詰める
