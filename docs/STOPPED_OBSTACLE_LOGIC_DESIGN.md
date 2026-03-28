# Stopped Obstacle Logic Design

`Town01` の停止障害物回避を、`CARLA helper` 依存から切り離して実装するための設計書です。

この文書の目的:

- 停止障害物回避を `pure logic` と `CARLA adapter` に分離する
- `BasicAgent._generate_lane_change_path()` に依存しない route-aligned overtake を定義する
- `clear / blocked_static / blocked_oncoming` を同じ状態機械で扱えるようにする

関連資料:

- [STOPPED_OBSTACLE_DEVELOPMENT_PLAN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_DEVELOPMENT_PLAN.md)
- [OVERTAKE_EXPERT_REQUIREMENTS.md](/home/masa/carla_alpamayo/docs/OVERTAKE_EXPERT_REQUIREMENTS.md)
- [TOWN01_OVERTAKE_PARAMETER_DECISIONS.md](/home/masa/carla_alpamayo/docs/TOWN01_OVERTAKE_PARAMETER_DECISIONS.md)

## 1. 背景と現状の問題

現行の停止障害物回避は、[expert_basic_agent.py](/home/masa/carla_alpamayo/ad_stack/agents/expert_basic_agent.py) の `_start_overtake()` で `CARLA BasicAgent._generate_lane_change_path()` を使っている。

この helper は

- 自レーン変更
- 同方向 lane の lane change

には便利だが、停止障害物回避のような

- route は北向きに継続したい
- 一時的に opposite lane を使う

ケースには向かない。

問題:

- opposite lane の `next()` は opposite lane の進行方向に従う
- そのため lane change 後の plan が route 進行方向と逆を向く
- fixed hold distance で pass 判定しているので、`target actor を本当に抜いたか` を見ていない
- rejoin 開始が actor 基準ではなく距離基準なので、static hazard や vegetation に刺さりやすい

結論:

- lane-change path 生成
- pass 完了判定
- rejoin 判定

は `pure logic` として自前で持つ。

## 2. 実装方針

### 2.1 分離したい層

#### pure logic

`CARLA` 型を持たない。入力は snapshot / scalar のみ。

責務:

- trigger / reject 判定
- overtake state machine
- pass 完了判定
- rejoin 可否判定
- route-aligned lane-change plan の生成
- preflight validation

#### CARLA adapter

`SceneState` や route trace から、pure logic が必要な snapshot を組み立てる。

責務:

- `SceneState` から lead / blocker / lane gap を拾う
- route trace の future points を lane-aligned samples に変換する
- pure logic の結果を `ExpertBasicAgent` の control / target waypoint に反映する

#### integration runtime

`run_route_loop.py` / `ad_stack.run` / environment / MCAP。

責務:

- scenario 実行
- telemetry 保存
- summary / manifest 出力

## 3. 予定する pure module

新設候補:

- `ad_stack/overtake/stopped_obstacle_logic.py`

### 3.1 主要 dataclass

#### `OvertakeLeadSnapshot`

- `actor_id`
- `lane_id`
- `distance_m`
- `speed_mps`
- `relative_speed_mps`
- `is_stopped`

#### `AdjacentLaneGapSnapshot`

- `lane_id`
- `front_gap_m`
- `rear_gap_m`
- `lane_open`

#### `RouteLaneSample`

- `route_index`
- `center_x_m`
- `center_y_m`
- `lane_id`
- `progress_m`

#### `StoppedObstacleContext`

- `timestamp_s`
- `current_lane_id`
- `origin_lane_id`
- `route_target_lane_id`
- `target_speed_kmh`
- `stopped_speed_threshold_mps`
- `lead`
- `left_lane`
- `right_lane`
- `active_signal_state`
- `signal_stop_distance_m`
- `allow_overtake`
- `preferred_direction`

#### `OvertakeMemory`

- `state`
- `direction`
- `origin_lane_id`
- `target_lane_id`
- `target_actor_id`
- `target_actor_lane_id`
- `target_actor_last_seen_s`
- `target_actor_last_seen_longitudinal_m`
- `target_actor_visibility_timeout_s`
- `pass_started_s`
- `pass_started_route_index`
- `target_passed`
- `target_pass_distance_m`
- `abort_reason`

#### `OvertakeDecision`

- `planner_state`
- `target_speed_kmh`
- `target_lane_id`
- `command_kind`
  - `follow_route`
  - `follow_target_lane`
  - `begin_rejoin`
  - `abort_rejoin`
  - `reject`
- `reject_reason`
- `debug`

## 4. 状態機械

状態は次で固定する。

- `idle`
- `car_follow`
- `lane_change_out`
- `pass_vehicle`
- `lane_change_back`
- `abort_return`

### 4.1 `idle`

開始状態。

遷移:

- lead がいなければ `nominal_cruise`
- stopped obstacle が条件を満たせば `lane_change_out`
- 条件を満たさなければ `car_follow`

### 4.2 `car_follow`

停止障害物を認識しているが、まだ追い越しを開始しない状態。

reject reason は必ず 1 つに落とす。

候補:

- `overtake_disabled`
- `lead_distance_unavailable`
- `lead_out_of_range`
- `lead_not_slow_enough`
- `adjacent_lane_closed`
- `adjacent_front_gap_insufficient`
- `adjacent_rear_gap_insufficient`
- `signal_suppressed`

### 4.3 `lane_change_out`

目的:

- origin lane から target lane へ route 進行方向を保ったまま移る

開始条件:

- stopped lead が trigger 内
- chosen adjacent lane の front / rear gap が十分
- signal suppression がない

完了条件:

- current lane id が target lane id と一致

失敗時:

- target lane が失われた
- signal suppression が入った
- target lane gap が急変した

なら `abort_return`

target lane の front / rear gap は、`left/right relation` ではなく

- `target lane id`

基準で見る。lane change 後は relation 名が入れ替わる可能性があるため。

### 4.4 `pass_vehicle`

目的:

- target actor を抜き切る

pass 完了は固定 hold distance ではなく actor 基準で決める。

`target_passed = true` の条件:

- target actor が ego より後方に回った
- かつ `distance_past_target_m >= overtake_resume_front_gap_m`

ここで `distance_past_target_m` は、ego の進行方向に対する `-target.longitudinal_distance_m` を使う。

#### target actor visibility contract

target actor が一時的に tracked object から消えることはありうる。

このときは即 `passed` にしない。

最低限:

- `target_actor_last_seen_s`
- `target_actor_last_seen_longitudinal_m`

を保持し、

- 一定時間だけ last seen を信頼する
- それを超えても target actor が復元できない場合は `abort_return` ではなく `pass_vehicle` 維持

とする。

ここで「一定時間」は config で持つ。

- `target_actor_visibility_timeout_s`

timeout を超えたら

- `target_actor_visible = false`
- `target actor state unknown`

として扱い、`passed = true` は立てない。

### 4.5 `lane_change_back`

開始条件:

- `target_passed = true`
- origin lane の rejoin front / rear gap が十分

完了条件:

- current lane id が origin lane id に戻る

### 4.5.1 speed contract

速度方針が曖昧だとテスト観点がぶれるので、state ごとに上限を持つ。

- `lane_change_out`
  - `max(target_speed_kmh, lead_speed + delta)` を上限とするが急加速しすぎない
- `pass_vehicle`
  - pass 完了まで target speed を維持
- `lane_change_back`
  - rejoin 中は保守的に target speed を落としてもよい
- `abort_return`
  - rejoin 優先で target speed を抑える

### 4.6 `abort_return`

rejoin はしたいが、安全性の都合で通常 path をやめる状態。

発火条件:

- signal suppression
- hazard emergence
- lane continuity break

## 5. route-aligned lane-change plan

### 5.1 必要な性質

生成される plan は次を満たす。

- route 進行方向に対して `progress_m` が単調増加
- lane change 前後で `target lane` の切替だけが起こる
- opposite lane に出ても `opposite lane の native direction` ではなく `route direction` を維持する

### 5.2 plan 生成方式

入力:

- future `RouteLaneSample[]` on origin lane
- desired direction
- `distance_same_lane`
- `lane_change_distance`
- `distance_other_lane`

生成:

1. origin lane 上を `distance_same_lane` だけ進む
2. その後の future sample について `adjacent lane sample` へ投影する
3. `target lane` 上を `distance_other_lane` だけ route-aligned に追従する

重要:

- `target lane` 上で `next()` を呼んで lane の native direction に従わない
- 常に `origin route sample` を基準に、同一 progress 上の adjacent lane sample を使う

### 5.3 lane selection contract

両側 lane が開いているケースでは、選好を曖昧にしない。

- `preferred_direction = left_first` なら left を優先
- `preferred_direction = right_first` なら right を優先
- ただし left が不成立で right が成立なら right を使う
- 両方不成立なら reject する

この tie-break は pure logic に置く。

## 6. preflight validation

run 前に scenario を検証し、結果を summary に残す。

### 6.1 出したい項目

- `ego_spawn_lane_id`
- `obstacle_lane_id`
- `blocker_lane_id`
- `ego_to_obstacle_longitudinal_distance_m`
- `ego_to_blocker_longitudinal_distance_m`
- `left_lane_is_driving`
- `right_lane_is_driving`
- `nearest_signal_distance_m`
- `nearest_junction_distance_m`
- `route_aligned_adjacent_lane_available`
- `scenario_kind`
- `validation_errors`
- `validation_warnings`

`scenario_kind` は少なくとも次に分ける。

- `clear`
- `blocked_static`
- `blocked_oncoming`

runtime 中にこの区別が失われないようにする。

### 6.2 clear の最低条件

- obstacle が ego と同一 lane
- obstacle が ego 前方にいる
- chosen overtake lane が `Driving`
- route 上で rejoin 先 lane が一致
- 近傍に signal / junction がない
- route corridor 上に known static hazard がない

### 6.3 blocked_static

- blocker が opposite lane 上にいる
- blocker が ego の overtake corridor を塞ぐ位置にいる

### 6.4 blocked_oncoming

- moving blocker が opposite lane から接近する
- front gap insufficiency で reject される

## 7. telemetry / manifest に追加するもの

### 7.1 `/ego/planning_debug`

- `overtake_target_actor_id`
- `overtake_origin_lane_id`
- `target_passed`
- `distance_past_target_m`
- `rejoin_front_gap_m`
- `rejoin_rear_gap_m`
- `target_actor_visible`
- `target_actor_last_seen_s`
- `lane_change_path_available`
- `lane_change_path_failed_reason`
- `scenario_validation_errors`

### 7.2 `/world/tracked_vehicles`

最低限このままで足りるが、必要なら

- `length_m`
- `width_m`

も追加余地あり。

### 7.3 summary

- `scenario_validation`
- `overtake_target_actor_id`
- `first_overtake_attempt_s`
- `first_target_passed_s`
- `first_rejoin_started_s`

## 8. 実装順

1. pure logic module を新設する
2. unit test を先に書く
3. `ExpertBasicAgent` から trigger / pass / rejoin の判定を pure logic へ寄せる
4. route-aligned lane-change plan を pure logic で生成する
5. preflight validation を `run.py` に差し込む
6. その後で `clear / blocked_static / blocked_oncoming` を CARLA で結合テストする

## 9. 非目標

この段階ではまだやらない。

- 複雑な junction 内での停止障害物回避
- 複数障害物の連続回避
- signal 付き交差点直前での対向 lane 追い越し
- VLA teacher としての最終 polish
