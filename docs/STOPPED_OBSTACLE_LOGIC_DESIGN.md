# Stopped Obstacle Logic Design

`Town01` の停止障害物回避を、`CARLA helper` 依存から切り離して実装するための設計書です。

この文書の目的:

- 停止障害物回避を `pure logic` と `CARLA adapter` に分離する
- `BasicAgent._generate_lane_change_path()` に依存しない route-aligned overtake を定義する
- `clear / blocked_static / blocked_oncoming` を同じ状態機械で扱えるようにする
- 複数停止車両を扱っても state machine 自体は単純なまま保つ

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

追加方針:

- 複数停止車両を個別 target の無制限列として扱わない
- adapter 側で `single actor` または `obstacle cluster` に正規化する
- pure logic は常に `1つの active overtake target` だけを扱う

これにより、複数車両の同時追い越しを可能にしつつ、runtime の状態機械は増やさない。

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
- 正規化済み `single actor / obstacle cluster` target を共通 contract で扱う

#### CARLA adapter

`SceneState` や route trace から、pure logic が必要な snapshot を組み立てる。

責務:

- `SceneState` から lead / blocker / lane gap を拾う
- same-lane stopped vehicles を `single actor` または `obstacle cluster` にまとめる
- route trace の future points を lane-aligned samples に変換する
- pure logic の結果を `ExpertBasicAgent` の control / target waypoint に反映する

#### integration runtime

`run_route_loop.py` / `ad_stack.run` / environment / MCAP。

責務:

- scenario 実行
- telemetry 保存
- summary / manifest 出力

## 3. 予定する pure module

pure logic の正本:

- `ad_stack/overtake/domain/`
- `ad_stack/overtake/application/`
- `ad_stack/overtake/policies/stopped_target_policy.py`

### 3.1 主要 dataclass

#### `OvertakeLeadSnapshot`

- `actor_id`
- `lane_id`
- `distance_m`
- `speed_mps`
- `relative_speed_mps`
- `is_stopped`

#### `StoppedObstacleClusterSnapshot`

- `primary_actor_id`
- `member_actor_ids`
- `lane_id`
- `entry_distance_m`
- `exit_distance_m`
- `max_member_speed_mps`
- `all_members_stopped`

定義:

- `entry_distance_m`
  - route 進行方向で見た cluster 先頭の進入点
- `exit_distance_m`
  - route 進行方向で見た cluster 末尾の離脱点

cluster を 1 つの corridor として扱うとき、pass 完了は actor 切替ではなく `exit_distance_m` 基準で判定する。

#### `OvertakeTargetSnapshot`

- `kind`
  - `single_actor`
  - `cluster`
- `primary_actor_id`
- `member_actor_ids`
- `lane_id`
- `entry_distance_m`
- `exit_distance_m`
- `speed_mps`
- `is_stopped`

設計原則:

- pure logic に渡す active target は常に 1 つだけ
- 近接した複数停止車両は `cluster` として 1 つにまとめる
- 十分に離れた停止車両は別イベントとして扱い、1 台目の rejoin 後に 2 台目を再取得する

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

#### `OvertakeContext`

- `timestamp_s`
- `current_lane_id`
- `origin_lane_id`
- `route_target_lane_id`
- `target_speed_kmh`
- `stopped_speed_threshold_mps`
- `lead`
- `active_target`
- `left_lane`
- `right_lane`
- `active_signal_state`
- `signal_stop_distance_m`
- `allow_overtake`
- `preferred_direction`

注記:

- `lead` は raw telemetry / debug 補助として残してよい
- pure logic の判定入力としては `active_target` を正本にする

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
- `target_kind`
- `target_member_actor_ids`
- `target_exit_distance_m`
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

### 4.2.1 single actor と cluster の扱い

trigger / reject の入口は共通にする。

- `single_actor`
  - `entry_distance_m = lead.distance_m`
  - `exit_distance_m` は actor 後端ベース
- `cluster`
  - `entry_distance_m` は最も近い停止車両の前端
  - `exit_distance_m` は最も遠い停止車両の後端

複数車両を同時に追い越す必要があるときでも、pure logic は `entry / exit` を持つ 1 つの corridor だけを扱う。

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

- `single_actor` のとき:
  - target actor が ego より後方に回った
  - かつ `distance_past_target_m >= overtake_resume_front_gap_m`
- `cluster` のとき:
  - cluster の `exit_distance_m` が ego より後方に回った
  - かつ `distance_past_target_m >= overtake_resume_front_gap_m`

ここで `distance_past_target_m` は、ego の進行方向に対する `-target_exit_distance_m` を使う。

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

`cluster` では `primary_actor_id` が一時的に見えなくなっても、cluster 内の別 member が見えていれば corridor 自体は visible とみなしてよい。

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

## 6. 複数停止車両の設計

### 6.1 基本方針

複数停止車両を扱うときも、code path は次の 2 形態だけに限定する。

- `separated double obstacle`
  - 1 台目と 2 台目の gap が十分大きい
  - 1 台目を抜いたら origin lane に戻り、2 台目を別イベントとして再取得する
- `clustered stopped obstacles`
  - 車間が小さく、途中 rejoin しないほうが自然
  - 複数停止車両を 1 つの `obstacle cluster` として同時に追い越す

これ以上の一般化、たとえば「任意個の actor を順次 state machine が持つ」ことはやらない。

### 6.2 cluster 化ルール

同一 lane 上の停止車両を route 進行方向で並べ、隣接車両の gap が `cluster_merge_gap_m` 以下なら同一 cluster にまとめる。

必要な config:

- `cluster_merge_gap_m`
- `cluster_max_member_speed_mps`

期待値:

- gap が閾値以下なら simultaneous overtake
- gap が閾値より大きければ separated handling

### 6.3 acceptance contract

#### separated

- 1 台目を抜いたら `lane_change_back` へ入る
- origin lane へ戻ったあと、2 台目を新しい `single_actor` として取得する
- `lead_vehicle_id` / `overtake_target_actor_id` は actor 切替が見える

#### clustered

- `pass_vehicle` を cluster 全体に対して維持する
- cluster の途中で不用意に rejoin しない
- actor id の切替自体は必須ではない
- 必須なのは `member_actor_ids` が安定し、cluster tail を抜いたあとにだけ rejoin すること

## 7. preflight validation

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
- `double_stopped_separated`
- `double_stopped_clustered`

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

## 8. telemetry / manifest に追加するもの

### 7.1 `/ego/planning_debug`

- `overtake_target_actor_id`
- `overtake_target_kind`
- `overtake_target_member_actor_ids`
- `overtake_origin_lane_id`
- `target_passed`
- `distance_past_target_m`
- `rejoin_front_gap_m`
- `rejoin_rear_gap_m`
- `target_actor_visible`
- `target_actor_last_seen_s`
- `lane_change_path_available`
- `lane_change_path_failed_reason`

### 7.2 `/world/tracked_vehicles`

最低限このままで足りるが、必要なら

- `length_m`
- `width_m`

も追加余地あり。

### 7.3 summary

- `overtake_target_actor_id`
- `first_overtake_attempt_s`
- `first_target_passed_s`
- `first_rejoin_started_s`

## 9. 実装順

1. pure logic module を新設する
2. unit test を先に書く
3. `ExpertBasicAgent` から trigger / pass / rejoin の判定を pure logic へ寄せる
4. route-aligned lane-change plan を pure logic で生成する
5. preflight validation を `run.py` に差し込む
6. `single_actor` baseline を `clear / blocked_static / blocked_oncoming` で CARLA 結合テストする
7. `double_stopped_separated` を追加する
8. `double_stopped_clustered` を追加する

## 10. 非目標

この段階ではまだやらない。

- 複雑な junction 内での停止障害物回避
- signal 付き交差点直前での対向 lane 追い越し
- VLA teacher としての最終 polish
- 任意個の移動障害物を一般グラフ探索で追い越すこと
