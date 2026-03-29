# Stopped Obstacle Test Design

`Stopped Obstacle Logic Design` に対するテスト設計書です。

目的:

- 停止障害物回避を `CARLA なしの unit test` でできるだけ詰める
- `CARLA` は最後の結合テストだけに使う
- 失敗時に「pure logic の問題か / adapter の問題か / scenario の問題か」を切り分けられるようにする

関連資料:

- [STOPPED_OBSTACLE_LOGIC_DESIGN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_LOGIC_DESIGN.md)
- [STOPPED_OBSTACLE_DEVELOPMENT_PLAN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_DEVELOPMENT_PLAN.md)
- [STOPPED_OBSTACLE_OPPOSITE_LANE_OCCUPANCY_TEST_DESIGN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_OPPOSITE_LANE_OCCUPANCY_TEST_DESIGN.md)

## 1. テスト方針

3 層に分ける。

### 1.1 pure unit test

`CARLA` を import しない。

対象:

- trigger / reject 判定
- pass 完了判定
- rejoin 判定
- route-aligned lane-change plan の単調性
- scenario preflight validation の contract 判定
- target actor visibility の扱い
- `single_actor / obstacle_cluster` の切替

ここが主戦場。

### 1.2 adapter test

`CARLA` に近い shape の入力を、pure logic へ正しく変換できるかを見る。

対象:

- `SceneState -> OvertakeContext`
- route trace -> `RouteLaneSample[]`
- tracked object -> lead / blocker / cluster / rejoin gap 変換

ここでは `fake snapshot` を使ってもよい。

### 1.3 integration test

最後に `run_route_loop.py` で回す。

対象:

- `clear`
- `blocked_static`
- `blocked_static_near`
- `blocked_oncoming`
- `double_stopped_separated`
- `double_stopped_clustered`

ただし gate は段階化する。

- baseline は `clear / blocked_static / blocked_oncoming`
- `blocked_static_near` は自然な近距離 negative case としてその後に追加する
- 複数停止車両はその後に `separated -> clustered` の順で追加する

## 2. pure unit test で必ず持つケース

### 2.1 trigger / reject

#### `test_overtake_rejects_when_lead_out_of_range`

- lead distance > trigger
- 期待: `car_follow`, reject=`lead_out_of_range`

#### `test_overtake_rejects_when_adjacent_front_gap_insufficient`

- opposite lane front gap < threshold
- 期待: reject=`adjacent_front_gap_insufficient`

#### `test_overtake_rejects_when_adjacent_rear_gap_insufficient`

- opposite lane rear gap < threshold
- 期待: reject=`adjacent_rear_gap_insufficient`

#### `test_overtake_rejects_when_signal_suppressed`

- red/yellow light within suppression distance
- 期待: reject=`signal_suppressed`

#### `test_overtake_accepts_clear_case`

- stopped obstacle ahead
- target lane open
- front / rear gap sufficient
- 期待: `lane_change_out`

#### `test_overtake_prefers_left_when_both_sides_are_open`

- left / right とも成立
- `preferred_direction = left_first`
- 期待: left を選ぶ

#### `test_overtake_prefers_right_when_both_sides_are_open_and_right_first`

- left / right とも成立
- `preferred_direction = right_first`
- 期待: right を選ぶ

### 2.2 pass completion

#### `test_pass_not_complete_while_target_still_ahead`

- target longitudinal distance positive
- 期待: `target_passed = false`

#### `test_pass_not_complete_until_resume_gap_is_met`

- target behind but `distance_past_target_m < resume_gap`
- 期待: `target_passed = false`

#### `test_pass_complete_once_target_is_behind_and_resume_gap_met`

- target behind
- `distance_past_target_m >= resume_gap`
- 期待: `target_passed = true`

#### `test_pass_does_not_complete_when_target_actor_temporarily_disappears`

- 直前まで target actor が見えていた
- current frame では target actor が欠落
- 期待: `target_passed = false`

#### `test_pass_does_not_complete_after_visibility_timeout_without_actor_reacquisition`

- target actor が timeout を超えて見えない
- 期待: `target_passed = false`
- 期待: state は `pass_vehicle` 維持

#### `test_cluster_pass_does_not_complete_until_cluster_exit_gap_is_met`

- `target_kind = cluster`
- 先頭車は抜いたが cluster tail はまだ前方
- 期待: `target_passed = false`

#### `test_cluster_pass_completes_once_cluster_exit_is_behind_and_resume_gap_is_met`

- `target_kind = cluster`
- cluster tail が ego 後方
- `distance_past_target_m >= resume_gap`
- 期待: `target_passed = true`

### 2.3 rejoin decision

#### `test_rejoin_rejects_when_origin_front_gap_insufficient`

- pass 済み
- origin lane front gap 不足
- 期待: `pass_vehicle` 継続

#### `test_rejoin_rejects_when_origin_rear_gap_insufficient`

- pass 済み
- origin lane rear gap 不足
- 期待: `pass_vehicle` 継続

#### `test_rejoin_begins_when_passed_and_origin_gaps_are_safe`

- pass 済み
- origin lane front / rear gap sufficient
- 期待: `lane_change_back`

#### `test_rejoin_gap_uses_origin_lane_id_not_relation_name`

- ego が opposite lane にいる
- relation label が入れ替わる
- 期待: origin lane の gap が lane id 基準で正しく取れる

#### `test_separated_double_obstacle_rejoins_then_reacquires_second_target`

- 1 台目と 2 台目の gap が `cluster_merge_gap_m` より大きい
- 1 台目 pass 完了後は `lane_change_back`
- origin lane 復帰後に 2 台目を新しい target として再取得

#### `test_clustered_double_obstacle_keeps_pass_vehicle_until_cluster_tail_is_cleared`

- 2 台が `cluster_merge_gap_m` 以下
- `target_kind = cluster`
- 途中 rejoin せず `pass_vehicle` を維持

### 2.4 abort path

#### `test_abort_return_when_signal_appears_during_overtake`

- `lane_change_out` or `pass_vehicle` 中
- signal suppression が入る
- 期待: `abort_return`

#### `test_abort_return_when_target_lane_becomes_invalid`

- target lane 消失 or invalid
- 期待: `abort_return`

#### `test_abort_return_when_route_aligned_adjacent_lane_sample_is_missing`

- route progress 上で target lane sample が取れない
- 期待: `abort_return` or plan unavailable

## 3. route-aligned plan の unit test

ここは今回の本丸。

### 3.1 単調性

#### `test_lane_change_plan_progress_is_monotonic`

- route sample の `progress_m` が単調増加する入力
- 期待: 出力 plan も単調増加

### 3.2 lane identity

#### `test_lane_change_plan_starts_on_origin_lane`

- 期待: plan 先頭は origin lane

#### `test_lane_change_plan_transitions_to_target_lane`

- 期待: 一定位置以降は target lane

#### `test_rejoin_plan_returns_to_origin_lane`

- 期待: rejoin plan 最後は origin lane

### 3.3 opposite lane native direction を使わないこと

#### `test_lane_change_plan_does_not_reverse_when_target_lane_native_direction_is_opposite`

- opposite lane の yaw が route と逆でも
- route progress は増え続ける
- 期待: southbound lane を northbound route 用に使っても plan が逆走しない

#### `test_lane_change_plan_marks_failure_when_adjacent_lane_sample_is_missing`

- route の途中で adjacent lane sample が切れる
- 期待: plan unavailable

## 4. preflight validation の unit test

### 4.1 clear

#### `test_preflight_clear_accepts_same_lane_obstacle_and_open_adjacent_lane`

### 4.2 invalid obstacle placement

#### `test_preflight_rejects_obstacle_not_in_ego_lane`

### 4.3 invalid blocker placement

#### `test_preflight_rejects_blocker_not_in_opposite_lane_for_blocked_static`

### 4.4 nearby hazard

#### `test_preflight_warns_when_signal_or_junction_is_too_close`

### 4.5 missing rejoin path

#### `test_preflight_rejects_when_route_target_lane_and_origin_lane_conflict`

#### `test_preflight_accepts_double_stopped_separated`

- same-lane に 2 台
- 相互 gap が merge threshold より大きい
- `scenario_kind = double_stopped_separated`
- 期待: valid

#### `test_preflight_accepts_double_stopped_clustered`

- same-lane に 2 台
- 相互 gap が merge threshold 以下
- `scenario_kind = double_stopped_clustered`
- 期待: valid

## 5. adapter test

adapter test は pure logic へ入る前の shape を保証する。

### 5.1 tracked object reduction

#### `test_scene_state_extracts_same_lane_lead`

#### `test_scene_state_extracts_origin_lane_rejoin_gaps_while_in_target_lane`

#### `test_scene_state_preserves_target_actor_id_after_lane_change`

#### `test_scene_state_marks_target_actor_not_visible_without_dropping_memory`

#### `test_scene_state_builds_cluster_from_nearby_stopped_vehicles`

#### `test_scene_state_keeps_separated_stopped_vehicles_as_independent_targets`

### 5.2 route projection

#### `test_route_trace_projects_to_origin_lane_samples`

#### `test_route_trace_projects_to_adjacent_lane_samples`

### 5.3 preflight adapters

#### `test_environment_spawn_points_convert_to_validation_snapshot`

#### `test_adapter_emits_scenario_kind_into_validation_snapshot`

## 6. integration test

integration は少数精鋭にする。

### 6.1 `clear`

run-config:

- `tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_clear_long_expert.json`

見ること:

- overtake attempt が 1 回
- collision 0
- static vegetation 0
- lane_change_out -> pass_vehicle -> lane_change_back を踏む
- route 継続成功

### 6.2 `blocked_static`

run-config:

- `tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_blocked_long_expert.json`

見ること:

- overtake attempt 0
- reject reason が `adjacent_front_gap_insufficient` か `adjacent_lane_closed`
- collision 0
- same-lane 停止障害物は far placement でも blocker 優先の negative baseline として機能する

### 6.3 `blocked_static_near`

planned run-config:

- `tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_blocked_near_long_expert.json`

見ること:

- same-lane の停止障害物は `30-40m` 前方にある
- adjacent lane の static blocker が corridor を塞ぐ
- overtake attempt 0
- reject reason は `adjacent_front_gap_insufficient` か `adjacent_lane_closed`
- 停止車両の手前で安全に減速して停止する
- collision 0

### 6.4 `blocked_oncoming`

run-config:

- `tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_blocked_oncoming_long_expert.json`

見ること:

- moving oncoming actor に対して reject
- blocker 通過前は出ない

### 6.5 `double_stopped_separated`

run-config:

- `tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_double_stopped_separated_long_expert.json`

見ること:

- 1 台目を抜いたあとに一度 rejoin する
- その後 2 台目が `lead_vehicle_id` / `overtake_target_actor_id` として再取得される
- collision 0

### 6.6 `double_stopped_clustered`

run-config:

- `tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_double_stopped_clustered_long_expert.json`

見ること:

- 近接した 2 台を 1 つの cluster として扱う
- cluster tail を抜くまで `pass_vehicle` を維持する
- 途中で不用意に rejoin しない
- collision 0

## 7. telemetry assertions

integration test 後は summary / manifest / MCAP から次を確認する。

### summary

- `overtake_attempt_count`
- `overtake_success_count`
- `overtake_abort_count`
- `collision_count`
- `failure_reason`
- `first_target_passed_s`
- `first_rejoin_started_s`

### summary / manifest contract

#### `test_manifest_contains_overtake_target_actor_and_pass_distance_fields`

### manifest

- `planner_state`
- `overtake_state`
- `current_lane_id`
- `target_lane_id`
- `lead_vehicle_id`
- `overtake_target_actor_id`
- `overtake_target_kind`
- `overtake_target_member_actor_ids`
- `distance_past_target_m`
- `overtake_reject_reason`

### MCAP

- `/ego/planning`
- `/ego/planning_debug`
- `/world/tracked_vehicles`
- `/world/npc_vehicles`
- `/world/npc_markers`

## 8. 実装順に対応した test gate

### Gate 1

- pure unit test 緑
- integration なし

### Gate 2

- route-aligned plan test 緑
- preflight validation test 緑

### Gate 3

- `clear` integration 緑

### Gate 4

- `blocked_static` integration 緑

### Gate 5

- `blocked_static_near` integration 緑

### Gate 6

- `blocked_oncoming` integration 緑

### Gate 7

- `double_stopped_separated` integration 緑

### Gate 8

- `double_stopped_clustered` integration 緑

## 9. 最初に書くテスト

優先度順はこれ。

1. `test_lane_change_plan_does_not_reverse_when_target_lane_native_direction_is_opposite`
2. `test_pass_complete_once_target_is_behind_and_resume_gap_met`
3. `test_pass_does_not_complete_when_target_actor_temporarily_disappears`
4. `test_rejoin_gap_uses_origin_lane_id_not_relation_name`
5. `test_rejoin_begins_when_passed_and_origin_gaps_are_safe`
6. `test_overtake_rejects_when_adjacent_front_gap_insufficient`
7. `test_preflight_rejects_obstacle_not_in_ego_lane`
8. `test_pass_does_not_complete_after_visibility_timeout_without_actor_reacquisition`
9. `test_overtake_prefers_right_when_both_sides_are_open_and_right_first`
10. `test_cluster_pass_does_not_complete_until_cluster_exit_gap_is_met`
11. `test_separated_double_obstacle_rejoins_then_reacquires_second_target`
12. `test_clustered_double_obstacle_keeps_pass_vehicle_until_cluster_tail_is_cleared`

この 12 本が通らない限り、複数停止車両の CARLA 結合テストへ進まない。
