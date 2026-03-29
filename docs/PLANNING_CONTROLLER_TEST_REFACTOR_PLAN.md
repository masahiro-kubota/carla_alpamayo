# Planning / Controller Test Refactor Plan

この文書は、[PLANNING_CONTROLLER_TARGET_ARCHITECTURE.md](/home/masa/carla_alpamayo/docs/PLANNING_CONTROLLER_TARGET_ARCHITECTURE.md) に合わせて、既存テストをどう変更するかを整理したものです。

関連:

- [PLANNING_CONTROLLER_ARCHITECTURE.md](/home/masa/carla_alpamayo/docs/PLANNING_CONTROLLER_ARCHITECTURE.md)
- [PLANNING_CONTROLLER_TARGET_ARCHITECTURE.md](/home/masa/carla_alpamayo/docs/PLANNING_CONTROLLER_TARGET_ARCHITECTURE.md)
- [PLANNING_CONTROLLER_REFACTOR_PLAN.md](/home/masa/carla_alpamayo/docs/PLANNING_CONTROLLER_REFACTOR_PLAN.md)

## 1. 前提

新設計では、planning / control の正本は次です。

- `BehaviorPathPlanner`
- `BehaviorPlan`
- `Trajectory`
- `pure pursuit Controller`

したがって、現行テストのうち

- `BasicAgent / LocalPlanner`
- `VehiclePIDController` と通常走行の二重系
- raw waypoint queue
- old planning debug field

に強く依存しているものは作り直す。

## 2. 既存テストの問題

### 2.1 controller 2 系統前提

対象:

- [test_overtake_controller_executor.py](/home/masa/carla_alpamayo/tests/test_overtake_controller_executor.py)

問題:

- `run_tracking_control()` が
  - target ありなら overtake controller
  - target なしなら base agent
  を使い分ける前提を直接テストしている

新設計では、

- 通常走行も overtaking も同じ controller

なので、この前提は消す。

### 2.2 raw waypoint queue 前提

対象:

- [test_overtake_controller_executor.py](/home/masa/carla_alpamayo/tests/test_overtake_controller_executor.py)
- [test_overtake_route_alignment.py](/home/masa/carla_alpamayo/tests/test_overtake_route_alignment.py)

問題:

- `OvertakeExecutionQueue`
- `activate_trace_plan()`
- `consume_next_waypoint()`

が正本の execution model としてテストされている。

新設計では controller 入力は `Trajectory` なので、queue 中心の保証は削る。

### 2.3 stopped-obstacle 専用 planning API 前提

対象:

- [test_overtake_step_service.py](/home/masa/carla_alpamayo/tests/test_overtake_step_service.py)
- [test_overtake_snapshot_builder.py](/home/masa/carla_alpamayo/tests/test_overtake_snapshot_builder.py)

問題:

- `OvertakeStepRequest`
- `OvertakeContext`
- `build_stopped_obstacle_targets`

など、現在の stopped-obstacle 実装にかなり密結合している。

新設計では public planning API は `BehaviorPathPlanner` に寄るので、責務の境界に合わせて再編する必要がある。

### 2.4 telemetry の旧 field 前提

対象:

- [test_overtake_telemetry_mapper.py](/home/masa/carla_alpamayo/tests/test_overtake_telemetry_mapper.py)

問題:

- `remaining_waypoints`
- `overtake_state`
- `target_lane_id`
- `lane_change_path_available`
- `emergency_stop`

などの旧 field に強く依存している。

新設計では telemetry 正本は

- `behavior_state`
- `route_command`
- `Trajectory`
- `pure pursuit` controller 指標

になるので、大幅に作り直す。

### 2.5 longitudinal / traffic-light helper の責務が旧構成前提

対象:

- [test_overtake_control_profile.py](/home/masa/carla_alpamayo/tests/test_overtake_control_profile.py)
- [test_overtake_traffic_light_service.py](/home/masa/carla_alpamayo/tests/test_overtake_traffic_light_service.py)

問題:

- `speed_control()` や `traffic_light_stop_control()` が、現在の imperative control profile を正本としている
- `select_active_light()` / `resolve_active_light()` が standalone helper として残る前提を置いている

新設計では、

- signal stop は `BehaviorPathPlanner` が `signal_stop + stop trajectory` として表現する
- controller は trajectory を追うだけ

なので、helper の存在自体を守るのではなく、

- `signal_stop trajectory` が正しく生成される
- controller がその trajectory を追った結果 stop 方向へ向かう

ことを正本としてテストし直す必要がある。

### 2.6 runtime state / runtime transition の public 性が強すぎる

対象:

- [test_overtake_runtime_state.py](/home/masa/carla_alpamayo/tests/test_overtake_runtime_state.py)
- [test_overtake_runtime_transition.py](/home/masa/carla_alpamayo/tests/test_overtake_runtime_transition.py)

問題:

- `OvertakeRuntimeState`
- `resolve_overtake_runtime_transition()`

を public な中核 API のように直接テストしている。

新設計では、state 正本は `BehaviorPlan.state` であり、遷移は `BehaviorPathPlanner` の振る舞いとして検証すべきである。

したがって、

- runtime 専用 state object を直接保証するテスト
- transition helper 単体を public contract とみなすテスト

は縮小または削除対象になる。

## 3. 削除するテスト

### 3.1 完全削除

以下は設計上の前提自体が消えるので、そのまま削除する。

- `run_tracking_control()` の base agent fallback を検証するテスト
- `OvertakeExecutionQueue` を正本として検証するテスト
- `activate_trace_plan()` の trace queue 挙動を正本とみなすテスト
- `emergency_stop` field を telemetry 正本とみなすテスト

### 3.2 削除ではなく置き換え

以下は観点は残すが、対象 API を置き換える。

- route-aligned lane-change 生成
  - `WaypointExecutionPlan` テスト
  - -> `Trajectory` 生成テストへ置換
- overtake state 遷移
  - `OvertakeStepRequest` テスト
  - -> `BehaviorPlan` 遷移テストへ置換
- traffic-light helper
  - `should_stop_for_light()` / `resolve_active_light()` 単体
  - -> `signal_stop trajectory` 生成テストへ置換
- runtime transition helper
  - `resolve_overtake_runtime_transition()` 単体
  - -> planner step ごとの `BehaviorPlan.state` 遷移テストへ置換

## 4. 新しく追加するテスト

### 4.1 BehaviorPathPlanner 単体テスト

新規ファイル候補:

- `tests/test_behavior_path_planner.py`

見ること:

- `lane_follow` 時に `BehaviorPlan + Trajectory` が返る
- `car_follow` 時に `BehaviorPlan + Trajectory` が返る
- stopped target で `lane_change_out` を返す
- pass 完了後に `lane_change_back` を返す
- abort 条件で `abort_return` を返す
- signal stop 時に `signal_stop` と stop trajectory を返す
- `route_command` を route backbone から導出する
- `PlanningScene` 必須項目が欠けている場合に失敗する
- `BehaviorPlan.state` の許可遷移だけが起きる
- `active_target_kind` が `single_actor / cluster` の有限集合に収まる
- reject 時のみ `reject_reason` が入る
- moving target を入れても `BehaviorPlan + Trajectory` の形式が変わらない

### 4.2 Trajectory contract テスト

新規ファイル候補:

- `tests/test_trajectory_contract.py`

見ること:

- `TrajectoryPoint` の最小構成
- trajectory の不変条件
  - 2 点以上
  - ordering
  - spacing
  - non-negative speed
- `trajectory_id` を持つ
- `origin_lane_id / target_lane_id / source_route_start_index / source_route_end_index` を metadata として持つ
- replanning 時の連続性
- `signal_stop` のとき stop line 手前で 0 に収束する
- `route_command` を持たないこと
- point に `lane_id` や `segment_kind` を持たないこと
- forward `60m` / minimum `30m` / resample `1.0m` の horizon ルール

### 4.3 pure pursuit controller テスト

新規ファイル候補:

- `tests/test_pure_pursuit_controller.py`

見ること:

- `Trajectory + lookahead_distance_m` だけで動く
- nearest/lookahead を内部計算する
- lookahead ルール
  - `clamp(4.0 + 0.3 * ego_speed_mps, 4.0, 12.0)`
- steer rate limit
- low-pass smoothing
- `BehaviorPlan.state` や target 情報を input に要求しない
- `BasicAgent.run_step()` を呼ばない

### 4.4 telemetry contract テスト

新規ファイル候補:

- `tests/test_behavior_trajectory_telemetry.py`

見ること:

- `behavior_state`
- `route_command`
- `active_target_id`
- `active_target_kind`
- `reject_reason`
- `desired_speed_mps`
- `applied_speed_mps`
- `lateral_error_m`
- `heading_error_deg`
- `lookahead_distance_m`
- `controller_steer_raw`
- `controller_steer_applied`

が正本として出ること

加えて次を保証する。

- `emergency_stop`
- `remaining_waypoints`
- `lane_change_path_available`

のような旧 field が public telemetry から消えること

### 4.5 RouteBackbone / route_command テスト

新規ファイル候補:

- `tests/test_route_backbone_contract.py`

見ること:

- `route_index -> road_option` が決定的に引ける
- `route_index -> lane_id` が引ける
- `progress_m` が単調増加である
- `route_command` が local path geometry から逆算されない
- controller が `RouteBackbone` を直接受け取らない

### 4.6 PlanningScene contract テスト

新規ファイル候補:

- `tests/test_planning_scene_contract.py`

見ること:

- `ego_pose`
- `ego_speed_mps`
- `route_index`
- `route_progress_m`
- `current_lane_id`
- `tracked_targets`
- `traffic_lights`
- `adjacent_lane_availability`

が最低契約として必要であること

### 4.7 signal stop trajectory テスト

新規ファイル候補:

- `tests/test_signal_stop_trajectory.py`

見ること:

- 赤信号で `BehaviorPlan.state = signal_stop` になる
- trajectory の速度が stop line 手前で単調に落ちる
- 先頭付近で負速度にならない
- signal stop は controller mode ではなく trajectory で表現される

### 4.8 integration harness contract テスト

既存 suite は維持するが、追加で

- planner public output が `BehaviorPlan + Trajectory`
- controller path に `BasicAgent.run_step()` が残っていない

ことを unit/integration 境界で検証する。

## 5. 追加で実装する予定のテスト

この refactor で新たに実装する予定のテストを、実装単位で固定する。

### 5.1 新規追加するテストファイル

- `tests/test_behavior_path_planner.py`
  - `BehaviorPlan + Trajectory` を返すこと
  - `car_follow`
  - `route_command` 導出
  - `signal_stop` 生成
  - `abort_return`
  - 許可遷移のみ起きること
  - `active_target_kind` / `reject_reason`
- `tests/test_trajectory_contract.py`
  - `TrajectoryPoint` 最小構成
  - trajectory 不変条件
  - trajectory metadata
  - replanning 連続性
  - horizon / resample ルール
- `tests/test_pure_pursuit_controller.py`
  - pure pursuit の lookahead
  - steer rate limit
  - low-pass smoothing
  - `Trajectory` 以外の planning 情報を要求しないこと
- `tests/test_behavior_trajectory_telemetry.py`
  - 新 telemetry schema
  - 旧 field 不在
- `tests/test_route_backbone_contract.py`
  - `route_index -> road_option`
  - `route_command` 導出
  - `route_index -> lane_id`
  - progress 単調性
- `tests/test_planning_scene_contract.py`
  - `PlanningScene` 必須契約
- `tests/test_signal_stop_trajectory.py`
  - signal stop の trajectory 表現

### 5.2 既存テストを置換して実装するもの

- `tests/test_overtake_route_alignment.py`
  - `test_trajectory_generation.py` 相当へ置換
- `tests/test_overtake_step_service.py`
  - `test_behavior_path_planner.py` へ吸収
- `tests/test_overtake_telemetry_mapper.py`
  - `test_behavior_trajectory_telemetry.py` へ置換
- `tests/test_overtake_control_profile.py`
  - `test_signal_stop_trajectory.py` と pure pursuit follow テストへ置換
- `tests/test_overtake_traffic_light_service.py`
  - planner の signal-stop decision テストへ置換
- `tests/test_overtake_runtime_state.py`
  - `BehaviorPlan` state 遷移テストへ吸収
- `tests/test_overtake_runtime_transition.py`
  - planner step ごとの transition テストへ吸収
- `tests/test_overtake_controller_executor.py`
  - unified controller adapter テストへ置換

### 5.3 integration で追加実装するテスト

- stopped-obstacle suite に対する adapter 境界テスト
  - planner 出力が `BehaviorPlan + Trajectory` であること
  - controller path に `BasicAgent.run_step()` が残っていないこと
- stopped-obstacle CARLA regression の維持
  - 既存 suite が refactor 後も pass すること

### 5.4 削除を伴うテスト作業

- `emergency_stop` を public field とみなす assertion を削除
- raw waypoint queue を正本とみなす assertion を削除
- dual-controller 前提の assertion を削除

## 6. 既存テストの扱い

### 維持するもの

次は観点としては有効なので残す。

- stopped-obstacle scenario contract test
- stopped-obstacle suite assertion test
- stopped-obstacle docs generation test
- integration summary / manifest assertion helper test

### 改名・再編するもの

- `test_overtake_route_alignment.py`
  - `test_trajectory_generation.py` 系へ再編
- `test_overtake_step_service.py`
  - `test_behavior_path_planner.py` 系へ再編
- `test_overtake_telemetry_mapper.py`
  - `test_behavior_trajectory_telemetry.py` 系へ再編
- `test_overtake_control_profile.py`
  - `test_signal_stop_trajectory.py` と controller follow test に再編
- `test_overtake_traffic_light_service.py`
  - planner の signal-stop decision test に再編
- `test_overtake_runtime_state.py`
  - `BehaviorPlan` state 遷移 test に吸収
- `test_overtake_runtime_transition.py`
  - `BehaviorPathPlanner` の step-level transition test に吸収

## 7. emergency stop について

方針:

- dedicated な `emergency stop` 概念は設計から外す
- したがって、`emergency_stop` を正本 field とみなすテストは削除する

現時点で確認できているのは、

- [test_overtake_telemetry_mapper.py](/home/masa/carla_alpamayo/tests/test_overtake_telemetry_mapper.py)

の fixture に `emergency_stop=False` が残っていることだけで、dedicated な emergency stop 単体テストはない。

## 8. 実施順

1. `test_overtake_controller_executor.py` の old controller 前提を削る
2. `RouteBackbone / PlanningScene / Trajectory` の contract test を先に追加する
3. `pure pursuit` controller test を追加する
4. `BehaviorPathPlanner` と `signal_stop trajectory` の test を追加する
5. telemetry test を新 schema に置き換える
6. runtime_state / runtime_transition / traffic-light helper の旧単体テストを削る
7. stopped-obstacle CARLA regression を回す

## 9. Done の定義

この test refactor が完了した状態は次。

- old architecture 前提の unit test が消えている
- `BehaviorPathPlanner`, `RouteBackbone`, `PlanningScene`, `Trajectory`, `pure pursuit` の unit test がある
- telemetry test が新 schema を見ている
- `signal_stop` が trajectory で表現されることを保証する test がある
- stopped-obstacle CARLA regression が pass している
