# Planning / Controller Target Architecture

この文書は、`ad_stack` の planning / controller をどういう形へ寄せるかを固定するための設計書です。

Autoware の考え方に寄せて、planning と control の境界は

- `BehaviorPathPlanner`
- `Trajectory`
- `Controller`

の 3 段を正本にします。

関連:

- [PLANNING_CONTROLLER_ARCHITECTURE.md](/home/masa/carla_alpamayo/docs/PLANNING_CONTROLLER_ARCHITECTURE.md)
- [PLANNING_CONTROLLER_REFACTOR_PLAN.md](/home/masa/carla_alpamayo/docs/PLANNING_CONTROLLER_REFACTOR_PLAN.md)
- Autoware Behavior Path Planner
  - https://autowarefoundation.github.io/autoware_universe/pr-10077/planning/behavior_path_planner/autoware_behavior_path_planner
- Autoware Planning Component Design
  - https://autowarefoundation.github.io/autoware_universe/pr-10996/planning/planning_validator/autoware_planning_validator/

## 1. スコープ

この設計で扱う対象は次です。

- 通常 route 追従
- stopped obstacle overtake
- moving lead vehicle overtake
- abort return / rejoin

## 2. 確定事項

以下はこの設計で確定とする。

1. planning の正本モジュール名は `BehaviorPathPlanner` とする
2. `BehaviorPathPlanner` は `BehaviorPlan` と `Trajectory` を確定する
3. low-level controller は 1 つに統一する
4. controller は `Trajectory` だけを追う
5. `BasicAgent / LocalPlanner` は control path から外す
6. behavior と trajectory は別オブジェクトにする
7. scenario contract は production package に入れない

## 3. 採用する全体 pipeline

planning / control の pipeline は次の 4 段で固定する。

1. `Route Backbone Builder`
   - `scenarios/routes/*.json` と `GlobalRoutePlanner` から route backbone を作る
2. `Scene Snapshot Builder`
   - ego、tracked objects、traffic lights、route progress を planning 用 snapshot に変換する
3. `BehaviorPathPlanner`
   - `BehaviorPlan` と `Trajectory` を返す
4. `Controller`
   - 返された `Trajectory` を追う

この設計では、

- planning の出力は `BehaviorPlan + Trajectory`
- controller の入力は `Trajectory`

で固定する。

## 4. BehaviorPathPlanner

### 4.1 役割

`BehaviorPathPlanner` の責務は次に限定する。

- current scene と route backbone から behavior を決める
- その behavior に対応する trajectory を作る
- signal / safety / rejoin 条件を反映した目標速度を trajectory に載せる

`BehaviorPathPlanner` は controller を持たない。steer / throttle / brake は出さない。

### 4.2 入力

`BehaviorPathPlanner` の入力は次で固定する。

- `RouteBackbone`
- `PlanningScene`
- `PreviousBehaviorPlan | None`
- `PreviousTrajectory | None`

### 4.2.1 PlanningScene

`PlanningScene` の最低限の項目は次で固定する。

- `ego_pose`
- `ego_speed_mps`
- `route_index`
- `route_progress_m`
- `current_lane_id`
- `tracked_targets`
- `traffic_lights`
- `adjacent_lane_availability`

ここでの `tracked_targets` は raw CARLA actor list ではなく、target policy が見られるように正規化済みの object list とする。

### 4.3 出力

`BehaviorPathPlanner` の出力は次で固定する。

- `BehaviorPlan`
- `Trajectory`

この 2 つが planner の正本であり、debug 用 field や scenario 名はここに入れない。

## 5. BehaviorPlan

`BehaviorPlan` は planner が今なにをしようとしているかを表す別オブジェクトとする。

最低限の項目:

- `state`
  - `lane_follow`
  - `car_follow`
  - `lane_change_out`
  - `pass_vehicle`
  - `lane_change_back`
  - `abort_return`
  - `signal_stop`
- `route_command`
  - `lane_follow`
  - `left`
  - `right`
  - `straight`
- `active_target_id | None`
- `active_target_kind | None`
- `origin_lane_id | None`
- `target_lane_id | None`
- `reject_reason | None`

`BehaviorPlan` は path point を持たない。geometry は `Trajectory` 側にのみ置く。

`active_target_kind` は次の有限集合に固定する。

- `single_actor`
- `cluster`

### 5.1 状態遷移

`BehaviorPlan.state` の遷移は次に固定する。

- `lane_follow -> car_follow`
- `car_follow -> lane_change_out`
- `lane_change_out -> pass_vehicle`
- `pass_vehicle -> lane_change_back`
- `lane_change_out -> abort_return`
- `pass_vehicle -> abort_return`
- `lane_change_back -> lane_follow`
- `abort_return -> lane_follow`
- `lane_follow -> signal_stop`
- `car_follow -> signal_stop`
- `signal_stop -> lane_follow`

この設計では、

- `lane_change_out -> lane_follow`
- `pass_vehicle -> lane_follow`

のような中間 state の飛び越しは許可しない。

### 5.2 route_command

`route_command` は route backbone 由来の進行指示であり、`BehaviorPlan.state` とは別に保持する。

理由:

- `lane_follow` と `car_follow` はどちらも route 上では `left / right / straight` を持ちうる
- learned stack の command conditioning は route 意図を必要とする

したがって、

- `BehaviorPlan.state`
  - 現在の挙動モード
- `BehaviorPlan.route_command`
  - route 上の進行指示

の 2 軸で管理する。

`route_command` の導出元は `RouteBackbone.route_index -> road_option` のみとし、controller や local path geometry から逆算しない。

## 6. RouteBackbone

`RouteBackbone` は map 全体の長い route の正本とする。

最低限の保持項目:

- `trace: list[(waypoint, road_option)]`
- `xy_points`
- `progress_m`
- `route_index -> trace_index`
- `route_index -> road_option`
- `route_index -> lane_id`

用途:

- route progress 計算
- trajectory 生成の土台
- behavior の route 文脈参照

`RouteBackbone` は controller に直接渡さない。

## 7. Path と Trajectory

### 7.1 正本は Trajectory

controller に渡す正本は `Trajectory` とする。

理由:

- controller には geometry だけでなく速度目標も必要
- `Path` と `Trajectory` の 2 正本を持つと責務がぶれる

したがって production interface は `Trajectory` を採用する。

### 7.2 Path は内部表現としてのみ許可

geometry-only の `Path` を内部実装で使うこと自体は許可するが、外部 interface の正本にはしない。

つまり、

- public output: `BehaviorPlan + Trajectory`
- optional internal helper: `Path`

とする。

## 8. Trajectory

### 8.1 役割

`Trajectory` は controller が追う唯一の geometric / kinematic target とする。

最低限の項目:

- `points: list[TrajectoryPoint]`
- `trajectory_id`

`path_kind` や `segment_kind` は point に埋めず、必要なら別 metadata か telemetry で持つ。

### 8.2 TrajectoryPoint

`TrajectoryPoint` はシンプルに保つ。

最低限の項目:

- `x`
- `y`
- `z`
- `yaw_deg`
- `longitudinal_velocity_mps`

これが controller に必要な最小セットである。

### 8.3 追加 metadata

trajectory 全体の metadata として必要なのは次。

- `origin_lane_id | None`
- `target_lane_id | None`
- `source_route_start_index`
- `source_route_end_index`

ただし point ごとに `lane_id` や `segment_kind` を持たせる設計は採用しない。そういう情報は `BehaviorPlan` または telemetry に寄せる。

### 8.4 不変条件

`Trajectory` は次の不変条件を満たすものとする。

- `points` は 2 点以上
- 点列の順序は controller が追う順序そのまま
- 隣接点の距離は `1.0m +/- 0.2m` 程度に保つ
- `longitudinal_velocity_mps >= 0.0`
- `yaw_deg` は進行方向と矛盾しない

controller はこの不変条件を前提に実装する。

### 8.5 引き継ぎルール

replanning 時は、`PreviousTrajectory` の先頭をそのまま再利用するのではなく、毎 tick 新しい `Trajectory` を正本として返す。

ただし、隣接する replanning 間で次の連続性は保つ。

- 先頭数点の heading が急変しない
- `longitudinal_velocity_mps` が不連続に跳ねない
- `lane_change_out / lane_change_back` 中に target lane が反転しない

controller 側は `PreviousTrajectory` を持たず、常に最新の `Trajectory` を追う。

## 9. trajectory 生成ルール

### 9.1 local trajectory horizon

`BehaviorPathPlanner` が作る trajectory horizon は固定で次とする。

- 前方 `60m`
- 最低 `30m`
- resample 間隔 `1.0m`
- replanning 周期 `10 Hz`

### 9.2 lane_follow trajectory

通常走行時の trajectory は route backbone から切り出した local corridor をそのまま resample したものとする。

このとき `route_command` は route backbone 由来のものをそのまま `BehaviorPlan` に入れる。

### 9.3 overtake trajectory

overtake trajectory は route backbone を土台にして、behavior に応じて local rewrite する。

状態ごとの構成は次。

- `lane_change_out`
  - same-lane hold
  - lane change
- `pass_vehicle`
  - adjacent lane hold
- `lane_change_back`
  - lane change back
  - route re-attach
- `abort_return`
  - safe side への復帰軌道

controller から見ると、これらはすべて同じ `Trajectory` である。

### 9.4 速度 profile

trajectory point の `longitudinal_velocity_mps` は次を合成した結果とする。

1. base cruise speed
2. follow / overtake / rejoin の behavior speed
3. signal stop による減速

つまり longitudinal の正本も trajectory 側に載せる。

### 9.5 signal stop の表現

`signal_stop` は別 controller mode にしない。

表現は次で固定する。

- `BehaviorPlan.state = signal_stop`
- `TrajectoryPoint.longitudinal_velocity_mps` を stop line 手前で 0 に収束させる trajectory

controller は通常どおりその trajectory を追うだけとする。

### 9.6 moving target を入れたときの差分位置

moving target を導入しても、差分は次の 2 箇所に限定する。

- `TargetPolicy`
  - 何を overtake candidate とみなすか
- `BehaviorPathPlanner`
  - pass 完了条件と rejoin 開始条件

`Controller` と `TrajectoryPoint` の形式は変えない。

## 10. Controller

### 10.1 controller の正本

low-level controller は 1 つだけとする。

現段階では `pure pursuit` を正本 controller とする。`BasicAgent / LocalPlanner` は steering 計算に使わない。

### 10.2 入力

controller 入力は次で固定する。

- `ego_pose`
- `ego_speed_mps`
- `trajectory`
- `lookahead_distance_m`

`pure pursuit` controller は `trajectory` から自身で nearest point と lookahead point を求める。

つまり public interface としては、

- planner が `Trajectory` を返す
- controller が `Trajectory` と `lookahead_distance_m` を受ける

だけでよい。`nearest_index` や cumulative progress は controller 内部計算に置く。

### 10.3 lookahead ルール

`lookahead_distance_m` は固定値ではなく速度依存とする。

採用する式:

- `lookahead_distance_m = clamp(4.0 + 0.3 * ego_speed_mps, 4.0, 12.0)`

この式は controller adapter の正本ルールとする。

### 10.4 steering post-process

steering post-process は controller 層の責務とする。

最低限必須なのは次。

- steer rate limit
- low-pass smoothing

expert と learned で別実装にはしない。同じ post-process を通す。

### 10.5 controller が持ってよい内部状態

controller が内部に持ってよい状態は次に限定する。

- nearest point 探索の前回 index
- 低域フィルタの前回 steer
- steer rate limit の前回適用 steer

逆に controller は次を持たない。

- behavior state
- target actor 情報
- scenario 名

## 11. 依存方向

依存方向は次で固定する。

- `libs/carla_utils`
  - CARLA generic helper のみ
- `ad_stack/overtake/domain`
  - `RouteBackbone`, `BehaviorPlan`, `Trajectory`, `TrajectoryPoint` などの pure DTO
- `ad_stack/overtake/application`
  - `BehaviorPathPlanner` 本体
  - domain にのみ依存
- `ad_stack/overtake/policies`
  - target policy
  - domain にのみ依存
- `ad_stack/overtake/infrastructure/carla`
  - CARLA world を planning input へ変換
  - trajectory を controller 実行へ変換
- `ad_stack/agents`
  - application と infrastructure を束ねる facade

依存制約:

- `domain` は `CARLA` を import しない
- `application` は `CARLA` を import しない
- `policies` は `application` を import しない
- suite 固有 scenario contract は production package に入れない

## 12. telemetry 設計

telemetry は `BehaviorPlan` と `Trajectory` を中心に設計する。

必須 field:

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

`MCAP`, `manifest`, `summary` はこの DTO から投影する。

## 13. directory 設計

### `libs/carla_utils`

- route backbone 生成
- generic geometry helper
- generic CARLA bootstrap

### `ad_stack/overtake/domain`

- `RouteBackbone`
- `BehaviorPlan`
- `Trajectory`
- `TrajectoryPoint`
- telemetry DTO

### `ad_stack/overtake/application`

- `BehaviorPathPlanner`
- behavior state machine
- behavior-aware trajectory generation

### `ad_stack/overtake/policies`

- stopped target policy
- moving target policy
- target acceptance policy

### `ad_stack/overtake/infrastructure/carla`

- scene snapshot builder
- CARLA map -> trajectory materializer
- unified controller adapter
- telemetry serializer

### `tests/integration/ad_stack/...`

- scenario matrix
- scenario contract
- expectation docs
- suite runner

## 14. Done の定義

この設計への移行完了は、次を満たした時点とする。

- control path から `BasicAgent.run_step()` が消えている
- planner の public output が `BehaviorPlan + Trajectory` になっている
- controller が `Trajectory` だけを追っている
- `current_behavior` が `BehaviorPlan.state` 由来になっている
- stopped-obstacle の CARLA regression がすべて pass している

## 15. まとめ

この設計の核は次です。

- planning の正本は `BehaviorPathPlanner`
- planner の出力は `BehaviorPlan + Trajectory`
- controller は `Trajectory` だけを追う

以後の実装は、この 3 点に反する shortcut を採らない。
