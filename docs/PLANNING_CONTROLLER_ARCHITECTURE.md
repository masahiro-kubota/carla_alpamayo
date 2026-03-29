# Planning / Controller Architecture

この文書は、現行 `ad_stack` の `planning` と `controller` の流れを、実装に即して整理したものです。

目的:

- 通常 route 追従と overtake の経路生成が、どこでどう作られているかを明確にする
- 低レベル制御が 1 系統ではなく 2 系統になっている理由を明確にする
- 現状の rejoin がなめらかでない理由を、コード構造の観点で説明できるようにする

関連:

- [PLANNING_CONTROLLER_TARGET_ARCHITECTURE.md](/home/masa/carla_alpamayo/docs/PLANNING_CONTROLLER_TARGET_ARCHITECTURE.md)
- [PLANNING_CONTROLLER_REFACTOR_PLAN.md](/home/masa/carla_alpamayo/docs/PLANNING_CONTROLLER_REFACTOR_PLAN.md)
- [OVERTAKE_ARCHITECTURE_REFACTOR_PLAN.md](/home/masa/carla_alpamayo/docs/OVERTAKE_ARCHITECTURE_REFACTOR_PLAN.md)
- [OVERTAKE_ARCHITECTURE_CURRENT_ISSUES.md](/home/masa/carla_alpamayo/docs/OVERTAKE_ARCHITECTURE_CURRENT_ISSUES.md)
- [expert_basic_agent.py](/home/masa/carla_alpamayo/ad_stack/agents/expert_basic_agent.py)

## 1. 結論

現状の `planning` は、1 つの planner が route から control まで一気通貫で担当しているわけではありません。

実際には次の 4 段に分かれています。

1. `GlobalRoutePlanner` が anchor 間の base route を作る
2. `route_geometry` が route progress と target lane を計算する
3. `overtake` が base route を土台に lane-change waypoint を合成する
4. 低レベル制御は
   - 通常走行: `BasicAgent / LocalPlanner`
   - overtaking 中: `VehiclePIDController`
   の 2 系統で動く

つまり、現状は

- `base route planning`
- `overtake patch planning`
- `dual controller execution`

の構造です。

## 2. Global Route Planning

通常 route の起点は `scenarios/routes/*.json` です。

読み込み:

- [routes.py](/home/masa/carla_alpamayo/libs/carla_utils/routes.py)
- [run.py](/home/masa/carla_alpamayo/ad_stack/run.py)

route config には主にこれがあります。

- `anchor_spawn_indices`
- `sampling_resolution_m`
- `closed_loop`

`build_planned_route(...)` は、anchor spawn の transform を取り、各 anchor 間を `GlobalRoutePlanner.trace_route(...)` で結びます。

結果として作られるもの:

- `planned_route.trace`
  - `[(waypoint, road_option), ...]`
- `anchor_transforms`
- `segment_summaries`
- `xy_points`

ここで作られる `planned_route.trace` が、現行 route planning の正本です。

## 3. Route Progress Planning

`planned_route.trace` はそのまま steering に使うだけでなく、route progress の計算にも使われます。

関連:

- [route_guidance.py](/home/masa/carla_alpamayo/libs/carla_utils/route_guidance.py)
- [api.py](/home/masa/carla_alpamayo/ad_stack/api.py)

流れ:

1. `route_geometry_from_planned_route(planned_route)` で `xy` の polyline を作る
2. 毎 step `compute_local_target_point(...)` で
   - 現在 route のどこにいるか
   - どの route point を見ているか
   - progress ratio はどれくらいか
   を計算する
3. `RouteState` として scene に入れる

この層の役割は、

- `progress_ratio`
- `route_index`
- `route_target_lane_id`

を出すことです。

重要なのは、これは低レベル control そのものではなく、上位の scene/context を作る層だということです。

## 4. Overtake Planning

通常 route と overtaking の最大の違いは、overtaking 中は一時的に base route を外れることです。

関連:

- [route_alignment.py](/home/masa/carla_alpamayo/ad_stack/overtake/infrastructure/carla/route_alignment.py)
- [lane_change_planner.py](/home/masa/carla_alpamayo/ad_stack/overtake/application/lane_change_planner.py)
- [execution_manager.py](/home/masa/carla_alpamayo/ad_stack/overtake/infrastructure/carla/execution_manager.py)

流れ:

1. base route から `route_index` 以降の sample を取る
2. 各 route point について
   - origin lane waypoint
   - left/right adjacent lane waypoint
   を揃える
3. `same_lane`
   - `lane_change`
   - `other_lane_hold`
   を progress ベースでつなぐ
4. 実行用の waypoint 列へ materialize する

ここで出来るのは、次の 2 種類です。

- overtake plan
  - `build_overtake_waypoint_execution_plan(...)`
- rejoin plan
  - `build_rejoin_waypoint_execution_plan(...)`

つまり現状の overtake planning は、

- map 全体の経路を再探索しているわけではなく
- base route を土台にした route-aligned lane-change patch

です。

## 5. Control Architecture

現行の low-level control は 1 系統ではありません。

### 5.1 通常走行

通常走行では、`BasicAgent / LocalPlanner` を使います。

関連:

- [expert_basic_agent.py](/home/masa/carla_alpamayo/ad_stack/agents/expert_basic_agent.py)
- [controller_executor.py](/home/masa/carla_alpamayo/ad_stack/overtake/infrastructure/carla/controller_executor.py)

初期化時:

- `BasicAgent(vehicle, target_speed=...)`
- `set_global_plan(planned_trace, stop_waypoint_creation=True, clean_queue=True)`

通常時の lateral planning は、この local planner が route trace を追うことで成立しています。

### 5.2 Overtake 中

overtaking 中は `VehiclePIDController` を使います。

関連:

- [expert_basic_agent.py](/home/masa/carla_alpamayo/ad_stack/agents/expert_basic_agent.py)
- [controller_executor.py](/home/masa/carla_alpamayo/ad_stack/overtake/infrastructure/carla/controller_executor.py)

流れ:

1. overtake/rejoin の waypoint queue を `OvertakeExecutionQueue` に積む
2. 毎 step `consume_next_waypoint(...)` で 1 点取り出す
3. `VehiclePIDController.run_step(target_speed_kmh, target_waypoint)` を呼ぶ

つまり overtaking 中の lateral は、local planner ではなく、単一の target waypoint を PID で追っています。

### 5.3 Longitudinal Control は別で上書きしている

ここが少しややこしい点です。

`run_tracking_control(...)` が返す `control` には

- `steer`
- `throttle`
- `brake`

がありますが、そのあと [expert_basic_agent.py](/home/masa/carla_alpamayo/ad_stack/agents/expert_basic_agent.py) で

- `speed_control(...)`
- `traffic_light_stop_control(...)`
- `emergency_stop`

で `throttle / brake` を上書きしています。

つまり、現行の実装は

- lateral
  - `BasicAgent/LocalPlanner` または `VehiclePIDController`
- longitudinal
  - `speed_control / traffic_light_stop_control / emergency brake`

の合成です。

## 6. なぜ rejoin がなめらかでないか

現状の rejoin がぎくしゃくしやすい理由は、主に次です。

### 6.1 controller が 2 系統ある

- 通常 route は `LocalPlanner`
- overtaking 中は `VehiclePIDController`

なので、phase 切り替えで追従器そのものが変わります。

### 6.2 waypoint 切り替えが離散的

overtake/rejoin path は point 列として materialize され、`consume_next_waypoint(...)` で acceptance radius を超えたら次の点へ進みます。

関連:

- [controller_executor.py](/home/masa/carla_alpamayo/ad_stack/overtake/infrastructure/carla/controller_executor.py)

このため、controller が見ている目標点は連続曲線ではなく、離散点列です。

### 6.3 expert path には steer smoothing がない

`_smooth_steer(...)` はありますが、これは learned 系 stack で使われています。

関連:

- [api.py](/home/masa/carla_alpamayo/ad_stack/api.py)
- [run.py](/home/masa/carla_alpamayo/ad_stack/run.py)

expert route policy では、

- `steering_smoothing`
- `max_steer_delta`

を通していません。

つまり、expert の route-loop では raw な steering がそのまま車両へ入ります。

### 6.4 lane-change path は route-aligned patch であって、専用 spline ではない

現状の overtake/rejoin path は

- origin lane sample
- adjacent lane sample
- progress ベース補間

から作っていて、専用の curvature-optimized path ではありません。

したがって、

- geometry が粗い
- controller 切り替えがある
- steering smoothing がない

と、rejoin が不自然になりやすいです。

## 7. 現状のアーキテクチャの評価

良い点:

- base route planning は明快
- overtake planning を base route 上の patch として実装しているので、実装コストが低い
- stopped-obstacle のような feature を追加しやすかった

弱い点:

- planning が 1 本の pipeline ではない
- controller が 2 系統ある
- longitudinal / lateral が別レイヤで合成されている
- overtaking 中だけ別 queue / 別 PID を通るので、なめらかさが落ちやすい

## 8. 関連文書

現状の説明と切り分けるため、次を別文書で管理する。

- 目標アーキテクチャ
  - [PLANNING_CONTROLLER_TARGET_ARCHITECTURE.md](/home/masa/carla_alpamayo/docs/PLANNING_CONTROLLER_TARGET_ARCHITECTURE.md)
- リファクタ計画
  - [PLANNING_CONTROLLER_REFACTOR_PLAN.md](/home/masa/carla_alpamayo/docs/PLANNING_CONTROLLER_REFACTOR_PLAN.md)

この文書は以後、

- 現状の planning / controller の実装説明
- なぜ不自然さが出るのかの構造説明

だけを持つ正本とする。
