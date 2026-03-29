# Planning / Controller Refactor Plan

この文書は、[PLANNING_CONTROLLER_TARGET_ARCHITECTURE.md](/home/masa/carla_alpamayo/docs/PLANNING_CONTROLLER_TARGET_ARCHITECTURE.md) を実装へ落とすための段階計画です。

関連:

- [PLANNING_CONTROLLER_ARCHITECTURE.md](/home/masa/carla_alpamayo/docs/PLANNING_CONTROLLER_ARCHITECTURE.md)
- [PLANNING_CONTROLLER_TARGET_ARCHITECTURE.md](/home/masa/carla_alpamayo/docs/PLANNING_CONTROLLER_TARGET_ARCHITECTURE.md)

## 1. 方針

このリファクタは、次の順で進める。

1. control path の 2 系統構造を壊す
2. planner の public output を `BehaviorPlan + Trajectory` に寄せる
3. `current_behavior` と telemetry を behavior / trajectory 中心に寄せる
4. stopped-obstacle regression を通し続ける

中間状態で許容するのは、

- domain DTO と adapter が併存すること
- telemetry field が新旧混在すること

だけとする。

許容しないのは、

- `BasicAgent.run_step()` と `VehiclePIDController.run_step()` が両方 live な control path
- route 用 queue と overtake 用 queue の二重管理

## 2. フェーズ

### Phase 1: controller 一元化

目的:

- `BasicAgent / LocalPlanner` を control path から外す
- 通常走行も pure-pursuit controller で追う

変更対象:

- `ad_stack/agents/expert_basic_agent.py`
- `ad_stack/overtake/infrastructure/carla/controller_executor.py`
- `ad_stack/overtake/infrastructure/carla/execution_manager.py`

完了条件:

- `run_tracking_control()` が `local_agent.run_step()` を呼ばない
- 通常走行でも trajectory executor が point 列を供給する
- unit test が通る
- stopped-obstacle CARLA regression が通る

### Phase 2: BehaviorPlan / Trajectory 正本化

目的:

- route / overtake / rejoin / abort return を同じ `Trajectory` DTO で表す

変更対象:

- `ad_stack/overtake/domain`
- `ad_stack/overtake/application`
- `ad_stack/overtake/infrastructure/carla/route_alignment.py`

完了条件:

- planner の public output が `BehaviorPlan + Trajectory`
- route follow も overtake も同じ `Trajectory` DTO を返す
- controller は raw waypoint list ではなく `Trajectory` を受ける
- `BehaviorPlan.route_command` が入る

### Phase 3: pure-pursuit controller

目的:

- `single target waypoint` 追従から `trajectory + lookahead distance` 追従へ上げる

変更対象:

- controller adapter
- pure-pursuit helper

完了条件:

- controller が `Trajectory` と `lookahead_distance_m` を入力に動く
- point 切り替えだけで steer が飛ばない
- rejoin の steering が以前より滑らかになる

### Phase 4: longitudinal を path に寄せる

目的:

- 速度計画の正本を `TrajectoryPoint.longitudinal_velocity_mps` に寄せる

変更対象:

- `step_service.py`
- `runtime_transition.py`
- `speed_control()` 呼び出し部

完了条件:

- 通常 target speed が trajectory 由来になる
- signal stop も trajectory 上の速度 profile として表現される
- path 外の独立 emergency brake override が消える

### Phase 5: behavior / telemetry 整理

目的:

- `current_behavior` と debug DTO を behavior / trajectory 中心にする

変更対象:

- `expert_basic_agent.py`
- telemetry mapper
- MCAP / manifest / summary projection

完了条件:

- `current_behavior` が `LocalPlanner.target_road_option` 依存でない
- `behavior_state`, `route_command`, `desired_speed_mps`, `lookahead_distance_m` が出る

## 3. テスト方針

### unit test

常に通す。

- `python -m unittest discover -s tests -p 'test_*.py'`

### CARLA integration

各フェーズの終わりに stopped-obstacle suite を回す。

- `./tests/integration/ad_stack/stopped_obstacle/run_stopped_obstacle_regressions.sh`

### 成功判定

各フェーズで最低限満たすべきなのは次。

- unit test pass
- stopped-obstacle regression pass
- worktree clean の状態で run していること

## 4. commit 粒度

commit は次の粒度で切る。

- DTO / contract 追加
- adapter 差し替え
- agent orchestration 差し替え
- telemetry 投影変更
- test 追加

1 commit で

- controller 切替
- telemetry 全変更
- suite expectation 全更新

を同時にやらない。

## 5. リスク

大きいリスクは次。

- controller 一元化で rejoin / curve 系が壊れる
- `current_behavior` の導出変更で learned stack の command conditioning が壊れる
- path DTO 化で telemetry と assertion がずれる

対策:

- stopped-obstacle regression を毎フェーズ回す
- `current_behavior` 変更時は `ad_stack/api.py` まで一緒に見る
- telemetry 変更時は manifest assertion も同じ commit で直す

## 6. Done の定義

このリファクタ計画の完了は、次を満たした時点とする。

- [PLANNING_CONTROLLER_TARGET_ARCHITECTURE.md](/home/masa/carla_alpamayo/docs/PLANNING_CONTROLLER_TARGET_ARCHITECTURE.md) の `Done の定義` を満たす
- stopped-obstacle suite が新アーキテクチャ上で pass する
- 現行の 2 controller 構造に戻る fallback が消えている
