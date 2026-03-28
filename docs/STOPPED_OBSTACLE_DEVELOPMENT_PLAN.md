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
- [STOPPED_OBSTACLE_LOGIC_DESIGN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_LOGIC_DESIGN.md)
- [STOPPED_OBSTACLE_TEST_DESIGN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_TEST_DESIGN.md)

## 1. まず成立させるシナリオ

最初に対象とするのは、次の 3 本です。

- `clear`:
  - ego 前方の同一 lane 上に停止車両が 1 台いる
  - 反対 lane は空いている
  - expert は lane change で回避して元 lane に戻る
- `blocked_static`:
  - ego 前方の同一 lane 上に停止車両が 1 台いる
  - 反対 lane に停止 blocker がいて回避 corridor が塞がれている
  - expert は無理に出ず、停止して待つ
- `blocked_oncoming`:
  - ego 前方の同一 lane 上に停止車両が 1 台いる
  - 反対 lane に moving vehicle が接近する
  - expert は無理に出ず、停止して待つ

対象 route:

- canonical:
  - [town01_northbound_overtake_short.json](/home/masa/carla_alpamayo/scenarios/routes/town01_northbound_overtake_short.json)
- historical / obsolete:
  - [town01_road12_overtake_short.json](/home/masa/carla_alpamayo/scenarios/routes/town01_road12_overtake_short.json)

以後の停止障害物開発では、`northbound` を canonical とする。`road12` は比較用に残すが、新規調整の基準にはしない。

## 1.1 canonical regression set

停止障害物回避の baseline は次の 3 本とする。

- clear:
  - [town01_stopped_obstacle_clear_long_expert.json](/home/masa/carla_alpamayo/simulation/run_configs/town01_stopped_obstacle_clear_long_expert.json)
- blocked_static:
  - [town01_stopped_obstacle_blocked_long_expert.json](/home/masa/carla_alpamayo/simulation/run_configs/town01_stopped_obstacle_blocked_long_expert.json)
- blocked_oncoming:
  - [town01_stopped_obstacle_blocked_oncoming_long_expert.json](/home/masa/carla_alpamayo/simulation/run_configs/town01_stopped_obstacle_blocked_oncoming_long_expert.json)

これらは [run_stopped_obstacle_regressions.sh](/home/masa/carla_alpamayo/simulation/scripts/run_stopped_obstacle_regressions.sh) で一括実行する。以後、停止障害物回避の調整ではこの 3 本が崩れないことを最低条件にする。

## 2. 環境として必要なもの

### 2.1 deterministic environment

停止障害物開発用 environment は、少なくとも次を固定する。

- ego route
- 停止障害物の spawn index
- 対向 blocker の spawn index
- weather
- max seconds
- traffic light condition

現時点の canonical 候補:

- [town01_stopped_obstacle_clear_long_60s.json](/home/masa/carla_alpamayo/scenarios/environments/town01_stopped_obstacle_clear_long_60s.json)
- [town01_stopped_obstacle_blocked_long_60s.json](/home/masa/carla_alpamayo/scenarios/environments/town01_stopped_obstacle_blocked_long_60s.json)
- [town01_stopped_obstacle_blocked_oncoming_long_60s.json](/home/masa/carla_alpamayo/scenarios/environments/town01_stopped_obstacle_blocked_oncoming_long_60s.json)

`town01_stopped_obstacle_clear_45s.json` / `town01_stopped_obstacle_blocked_45s.json` は短い初期案として残すが、以後の調整基準は long シナリオに寄せる。

### 2.1.1 scenario contract

ここが曖昧だと、失敗が

- environment の置き方の問題
- overtake policy の問題

のどちらか切れない。停止障害物開発用 scenario では、次を明示的に固定する。

- ego spawn lane id
- stopped obstacle lane id
- blocker lane id
- ego と stopped obstacle の初期縦距離
- blocker と ego の初期縦距離
- 使用する隣接 lane が `Driving` であること
- 隣接 lane の直進可能長
- 近傍に junction / signal / parked props がないこと
- 復帰先 lane が route と一致していること

最低限、`clear` では

- ego と stopped obstacle が同一 lane
- 隣接 lane が開いている
- 停止障害物の前後に十分な直線長がある
- 復帰先 lane に静的障害物がない

を保証する。

`blocked` ではさらに

- 反対 lane が本当に使えない理由を 1 つに限定する

ことを保証する。たとえば

- stationary blocker で lane occupied
- oncoming vehicle で front gap insufficient

は意味が違うので、scenario は分ける。

### 2.2 stopped obstacle profile

停止障害物は `autopilot off` で固定し、速度 `0 km/h` を保つ。

- [stopped_obstacle_profile_v1.json](/home/masa/carla_alpamayo/scenarios/npc_profiles/stopped_obstacle_profile_v1.json)

### 2.2.1 blocked scenario semantics

`blocked` は現在

- 「対向 lane が stationarily occupied で出られない」

として扱っている。

これは

- 「対向車が接近してくるので出られない」

とは別物なので、設計書でも分ける。

最低でも次の 2 種類を分離する。

- `blocked_static`
  - opposite lane に停止 blocker がいる
- `blocked_oncoming`
  - opposite lane に moving vehicle が接近する

### 2.3 run-config

シミュレーション実行は shell wrapper ではなく JSON run-config で固定する。

必要な run-config:

- [town01_stopped_obstacle_clear_long_expert.json](/home/masa/carla_alpamayo/simulation/run_configs/town01_stopped_obstacle_clear_long_expert.json)
- [town01_stopped_obstacle_blocked_long_expert.json](/home/masa/carla_alpamayo/simulation/run_configs/town01_stopped_obstacle_blocked_long_expert.json)
- [town01_stopped_obstacle_blocked_oncoming_long_expert.json](/home/masa/carla_alpamayo/simulation/run_configs/town01_stopped_obstacle_blocked_oncoming_long_expert.json)

含めるべき設定:

- `policy.kind = expert`
- `record_mcap = true`
- `record_video = false`
- `mcap_segment_seconds`
- `ignore_traffic_lights = false`
- `ignore_vehicles = false`

### 2.4 preflight validation

run 前に scenario を自動検証できるようにする。最低限ほしい check は次。

- ego / obstacle / blocker の spawn lane id を出す
- ego と stopped obstacle の初期縦距離を出す
- opposite lane の front / rear gap を出す
- 隣接 lane が `Driving` かどうかを出す
- route 上で rejoin 先 lane がどこかを出す
- 近傍の signal / junction を出す

これを満たさない scenario は、expert の調整対象に入れない。

preflight の結果は必ず

- summary
- manifest 先頭

の両方で確認できるようにする。

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

## 5.1 挙動契約

停止障害物回避は、状態名だけでなく遷移条件を固定する。

### lane_change_out

開始条件:

- lead vehicle が同一 lane 上にいる
- lead speed が threshold 以下
- lead distance が trigger 以内
- target adjacent lane が `Driving`
- front / rear gap が threshold 以上
- signal suppression なし

完了条件:

- current lane id が overtake target lane id に一致

### pass_vehicle

開始条件:

- ego が overtake target lane に乗った

継続条件:

- stopped obstacle をまだ十分に追い越していない

終了条件:

- ego rear が obstacle front を十分に抜いた
- かつ origin lane へ戻るための front / rear gap がある

ここは固定距離 hold ではなく、

- overtake target actor との相対位置
- origin lane の再侵入余裕

で決めるほうが自然。

### lane_change_back

開始条件:

- pass 完了
- origin lane の rejoin gap が十分

完了条件:

- current lane id が origin lane id に戻る

### abort_return

発火条件:

- signal suppression
- target lane 消失
- unexpected obstacle 出現

期待:

- 元 lane へ最短で安全に戻る

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
- opposite lane 滞在時間が過剰でない
- shoulder / vegetation への接触 0
- overtake target actor を明示的に pass してから戻る

### AC-2 blocked

- 停止障害物がいても、危険な対向車がある間は出ない
- collision 0
- 無理な overtake attempt 0
- reject reason が一意に説明できる

### AC-3 logs

- MCAP だけ見て、停止障害物回避の判断過程を追える
- manifest だけ見ても、accept / reject の理由を grep できる

### AC-4 scenario validity

- preflight validation だけで scenario の契約違反を検出できる
- clear / blocked / blocked_oncoming を environment 名だけで区別できる

## 8. 追加で MCAP / manifest に欲しい情報

今の debug telemetry に加えて、停止障害物回避では次があると切り分けが速い。

- `overtake_target_actor_id`
- `overtake_origin_lane_id`
- `passed_overtake_target`
- `distance_past_overtake_target_m`
- `rejoin_front_gap_m`
- `rejoin_rear_gap_m`
- `lane_change_path_available`
- `lane_change_path_failed_reason`
- `nearest_static_hazard_distance_m`
- `scenario_validity_errors`

特に

- 「本当に obstacle を抜いたから戻った」のか
- 「固定距離を消化したから戻ろうとした」のか

は切り分けられるようにする。

## 9. 直近でやること

- [ ] stopped-obstacle 用 run-config を追加する
- [ ] `/world/tracked_vehicles` を runtime から `MCAP` へ配線する
- [ ] `/world/traffic_lights` を runtime から `MCAP` へ配線する
- [ ] planning_debug の完全重複 field を整理する
- [ ] clear / blocked を通常 entrypoint で smoke test する
- [ ] preflight validation を作る
- [ ] `blocked_static` と `blocked_oncoming` を分ける
- [ ] pass 完了条件を固定距離ではなく target actor 基準へ変える
- [ ] その結果を見て、停止障害物の追い越しロジックを詰める
