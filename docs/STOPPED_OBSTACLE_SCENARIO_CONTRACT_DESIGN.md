# Stopped Obstacle Scenario Contract Design

停止障害物回避の integration scenario について、`CARLA` を起動する前に壊れた asset を落とすための `scenario contract` 層の設計書です。

この文書の目的:

- run-config / environment JSON の意図を明文化する
- `validate_preflight()` に渡す synthetic snapshot の正本を決める
- `CARLA` を使わずに確認できる範囲を明確にする
- scenario asset failure と runtime failure を分離する

関連:

- [STOPPED_OBSTACLE_LOGIC_DESIGN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_LOGIC_DESIGN.md)
- [STOPPED_OBSTACLE_TEST_DESIGN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_TEST_DESIGN.md)
- [README.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/README.md)

## 1. ねらい

現状の stopped-obstacle suite は、scenario ごとの markdown と CARLA integration run はあるが、その間にある

- `run-config が正しい route / environment を指しているか`
- `environment の stopped_obstacle metadata が意図どおりか`
- `preflight contract` が意図どおり valid / invalid になるか

を `CARLA` なしでまとめて叩く層が薄い。

この層を追加すると、次を早く落とせる。

- environment path の取り違え
- wrong `scenario_kind`
- `nearest_signal_distance_m` / `nearest_junction_distance_m` の意図ずれ
- `route_aligned_adjacent_lane_available` の宣言ミス
- `adjacent_lane_closed` のような未完成 scenario の見落とし

## 2. 対象 scenario

この contract 層の対象は、まず次の 5 本とする。

- `signal_suppressed`
- `near_junction_preflight_reject`
- `rejoin_blocked_then_release`
- `curve_clear`
- `adjacent_lane_closed`

baseline 5 本とは別枠で、`planned / candidate` 側の asset を先に固めるための層とする。

## 3. 責務

### 3.1 contract 層が持つ責務

- 実ファイルを読む
  - run-config JSON
  - environment JSON
- scenario metadata を検証する
  - `scenario_kind`
  - route path
  - environment path
  - obstacle / blocker index
  - suppression / junction metadata
- synthetic `PreflightValidationInput` を scenario ごとに固定する
- `validate_preflight()` の期待結果を固定する
- 必要なものだけ pure logic を追加で叩く

### 3.2 contract 層が持たない責務

- 実 actor の lane id 決定
- spawn 後 warmup した pose の観測
- `lead_vehicle_id` の収束確認
- `lane_change_out / pass_vehicle / lane_change_back` の runtime 遷移確認

これらは引き続き `CARLA integration` の責務。

## 4. 入出力

### 4.1 入力

1. 実ファイル
- `tests/integration/ad_stack/stopped_obstacle/run_configs/*.json`
- `scenarios/environments/*.json`

2. synthetic preflight snapshot
- `PreflightValidationInput`

3. 必要に応じた pure logic input
- `StoppedObstacleContext`
- `OvertakeMemory`

### 4.2 出力

- asset が意図どおり構成されていること
- preflight が意図どおり valid / invalid になること
- scenario 固有の pure contract が 1 つ以上守られていること

## 5. scenario ごとの contract

### 5.1 `signal_suppressed`

asset contract:

- run-config は `town01_signal_conflict_short` を使う
- environment は `town01_stopped_obstacle_signal_suppressed_45s.json`
- `scenario_kind = signal_suppressed`
- `nearest_signal_distance_m` が設定されている

preflight contract:

- `valid = true`
- `warnings` に `signal_nearby`
- `errors = []`

pure contract:

- `choose_overtake_action()` は `reject_reason = signal_suppressed`

### 5.2 `near_junction_preflight_reject`

asset contract:

- signal-conflict route を使う
- `scenario_kind = near_junction_preflight_reject`
- `nearest_junction_distance_m` が設定されている

preflight contract:

- `valid = false`
- `errors` に `junction_nearby`

pure contract:

- 追加の runtime 判定までは持たない
- contract test の主眼は `preflight invalid` の固定

### 5.3 `rejoin_blocked_then_release`

asset contract:

- long northbound route を使う
- `scenario_kind = rejoin_blocked_then_release`
- obstacle と blocker の 2 actor を宣言する

preflight contract:

- `valid = true`
- same-lane obstacle と blocker metadata が揃う

pure contract:

- `target_passed = true` でも rejoin gap が不足していれば `should_begin_rejoin() = false`
- gap 解放後は `should_begin_rejoin() = true`

### 5.4 `curve_clear`

asset contract:

- curve route を使う
- `scenario_kind = curve_clear`
- obstacle は current lane ではなく future route lane にいてよい
- junction / signal suppression は十分遠い

preflight contract:

- `valid = true`
- `obstacle_lane_id != ego_lane_id` でも `obstacle_route_lane_id` が route 上なら許容

pure contract:

- `curve_clear` だけは future route lane 上の obstacle を許容する

### 5.5 `adjacent_lane_closed`

asset contract:

- candidate route は `town01_adjacent_lane_closed_corridor`
- `scenario_kind = adjacent_lane_closed`
- `route_aligned_adjacent_lane_available = false`

preflight contract:

- 現行設計では `valid = false`
- 少なくとも
  - `no_adjacent_driving_lane`
  - `route_aligned_adjacent_lane_unavailable`
  を含む

pure contract:

- `choose_overtake_action()` は `reject_reason = adjacent_lane_closed`

補足:

- この scenario は runtime integration 前段の contract test で止める対象として扱う
- exact transform が固まるまでは baseline に入れない

## 6. テスト資産の置き場

### 6.1 正本

- 設計: `docs/`
- 実ファイル:
  - `tests/integration/ad_stack/stopped_obstacle/run_configs/`
  - `scenarios/environments/`
- non-CARLA contract test:
  - `tests/test_stopped_obstacle_scenario_contracts.py`

### 6.2 期待しないこと

この層で

- MCAP / manifest の中身
- actual actor id
- `lead_vehicle_id` の収束

までは見ない。そこは integration suite 側に残す。

## 7. Done の定義

この設計書に対して done と言える条件は次。

1. 5 本すべてに run-config / environment asset がある
2. 5 本すべてに synthetic preflight fixture がある
3. 5 本すべてに expected `valid / errors / warnings` が固定されている
4. 5 本のうち pure logic が必要な scenario は、その contract を unit test で持つ
5. `CARLA` を起動せずに `asset wiring` と `preflight contract` の破綻を落とせる
