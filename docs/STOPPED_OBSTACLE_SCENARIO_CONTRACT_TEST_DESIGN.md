# Stopped Obstacle Scenario Contract Test Design

`Stopped Obstacle Scenario Contract Design` を、`CARLA なし` でどう検証するかのテスト設計書です。

関連:

- [STOPPED_OBSTACLE_SCENARIO_CONTRACT_DESIGN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_SCENARIO_CONTRACT_DESIGN.md)
- [STOPPED_OBSTACLE_LOGIC_DESIGN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_LOGIC_DESIGN.md)
- [STOPPED_OBSTACLE_TEST_DESIGN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_TEST_DESIGN.md)

## 1. テストのゴール

この層のテストは、scenario を `CARLA` で回す前に次を保証する。

- run-config と environment の対応が壊れていない
- stopped-obstacle metadata が意図どおり
- synthetic preflight snapshot が意図どおり valid / invalid
- scenario 固有の pure contract が壊れていない

## 2. テストファイル

追加する正本:

- `tests/test_overtake_scenario_contracts.py`

unittest ベースで実装する。

## 3. テスト構成

### 3.1 asset wiring test

各 scenario について、実ファイルを読む。

使う loader:

- `load_route_loop_run_config()`
- `load_environment_config()`

確認項目:

- run-config path が存在する
- environment path が存在する
- route path が存在する
- `scenario_kind` が期待どおり
- `obstacle_npc_index` / `blocker_npc_index` が期待どおり
- `npc_vehicles` 数が期待どおり

### 3.2 preflight contract test

各 scenario に synthetic `PreflightValidationInput` を 1 つ固定する。

確認項目:

- `validate_preflight()` の `is_valid`
- `errors`
- `warnings`

重要:

- ここでは actual map から snapshot を生成しない
- 実ファイルとは別に、scenario intent を表す fixture を持つ
- これにより `asset wiring` と `preflight contract` を分離できる

### 3.3 scenario-specific pure contract test

`validate_preflight()` だけでは足りない scenario に限り、追加で pure logic を叩く。

対象:

- `signal_suppressed`
- `rejoin_blocked_then_release`
- `adjacent_lane_closed`

## 4. scenario ごとのテスト

### 4.1 `signal_suppressed`

#### `test_signal_suppressed_assets_match_contract`

- run-config / environment / scenario_kind を確認

#### `test_signal_suppressed_preflight_contract`

- `valid = true`
- `warnings = ["signal_nearby"]`
- `errors = []`

#### `test_signal_suppressed_rejects_overtake_in_pure_logic`

- signal suppression 距離内の red signal を与える
- `reject_reason = signal_suppressed`

### 4.2 `near_junction_preflight_reject`

#### `test_near_junction_assets_match_contract`

- run-config / environment / scenario_kind を確認

#### `test_near_junction_preflight_contract`

- `valid = false`
- `errors` に `junction_nearby`

pure logic 追加テストは不要。

### 4.3 `rejoin_blocked_then_release`

#### `test_rejoin_blocked_then_release_assets_match_contract`

- obstacle / blocker の 2 actor 構成を確認

#### `test_rejoin_blocked_then_release_preflight_contract`

- `valid = true`
- `errors = []`

#### `test_rejoin_blocked_then_release_waits_then_rejoins`

- `target_passed = true`
- まず gap 不足で `should_begin_rejoin() = false`
- 次に gap 解放で `should_begin_rejoin() = true`

### 4.4 `curve_clear`

#### `test_curve_clear_assets_match_contract`

- curve route を参照している
- `scenario_kind = curve_clear`

#### `test_curve_clear_preflight_contract`

- `valid = true`
- future route lane 上の obstacle を許容

pure logic 追加テストは、既存の validation test と重複するので不要。

### 4.5 `adjacent_lane_closed`

#### `test_adjacent_lane_closed_assets_match_contract`

- route / environment / scenario_kind を確認
- `route_aligned_adjacent_lane_available = false`

#### `test_adjacent_lane_closed_preflight_contract`

- 現行設計では `valid = false`
- `errors` に
  - `no_adjacent_driving_lane`
  - `route_aligned_adjacent_lane_unavailable`

#### `test_adjacent_lane_closed_rejects_without_open_lane`

- pure logic で `reject_reason = adjacent_lane_closed`

## 5. fixture 設計

### 5.1 table-driven

1 本の dataclass fixture を使う。

最低限の項目:

- `name`
- `run_config_path`
- `environment_path`
- `route_path`
- `scenario_kind`
- `expected_npc_count`
- `expected_blocker_present`
- `preflight_input`
- `expected_valid`
- `expected_errors`
- `expected_warnings`

### 5.2 fixture の原則

- fixture は map 観測結果ではなく `scenario intent` を表す
- `CARLA inspector` の snapshot と 1:1 で合わせる必要はない
- ただし scenario markdown と矛盾してはいけない

## 6. 失敗時の意味

このテストが落ちたときの意味は次のどれかに限る。

- run-config の参照先が壊れた
- environment metadata が壊れた
- preflight contract の意図が変わった
- pure logic の contract が壊れた

つまり、ここで落ちたら `CARLA` を回す前に直す。

## 7. Done の定義

1. `tests/test_overtake_scenario_contracts.py` が追加されている
2. 5 本すべてに asset wiring test がある
3. 5 本すべてに preflight contract test がある
4. `signal_suppressed / rejoin_blocked_then_release / adjacent_lane_closed` に pure contract test がある
5. `python -m unittest discover -s tests -p 'test_*.py'` が通る
