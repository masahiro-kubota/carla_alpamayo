# Stopped Obstacle Next Steps

停止障害物回避の integration suite を次にどう進めるかの作業メモです。

## 現状

### Verified baseline

- `clear`
- `blocked_static`
- `blocked_oncoming`

この 3 本は、期待どおりの `overtake / reject / wait` を確認済みです。

### Newly added but not yet acceptable

- `signal_suppressed`
- `rejoin_blocked_then_release`
- `double_stopped_separated`
- `double_stopped_clustered`
- `curve_clear`
- `near_junction_preflight_reject`

このうち `near_junction_preflight_reject` は期待どおりに `scenario_preflight_invalid` で early reject できています。

残り 4 本は、`scenario_validation.valid=false` のまま走っており、主因は `ad_stack` の回避ロジックではなく scenario asset 側の整合不足です。

### Still missing

- `adjacent_lane_closed`

これは expectation markdown はあるが、Town01 上で正しい corridor / obstacle placement をまだ確定していないため、environment と run-config を未作成のままにしています。

## いまの問題

追加 4 シナリオでは、共通して次が起きています。

- `obstacle_not_ahead`
- `route_target_lane_conflict`

つまり、

- obstacle が本当に ego の前方同一レーンに載っていない
- route と scenario が想定する target lane が噛み合っていない

という scenario authoring の問題です。

## 方針

### 1. 先に scenario asset を直す

まずはロジックを触らない。

対象:

- `signal_suppressed`
- `rejoin_blocked_then_release`
- `double_stopped_separated`
- `double_stopped_clustered`
- `curve_clear`

これらは `scenario_validation.valid=true` になるまで、route / environment / spawn を修正する。

### 2. `spawn_index` 依存を減らし、必要なら `spawn_transform` を使う

再現性のある停止障害物シナリオでは、

- `spawn_index`

だけに頼ると lane 契約が崩れやすい。

必要な actor は、

- `NPCSpawnTransformSpec`

で明示配置する。

### 3. scenario inspector を先に作る

CARLA 本実行の前に、route と environment を読んで次を確認できる inspector を用意する。

- obstacle が本当に ego 前方か
- obstacle / blocker の lane id は期待どおりか
- route target lane conflict がないか
- 近傍 signal / junction 条件が設計どおりか

この段階で NG なら、本実行しない。

### 4. 4 本を 1 本ずつ再設計する

#### `signal_suppressed`

- 前方停止障害物
- 近接赤信号
- overtake は開始しない

#### `rejoin_blocked_then_release`

- overtake 開始はできる
- 元 lane の rejoin gap が最初は不足
- 後から開いて rejoin できる

#### `double_stopped_separated`

- 同一進路上に十分離れた停止障害物が 2 台
- 1 台目を抜いて rejoin 後に 2 台目を再取得できる

#### `double_stopped_clustered`

- 近接した停止障害物 cluster を 1 つの corridor として扱う
- cluster tail を抜くまで途中 rejoin しない

#### `curve_clear`

- カーブ上でも前方同一レーンで scenario が成立
- route-aligned plan が逆走しない

### 5. `scenario_validation.valid=true` を acceptance gate にする

新規 scenario は、

- `scenario_validation.valid=true`

になるまで baseline 候補に入れない。

### 6. その後に CARLA 結合テストを回す

scenario authoring が通ったあとで、はじめて次を評価する。

- `overtake_attempt_count`
- `overtake_success_count`
- `collision_count`
- `failure_reason`
- MCAP 上の `target actor / reject reason / pass / rejoin`

### 7. `adjacent_lane_closed` は最後

Town01 で本当に

- adjacent lane が `Driving` でない
- もしくは route-aligned に使えない

corridor を確定してから作る。

## 直近の作業順

1. inspector を作る
2. `signal_suppressed` を inspector が通るように直す
3. `rejoin_blocked_then_release` を inspector が通るように直す
4. `double_stopped_separated` / `double_stopped_clustered` を inspector が通るように直す
5. `curve_clear` を inspector が通るように直す
6. 5 本の CARLA 結合テスト
7. `adjacent_lane_closed` の corridor 探索

## Done の定義

追加 scenario を `done` と呼ぶ条件は次です。

- run-config がある
- expectation markdown がある
- `scenario_validation.valid=true`
- CARLA 結合テストで期待挙動になる
- baseline に入れるかどうかを README に反映済み
