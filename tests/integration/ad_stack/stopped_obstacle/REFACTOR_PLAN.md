# Stopped Obstacle Refactor Plan

停止車両回避を今後継続的に開発できる形へ寄せるためのリファクタ計画です。

この文書の目的:

- `clear / blocked_static / blocked_oncoming` でたまたま通った状態から抜ける
- scenario authoring, preflight validation, runtime behavior の責務を分ける
- 新しい integration scenario を追加しても、どこが壊れているかをすぐ切り分けられるようにする

関連:

- [README.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/README.md)
- [NEXT_STEPS.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/NEXT_STEPS.md)
- [STOPPED_OBSTACLE_LOGIC_DESIGN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_LOGIC_DESIGN.md)
- [STOPPED_OBSTACLE_TEST_DESIGN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_TEST_DESIGN.md)

## 1. 問題整理

いま時間がかかっている主因は、停止障害物回避ロジックそのものより、周辺の test harness と scenario authoring です。

具体的には:

- `spawn_index` ベースの配置が fragile
  - 同じ corridor を狙ったつもりでも、spawn 後の lane 契約が崩れやすい
- preflight validation が runtime の観測タイミングに依存しやすい
  - spawn 直後の未安定 pose で lane id や前方距離を取ると誤判定しやすい
- scenario contract が明文化されていない
  - `obstacle が route 上のどこにいるべきか`
  - `blocker が何 lane を塞ぐべきか`
  - `signal / junction との距離条件`
  を JSON だけでは読み取りづらい
- integration scenario ごとの期待値は書いてあるが、authoring と validation の修正手順がまだ散っている

結論:

- `pure logic`
- `scenario contract / validation`
- `runtime adapter`
- `integration assets`

の 4 層に分けて進める。

## 2. ゴール

この refactor で達成したいことは次です。

1. baseline 3 scenario を stable に維持する
2. planned scenario を 1 本ずつ `scenario_validation.valid=true` にできる
3. scenario が壊れているのか runtime が壊れているのかを 1 回で切り分けられる
4. 新しい scenario を足すときに `spawn_index` の勘に頼らない
5. MCAP / manifest の debug 情報が scenario expectation と対応付く

## 3. 非ゴール

この計画の対象外:

- 停止障害物以外の追い越し全般の一般化
- PID / longitudinal control の全面見直し
- Town01 以外の map への同時展開
- VLA 学習用データセットの設計全体

## 4. あるべき責務分離

### 4.1 `ad_stack/overtake/`

責務:

- stopped-obstacle の pure logic
- preflight validation contract
- pass / rejoin / reject の判定

ここに置くもの:

- `stopped_obstacle_logic.py`
- 必要なら `stopped_obstacle_contracts.py`

ここに置かないもの:

- CARLA actor spawn
- route-loop entrypoint
- JSON file path 解決

### 4.2 `simulation/`

責務:

- generic runtime / environment config parsing
- generic inspector / CLI

ここに置くもの:

- `environment_config.py`
- `inspect_stopped_obstacle_scenario.py`
- generic route-loop pipeline

ここに置かないもの:

- stopped-obstacle suite 固有の期待値 markdown
- stopped-obstacle suite 固有の regression policy

### 4.3 `tests/integration/ad_stack/stopped_obstacle/`

責務:

- integration suite の正本
- scenario expectation
- run-config
- regression runner
- refactor / next-steps tracking

ここに置くもの:

- `run_configs/`
- scenario ごとの markdown
- `run_stopped_obstacle_regressions.sh`
- この `REFACTOR_PLAN.md`

## 5. リファクタ方針

### 5.1 `spawn_index` 依存を baseline から減らす

方針:

- baseline 3 本を含め、停止障害物シナリオは最終的に `spawn_transform` 正本へ寄せる
- `spawn_index` は authoring の探索補助に留める

理由:

- `spawn_index` は Town01 の corridor 契約を表さない
- `spawn_transform` なら
  - lane id
  - yaw
  - 前方距離
  を明示できる

Acceptance:

- baseline 3 本の obstacle / blocker が `spawn_transform` でも同じ挙動を再現できる

### 5.2 preflight を `route-aligned corridor contract` に寄せる

現状の問題:

- `same lane` の定義が spawn 直後の waypoint に引っ張られやすい

今後の contract:

- `ego_lane_id == obstacle_lane_id` だけでなく
- `route progress 上で obstacle が ego より前方`
- `chosen adjacent lane が route-aligned に利用可能`

を主条件にする

Acceptance:

- baseline `clear` が warmup 後に stable に `valid=true`
- `near_junction_preflight_reject` は狙いどおり invalid

### 5.3 inspector を scenario authoring の入口に固定する

今後の順序は固定する。

1. route / environment を作る
2. inspector を回す
3. `scenario_validation.valid=true` を確認する
4. その後にだけ route-loop を回す

Acceptance:

- planned scenario を結合テストする前に inspector artifact が必ず存在する

### 5.4 integration scenario を段階分けする

scenario は 3 層に分ける。

- `baseline`
  - すでに regression 対象
- `candidate`
  - inspector green、まだ route-loop 未固定
- `verified`
  - expectation を満たして regression 候補

対象:

- `signal_suppressed`
- `rejoin_blocked_then_release`
- `double_stopped_obstacle`
- `curve_clear`
- `near_junction_preflight_reject`
- `adjacent_lane_closed`

Acceptance:

- README で各 scenario のステータスが分かる

### 5.5 runtime 変更は scenario authoring が落ち着いてから

原則:

- inspector で invalid の scenario に対して runtime を触らない
- runtime を触るのは
  - scenario contract が通っている
  - それでも expectation を満たさない
場合だけ

理由:

- scenario asset failure を runtime bug と誤認しないため

## 6. 実行フェーズ

### Phase 1: 契約の安定化

- baseline 3 本を inspector green にする
- preflight / warmup / route-aligned contract を安定化

Done:

- `clear`
- `blocked_static`
- `blocked_oncoming`

が `scenario_validation.valid=true`

### Phase 2: asset migration

- baseline 3 本を `spawn_transform` 正本へ寄せる
- inspector の lane sample を使って transform を決める

Done:

- baseline env JSON が `spawn_transform` を使う
- inspector で same result を再現する

### Phase 3: candidate scenario authoring

優先順:

1. `signal_suppressed`
2. `rejoin_blocked_then_release`
3. `double_stopped_obstacle`
4. `curve_clear`
5. `near_junction_preflight_reject`
6. `adjacent_lane_closed`

各 scenario の手順:

1. transform を置く
2. inspector green にする
3. expectation markdown を更新する

### Phase 4: runtime verification

candidate scenario を route-loop で回し、

- success / stall / collision
- reject reason
- target actor
- pass / rejoin

を expectation と照合する

### Phase 5: regression promotion

条件を満たした scenario を regression set に入れる

## 7. 直近の TODO

最初にやること:

1. baseline 3 本の `spawn_transform` 化
2. `blocked_static` / `blocked_oncoming` も inspector green か再確認
3. `signal_suppressed` を candidate に上げる
4. `rejoin_blocked_then_release` を candidate に上げる

この段階では、まだ `curve_clear` と `adjacent_lane_closed` を急がない。

## 8. Done の定義

この refactor が完了したと呼べる条件:

- baseline 3 本が `spawn_transform` 正本になっている
- inspector が scenario authoring の必須入口として定着している
- planned scenario のうち少なくとも 4 本が `candidate` 以上になっている
- regression runner の対象が README と一致している
- `scenario invalid` と `runtime bug` を別々に追える状態になっている
