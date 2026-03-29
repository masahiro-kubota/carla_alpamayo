# Stopped Obstacle Opposite-Lane Occupancy Test Design

停止障害物回避における `反対車線占有` の観点を、どの粒度でテストへ入れるかを整理した設計書です。

目的:

- Town01 のような `片側1車線` の道路で、追い越し可否を左右する `pass corridor` 判定を明確にテストする
- 既存 scenario との重複を避けつつ、見落としやすい opposite-lane occupancy 境界だけを追加する
- `10 scenario x opposite-lane occupied/unoccupied` のような無駄な直積を避ける

関連資料:

- [STOPPED_OBSTACLE_LOGIC_DESIGN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_LOGIC_DESIGN.md)
- [STOPPED_OBSTACLE_TEST_DESIGN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_TEST_DESIGN.md)
- [STOPPED_OBSTACLE_SCENARIO_CONTRACT_TEST_DESIGN.md](/home/masa/carla_alpamayo/docs/STOPPED_OBSTACLE_SCENARIO_CONTRACT_TEST_DESIGN.md)
- [README.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/README.md)

## 1. 基本方針

反対車線占有は、`全 scenario に一律で掛け算する軸` ではなく、`pass corridor 判定を変える時だけ効く軸` として扱う。

つまり、

- `signal_suppressed`
- `near_junction_preflight_reject`
- `rejoin_blocked_then_release`

のように、別の条件が先に支配する scenario では、反対車線占有 variation を基本的に増やさない。

逆に、

- `clear`
- `curve_clear`
- `double_stopped_*`

のように、追い越し開始可否や pass corridor の安全判定が主眼の scenario では、反対車線占有 variation を持つ価値が高い。

## 2. テスト軸

### 2.1 主シナリオ軸

- `straight clear`
- `curve clear`
- `double stopped separated`
- `double stopped clustered`
- `signal suppressed`
- `near junction preflight reject`
- `rejoin blocked then release`

### 2.2 opposite-lane occupancy 軸

- `open`
  - pass corridor が空いている
- `static_blocking`
  - 反対車線上の静止車両が corridor を塞ぐ
- `moving_oncoming`
  - 対向 moving vehicle が来ていて一時的に塞ぐ
- `unavailable`
  - 反対車線がそもそも使えない
- `irrelevant_far_static`
  - 反対車線に車はいるが、pass corridor の required clear distance 外にあり判断へ影響しない

### 2.3 geometry / suppression 軸

- `straight`
- `curve`
- `signal_suppressed`
- `near_junction_preflight_reject`

## 3. 直積しない理由

`反対車線に車がいる` という事実だけではテスト価値にならない。

重要なのは、

- その車が `pass corridor` に実際に影響するか
- 影響するなら `open -> wait -> reject` のどの分岐を変えるか

である。

したがって、同じ判断境界を何度も叩くような scenario は増やさない。

例:

- `signal_suppressed + opposite static`
  - 信号抑制が先に効くので、反対車線 variation の価値は低い
- `near_junction_preflight_reject + opposite static`
  - preflight reject が先に効くので、同様に価値は低い
- `rejoin_blocked_then_release + opposite static`
  - 主眼は rejoin gap であり、pass corridor の占有ではない

## 4. 既存 scenario との対応

現在すでに opposite-lane occupancy を直接見ている scenario:

- `blocked_static`
  - `static_blocking`
- `blocked_oncoming`
  - `moving_oncoming`
- `adjacent_lane_closed`
  - `unavailable`
- `clear`
  - `open`
- `curve_clear`
  - `open` on curve

つまり、`反対車線占有そのもの` はすでに一通り触れている。

不足しているのは、

- `irrelevant_far_static`
  - 反対車線に車はいるが、追い越しを抑制しないこと
- `curve + static_blocking`
  - geometry が曲がっていても static blocker を正しく blocker とみなすこと
- `double stopped + occupied opposite lane`
  - 複数停止障害物でも opposite-lane occupancy を正しく優先すること

である。

## 5. 追加すべき integration scenario

### 5.1 `clear_with_far_opposite_static`

狙い:

- 反対車線に静止車両がいても、required clear distance 外なら false positive reject しないこと

期待:

- `lane_change_out`
- `pass_vehicle`
- `lane_change_back`
- `collision_count = 0`
- reject は `adjacent_front_gap_insufficient` にならない

### 5.2 `curve_clear_with_opposite_static_blocked`

狙い:

- カーブ区間でも `route-aligned pass corridor` 上の static blocker を blocker として扱えること

期待:

- `overtake_attempt_count = 0`
- reject は `adjacent_front_gap_insufficient` か `adjacent_lane_closed`
- collision しない

### 5.3 `double_stopped_clustered_with_oncoming_block`

狙い:

- clustered target でも、opposite lane occupancy が空くまで出ないこと

期待:

- 初期は `blocked_oncoming` 相当の reject / wait
- corridor 解放後に `target_kind = cluster` で 1 回の overtake
- cluster member actor ids を保持する

### 5.4 `double_stopped_separated_with_far_opposite_static`

狙い:

- separated 2台構成でも、無関係な opposite static actor で false positive reject しないこと

期待:

- `overtake_attempt_count = 2`
- target actor は `1台目 -> 2台目`
- irrelevant actor は blocker 扱いされない

## 6. pure / non-CARLA test に追加するもの

### 6.1 pure logic

#### `test_overtake_does_not_reject_for_irrelevant_far_opposite_static_actor`

- opposite lane の actor はいる
- ただし required clear distance 外
- 期待: reject しない

#### `test_curve_route_marks_opposite_static_actor_as_blocker_when_route_aligned_gap_is_insufficient`

- curve route 上の blocker
- route-aligned front gap 不足
- 期待: reject

#### `test_clustered_target_waits_for_oncoming_corridor_release`

- `target_kind = cluster`
- oncoming gap 不足
- 期待: `lane_change_out` に入らない

### 6.2 scenario contract

#### `test_clear_with_far_opposite_static_preflight_contract`

- ego / obstacle は same-lane
- opposite static は pass corridor required distance 外
- contract は valid

#### `test_curve_clear_with_opposite_static_blocked_preflight_contract`

- obstacle は route-ahead
- opposite static は route-aligned opposite lane 上
- contract は valid

#### `test_double_stopped_clustered_with_oncoming_block_preflight_contract`

- clustered stopped target
- oncoming actor が pass corridor 上
- contract は valid

## 7. 優先順位

追加するなら順は次です。

1. `clear_with_far_opposite_static`
2. `curve_clear_with_opposite_static_blocked`
3. `double_stopped_clustered_with_oncoming_block`
4. `double_stopped_separated_with_far_opposite_static`

理由:

- まず false positive reject を抑える
- 次に geometry 依存の blocker 検出を詰める
- 最後に multi-target との組み合わせへ広げる

## 8. done の定義

この観点が十分と言える条件は次です。

- `open / static_blocking / moving_oncoming / unavailable / irrelevant_far_static` の 5 種が、少なくとも 1 本ずつ integration scenario で表現されている
- `curve` でも opposite-lane blocker 判定が 1 本以上ある
- `cluster` でも opposite-lane occupancy が 1 本以上ある
- signal / junction のような支配的 scenario には、無意味な occupancy variation を増やしていない

この状態になれば、Town01 の `片側1車線` 前提で必要な opposite-lane occupancy coverage は十分とみなせる。
