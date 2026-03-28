# Town01 Overtake Parameter Decisions

`Town01` に対して、`overtake expert` のパラメータを根拠つきで 1 個ずつ決めていくメモです。

このドキュメントの目的:

- `Town01` で実際に観測した lane 情報と route 情報を根拠にする
- 後で他マップへ移るときに、どういう観点で決めたかを再利用できるようにする

## 現時点の Town01 初期値

Town01 向けの値は、`今の config / 実装へそのまま入れられるもの` と
`今はまだ未実装だが、Town01 用の設計値として先に決めておくもの` を分けて管理する。

### 0.1 既存実装へそのまま反映できる値

| parameter | Town01 initial value | 根拠の要約 |
| --- | --- | --- |
| `preferred_overtake_direction` | `left_first` | overtake 候補区間は右側通行構造だった |
| `overtake_trigger_distance_m` | `24.0` | 98m / 112m の直線区間で lane-change 一式を始める距離として妥当 |
| `overtake_speed_delta_kmh` | `12.0` | `12 km/h` の slow lead に対し、lane change out 中でも `24 km/h` を出せる |
| `overtake_min_front_gap_m` | `20.0` | 低速環境では十分保守的。ただし実走による追加確認はまだ必要 |
| `overtake_min_rear_gap_m` | `15.0` | 同上。`30 km/h` 環境では約 `1.8s` 相当 |
| `overtake_resume_front_gap_m` | `12.0` | 抜いた直後の cut-in を避けつつ、98m の直線区間でも戻りやすい |
| `overtake_signal_suppression_distance_m` | `35.0` | signal conflict route で same-lane の信号 trigger が `32m` 先にあった |
| `lane_change_same_lane_distance_m` | `6.0` | Town01 safe route で無理なく準備できる距離として維持 |
| `lane_change_distance_m` | `12.0` | current の `14m` より短くしても safe route 上で十分成立 |
| `lane_change_other_lane_distance_m` | `16.0` | current の `22m` は Town01 の短い直線には長すぎた |
| `overtake_hold_distance_m` | `20.0` | current の `30m` は northbound 98m 区間で余裕を削りすぎた |

### 0.2 追加実装時の Town01 設計値

| parameter | Town01 initial value | 根拠の要約 |
| --- | --- | --- |
| `overtake_min_lead_speed_kmh` | `18.0` | `30 km/h` 巡航に対し、これより速い lead は Town01 では無理に抜く価値が低い |
| `overtake_min_speed_delta_kmh` | `8.0` | 追い越しの利得がこれ未満なら Town01 の短い直線で無理をしない |
| `overtake_max_target_speed_kmh` | `30.0` | Town01 の teacher 巡航速度上限に合わせる |
| `overtake_junction_suppression_distance_m` | `60.0` | Town01 の safe route 上で必要な lane-change 幾何が約 `55m` だった |
| `visible_oncoming_corridor_min_m` | `80.0` | Town01 safe route の対向 lane 可視 corridor `112m` 以上に対し、十分保守的 |
| `visible_oncoming_corridor_cap_m` | `120.0` | Town01 safe route の corridor は `112-151m`。120m 以上は安全度を増やしすぎない |
| `required_clear_distance_margin_m` | `10.0` | Town01 の lane-change 幾何に対する上乗せ余裕として妥当 |
| `oncoming_vehicle_time_gap_s` | `4.0` | `30 vs 30 km/h` の相対速度で Town01 の `80m` corridor と整合する |
| `overtake_curve_suppression_curvature` | `0.001 1/m` | まずは「ほぼ直線しか許可しない」保守値にする |
| `occlusion_blocking_enabled` | `true` | requirement 上、「見えていない = unsafe」を徹底するため |
| `overtake_lane_marking_rule` | `allow only Broken Yellow on the left` | Town01 safe route 2 本はどちらも `Broken Yellow` だった |

### 0.3 Town01 ではまだ値を固定しないもの

Town01 でも必要だが、今の観測だけではまだ数値を決め切らないものは次。

| parameter | 現時点の扱い | まだ弱い点 |
| --- | --- | --- |
| `occlusion_margin_m` | pending | Town01 で建物・停止車両が作る occlusion の膨らませ量をまだ測っていない |
| `stopped_obstacle_speed_threshold_mps` | pending | stopped obstacle scenario をまだ起こしていない |
| `stopped_obstacle_wait_seconds` | pending | visible corridor 不足時の待ち時間を Town01 でまだ観測していない |
| `stopped_obstacle_bypass_clearance_m` | pending | 停止車両の横通過余裕を Town01 でまだ観測していない |
| `visible_oncoming_corridor_m` などのログ派生量 | required | 値ではなく logging contract なので、実装時に別途 schema 化する |

### 0.4 注意

- `overtake_speed_delta_kmh` は current 実装では `trigger` 条件にも効く
- つまり current code のままだと
  - `target_speed_kmh - overtake_speed_delta_kmh`
  - `30 - 12 = 18`
  となり、これは `overtake_min_lead_speed_kmh = 18` と実質同じ意味になる
- 将来、`追い越し速度上乗せ` と `追い越し対象認定` を分離したら、Town01 でもこの 2 つは別々に再評価する

## 1. `preferred_overtake_direction`

### 結論

- `preferred_overtake_direction = "left_first"`

### 観測方法

`CARLA` を起動して、Town01 の overtake 用 short route の中間 waypoint で

- 現在 lane
- 左隣 lane
- 右隣 lane

の `lane_type` と進行方向を確認した。

確認した route:

- [town01_northbound_overtake_short.json](/home/masa/carla_alpamayo/scenarios/routes/town01_northbound_overtake_short.json)
- [town01_road12_overtake_short.json](/home/masa/carla_alpamayo/scenarios/routes/town01_road12_overtake_short.json)

### 実測結果

#### `town01_northbound_overtake_short`

- current lane:
  - `road_id=15`
  - `lane_id=-1`
  - `yaw=89.97`
  - `lane_type=Driving`
- left lane:
  - `lane_id=1`
  - `lane_type=Driving`
  - `yaw_diff=180 deg`
  - つまり左隣は対向車線
- right lane:
  - `lane_id=-2`
  - `lane_type=Shoulder`
  - つまり右側は路肩

#### `town01_road12_overtake_short`

- current lane:
  - `road_id=12`
  - `lane_id=1`
  - `yaw=180.0`
  - `lane_type=Driving`
- left lane:
  - `lane_id=-1`
  - `lane_type=Driving`
  - `yaw_diff=180 deg`
  - つまり左隣は対向車線
- right lane:
  - `lane_id=2`
  - `lane_type=Shoulder`
  - つまり右側は路肩

### 判断

上の 2 つの代表区間では、

- 走行 lane の左側に対向の `Driving` lane がある
- 右側には `Shoulder` がある

という構造だった。

これは `Town01` が少なくともこれらの overtake 想定区間では

- 右側通行
- 追い越しは左へ出る

構造になっている、という根拠になる。

したがって、Town01 の overtaking teacher は

- まず左側を優先して追い越し可能性を評価する

設定にするのが自然。

### 注意

- これは `Town01` 全域の全 lane を証明したわけではない
- ただし、少なくとも現在 overtake 候補として見ている代表的な直線区間では一貫していた
- そのため、Town01 向けの初期値としては十分妥当

### 今後の扱い

- `preferred_overtake_direction` は Town01 では `left_first` で固定する
- 他マップへ移るときも、まず同じ観測
  - 左隣 / 右隣 lane の種類
  - 隣接 lane の進行方向
  を見て、通行側を決める

## 2. `lane_change_*` と `overtake_hold_distance_m`

### 結論

- `lane_change_same_lane_distance_m = 6.0`
- `lane_change_distance_m = 12.0`
- `lane_change_other_lane_distance_m = 16.0`
- `overtake_hold_distance_m = 20.0`

### 観測方法

`CARLA` の `BasicAgent._generate_lane_change_path()` をそのまま使って、

- current config
- candidate A
- candidate B

の 3 通りで、Town01 の safe route 上に必要な lane-change path 長を比較した。

比較した route:

- [town01_northbound_overtake_short.json](/home/masa/carla_alpamayo/scenarios/routes/town01_northbound_overtake_short.json)
- [town01_road12_overtake_short.json](/home/masa/carla_alpamayo/scenarios/routes/town01_road12_overtake_short.json)

比較した candidate:

- current
  - `same_lane=6`
  - `lane_change=14`
  - `other_lane=22`
  - `hold=30`
- candidate A
  - `same_lane=6`
  - `lane_change=12`
  - `other_lane=16`
  - `hold=20`
- candidate B
  - `same_lane=4`
  - `lane_change=10`
  - `other_lane=14`
  - `hold=16`

### 実測結果

#### safe route の長さ

- `town01_northbound_overtake_short`
  - total `98.0m`
- `town01_road12_overtake_short`
  - total `112.0m`

#### path 長 + hold の合計

##### `town01_northbound_overtake_short`

- current
  - `74.56m`
- candidate A
  - `56.65m`
- candidate B
  - `46.77m`

##### `town01_road12_overtake_short`

- current
  - `72.56m`
- candidate A
  - `54.65m`
- candidate B
  - `44.77m`

### 判断

current config のままだと、

- northbound 98m 区間に対して `74.56m`
- road12 112m 区間に対して `72.56m`

を lane-change 幾何だけで使ってしまう。

これだと、

- 追い越し対象に追いつくための距離
- 抜き切る距離
- 戻った後の余裕

がほとんど残らない。

candidate B は成立性だけ見るとさらに短いが、

- same lane 準備距離
- lane-change 本体

までかなり攻めた値になるので、Town01 の最初の teacher としては少し急すぎる。

そのため、Town01 の初期値は

- current より明確に短く
- ただし B ほど aggressive ではない

`candidate A` を採用する。

### 理由

- `lane_change_same_lane_distance_m = 6.0`
  - current と同じ。Town01 safe route 上で準備距離として十分で、ここを削る優先度は低い
- `lane_change_distance_m = 12.0`
  - current の `14m` は Town01 の短い直線区間では少し長い
- `lane_change_other_lane_distance_m = 16.0`
  - current の `22m` は opposing lane に長く居座りすぎる
- `overtake_hold_distance_m = 20.0`
  - current の `30m` は northbound 98m 直線で余裕を削りすぎる

## 3. `overtake_trigger_distance_m`

### 結論

- `overtake_trigger_distance_m = 24.0`

### 観測方法

Town01 safe route 上で、

- route 全長
- lane-change 幾何に必要な距離

を比較した。

Town01 safe route の全長:

- northbound: `98.0m`
- road12: `112.0m`

Town01 採用値の lane-change 幾何:

- `56.65m` / `54.65m`

### 判断

trigger が遅すぎると、

- slow lead に十分近づいてから lane change を始めることになり
- Town01 の 98m 級の短い直線では、抜いて戻る余裕が足りなくなる

一方、trigger を大きくしすぎると、

- junction や signal suppression に入る前から overtake を検討し始めて
- 不要な attempt が増えやすい

Town01 では、

- `20m` だとやや遅い
- `30m` だと少し早い

ので、その中間の `24m` を初期値にするのが自然。

### 理由

- northbound 98m 区間でも、`24m` trigger なら remaining route が約 `74m`
- 採用した lane-change 幾何 `56.65m` を差し引いても、まだ `17m` 以上の余裕が残る
- road12 112m 区間ではさらに余裕が大きい

## 4. `overtake_speed_delta_kmh`, `overtake_min_lead_speed_kmh`, `overtake_min_speed_delta_kmh`, `overtake_max_target_speed_kmh`

### 結論

- `overtake_speed_delta_kmh = 12.0`
- `overtake_min_lead_speed_kmh = 18.0`
- `overtake_min_speed_delta_kmh = 8.0`
- `overtake_max_target_speed_kmh = 30.0`

### 観測方法

Town01 の teacher 巡航速度と、既存の slow-lead scenario の lead speed を使って決める。

既知の条件:

- ego target speed
  - `30 km/h`
- slow lead profile
  - `12 km/h`

### 判断

Town01 の safe overtake は高速道路ではなく、短い直線の市街地 overtake なので

- 追い越し中に `30 km/h` を大きく超えるような設定は不要
- ただし lane change out 中に current の `+8 km/h` では、`12 km/h` lead に対して `20 km/h` にしかならず、少し鈍い

そこで、

- lane change out 中でも `24 km/h`
- pass 中は `30 km/h` 上限

になるように `+12 km/h` を初期値にする。

また、lead が `18 km/h` を超えるなら、Town01 の短い直線区間で無理に抜く価値はあまり高くない。

### 理由

- `overtake_speed_delta_kmh = 12.0`
  - slow lead `12 km/h` に対して lane change out でも `24 km/h`
- `overtake_min_lead_speed_kmh = 18.0`
  - `30 km/h` 巡航に対して、これより速い相手は follow でも破綻しにくい
- `overtake_min_speed_delta_kmh = 8.0`
  - 利得が小さい追い越しを Town01 の短い直線で無理にやらないため
- `overtake_max_target_speed_kmh = 30.0`
  - Town01 の teacher 巡航上限を維持するため

## 5. `overtake_signal_suppression_distance_m`

### 結論

- `overtake_signal_suppression_distance_m = 35.0`

### 観測方法

[town01_signal_conflict_short.json](/home/masa/carla_alpamayo/scenarios/routes/town01_signal_conflict_short.json) 上で、

- same-lane の traffic light trigger が route 上のどこにあるか

を `CARLA` から直接取った。

### 実測結果

- same-lane light trigger
  - actor `33`
  - route 上の距離 `32.0m`

### 判断

Town01 では、signal conflict route で same-lane の信号 trigger が `32m` 先にある。  
この区間では overtake を suppress したいので、threshold は少なくともこれより少し大きくする必要がある。

そのため、Town01 の初期値は `35m` にする。

### 理由

- `32m` という実測に対して `3m` の buffer がある
- current 値でもあり、signal conflict 用 route を抑制するという役割に合っている

## 6. `overtake_junction_suppression_distance_m`

### 結論

- `overtake_junction_suppression_distance_m = 60.0`

### 観測方法

Town01 の代表 route で、

- first junction までの距離
- Town01 採用値の lane-change 幾何長

を比較した。

### 実測結果

- `town01_eastbound_overtake_short`
  - first junction `16.35m`
  - longest non-junction run `14.0m`
- `town01_signal_conflict_short`
  - first junction `40.49m`
  - longest non-junction run `38.0m`
- Town01 採用値の lane-change 幾何長
  - `54.65m` から `56.65m`

### 判断

Town01 では、junction まで `40m` 前後しかない route は、

- lane-change 幾何だけで使う距離より短い
- つまり抜いて戻る前に junction に入る危険が高い

したがって、Town01 では

- lane-change 幾何 `55m` 前後
- それに少し buffer

を見て、`60m` 以内に junction があるなら overtake 禁止にするのが自然。

### 理由

- `eastbound` は当然抑制できる
- `signal_conflict` も junction 近傍として抑制できる
- safe route 2 本は junction が近くないので、この値でも通せる

## 7. `overtake_min_front_gap_m`, `overtake_min_rear_gap_m`, `overtake_resume_front_gap_m`

### 結論

- `overtake_min_front_gap_m = 20.0`
- `overtake_min_rear_gap_m = 15.0`
- `overtake_resume_front_gap_m = 12.0`

### 判断

この 3 つは Town01 の map 幾何だけで決まる値ではないが、

- `30 km/h` 程度の低速環境
- 98m / 112m 級の短い safe route

を両立させるには、current の値は大きくは外していない。

Town01 では、長距離の高速追い越しではなく

- 直線の短い 2 車線道路で
- 1 台の slow lead を抜いて
- すぐ戻る

ことが主目的なので、過度に gap を大きくすると成立しづらくなる。

### 根拠の強さ

- `中`
- Town01 の車速レンジと safe route 長には整合している
- ただし、Town01 の safe / blocked scenario を流して
  - accept 率
  - abort 率
  - 戻り遅れ
  を見て、まだ実走で詰める余地がある

### 理由

- `overtake_min_front_gap_m = 20.0`
  - `30 km/h` に対して約 `2.4s` 相当で、Town01 の低速環境では十分保守的
- `overtake_min_rear_gap_m = 15.0`
  - 約 `1.8s` 相当で、Town01 の短い straight でも過剰 rejection になりにくい
- `overtake_resume_front_gap_m = 12.0`
  - 抜いた直後の cut-in を避けつつ、northbound 98m 区間でも戻る余地を残せる

## 8. `visible_oncoming_corridor_min_m`, `visible_oncoming_corridor_cap_m`, `required_clear_distance_margin_m`, `oncoming_vehicle_time_gap_s`

### 結論

- `visible_oncoming_corridor_min_m = 80.0`
- `visible_oncoming_corridor_cap_m = 120.0`
- `required_clear_distance_margin_m = 10.0`
- `oncoming_vehicle_time_gap_s = 4.0`

### 観測方法

Town01 safe route 上の代表 waypoint から、

- ego lane の corridor
- left opposing lane の corridor

を `junction` に入るまで測った。

### 実測結果

#### `town01_northbound_overtake_short`

- route `20m` 地点
  - left lane corridor `140.84m`
- route `24m` 地点
  - left lane corridor `144.84m`
- route `30m` 地点
  - left lane corridor `150.84m`

#### `town01_road12_overtake_short`

- route `20m` 地点
  - left lane corridor `112.0m`
- route `24m` 地点
  - left lane corridor `116.0m`
- route `30m` 地点
  - left lane corridor `122.0m`

#### Town01 採用値の lane-change 幾何

- `54.65m` から `56.65m`

### 判断

Town01 safe route では opposing lane の可視 corridor が少なくとも `112m` ある。  
Town01 採用値の lane-change 幾何は約 `55m` なので、

- 幾何 `55m`
- return gap `12m`
- 安全余裕 `10m`

でおよそ `77m`

になる。

したがって、Town01 では

- `80m` 未満しか見えていない opposing lane では overtake 禁止

とするのが自然。

また、safe route の corridor は `112-151m` あるので、`120m` 以上は安全度を増やしすぎないために cap してよい。

時間基準では、`30 km/h` 同士の head-on は相対速度が約 `16.7 m/s` なので、

- `80m / 16.7 ≈ 4.8s`

になる。

そこから margin を考慮して、Town01 の初期値としては `4.0s` を採用する。

### 理由

- `visible_oncoming_corridor_min_m = 80.0`
  - Town01 safe route の下限 `112m` に対して保守的
- `visible_oncoming_corridor_cap_m = 120.0`
  - northbound のような長い corridor を過大評価しない
- `required_clear_distance_margin_m = 10.0`
  - Town01 の短い直線でも取りやすい追加余裕
- `oncoming_vehicle_time_gap_s = 4.0`
  - `80m` corridor と `30 vs 30 km/h` の相対速度から見て妥当

## 9. `overtake_curve_suppression_curvature`

### 結論

- `overtake_curve_suppression_curvature = 0.001 1/m`

### 観測方法

Town01 の short route で route trace の局所曲率を見た。

### 実測結果

- `town01_northbound_overtake_short`
  - max curvature `0.0002 1/m`
- `town01_road12_overtake_short`
  - max curvature `0.0 1/m`

### 判断

Town01 の P0 overtake は「ほぼ直線」だけに絞りたい。  
safe route の実測値は `0.0002 1/m` 以下だったので、これより 5 倍大きい `0.001 1/m` を保守的な straight-only gate として使う。

### 根拠の強さ

- `弱`
- straight 側の実測はある
- ただし「Town01 のどの曲率から unsafe に見えるか」を curve route でまだ対照観測していない
- したがって、この値は Town01 の P0 を始めるための provisional gate として扱う

### 理由

- straight route のノイズは吸収する
- それでも Town01 の緩い曲線や見通しの怪しい区間は極力弾ける

## 10. lane marking rule

### 結論

- Town01 の P0 overtake は `left lane marking = Broken Yellow` のときだけ許可する
- 実装では `allow rule` をそのまま持つのではなく、
  - `Broken Yellow` 以外を reject する
  blocklist へ落とす

### 観測方法

Town01 の short route 中間 waypoint で lane marking を確認した。

### 実測結果

- `town01_northbound_overtake_short`
  - left marking `Broken Yellow`
- `town01_road12_overtake_short`
  - left marking `Broken Yellow`
- `town01_signal_conflict_short`
  - left marking `Broken Yellow`
  - ただし signal / junction suppression で別途抑制すべき

### 判断

Town01 では、safe overtake 候補区間 2 本がどちらも `Broken Yellow` だった。  
したがって、P0 実装では

- `Broken Yellow` 以外は追い越し禁止

にしてよい。

### 理由

- `Solid` 系を誤って跨ぐリスクを避けられる
- Town01 で safe route として確認した marking と一致している

## 11. `occlusion_blocking_enabled`, `occlusion_margin_m`

### 結論

- `occlusion_blocking_enabled = true`
- `occlusion_margin_m` は pending

### 判断

requirement 側で、

- `見えていない = 安全` と扱わない
- occlusion の先を直接覗く oracle 判定を使わない

としているので、Town01 でも `occlusion_blocking_enabled` は `true` で固定してよい。

一方、`occlusion_margin_m` は

- 建物
- 停止車両
- 曲がり角

の具体的な膨らませ量を Town01 でまだ測っていないので、この段階では固定しない。

### 理由

- `occlusion_blocking_enabled = true`
  - Town01 の overtake teacher を観測整合に保つため
- `occlusion_margin_m`
  - Town01 の stopped-obstacle / building occlusion scenario をまだ見ていないため

## 12. stopped obstacle 関連パラメータ

### 結論

- `stopped_obstacle_speed_threshold_mps` は pending
- `stopped_obstacle_wait_seconds` は pending
- `stopped_obstacle_bypass_clearance_m` は pending

### 判断

Town01 では、まだ

- 明示的な stopped obstacle scenario
- visible corridor 不足時の待ち時間
- 停止車両の横通過余裕

を測っていない。

そのため、停止車両回避は requirement 上は必要だが、Town01 decision として数値を固定する段階にはまだ入っていない。

### 今後 Town01 で見るもの

- 停止車両を `very slow lead` と見なしたときの false accept / false reject
- 停止車両の横を抜くとき、対向 lane にどれだけはみ出すか
- visible corridor 不足時に、何秒待てば自然に諦めるか

## 13. logging contract

### 結論

Town01 で implementation する際は、少なくとも次を logging に残す。

- `visible_oncoming_corridor_m`
- `required_clear_distance_m`
- `lead_speed_kmh`
- `relative_speed_kmh`
- `oncoming_min_distance_m`
- `oncoming_min_ttc_s`
- `overtake_reject_reason`

### 判断

これは Town01 の数値 tuning というより、Town01 実走で

- なぜ accept したのか
- なぜ reject したのか
- なぜ abort したのか

を後で検証するための最低限の記録項目。

Town01 の overtake parameter は conservative に置いてあるので、
最初の検証ではこの logging がないと原因切り分けが難しい。
