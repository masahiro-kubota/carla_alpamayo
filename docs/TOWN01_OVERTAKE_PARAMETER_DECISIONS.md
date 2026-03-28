# Town01 Overtake Parameter Decisions

`Town01` に対して、`overtake expert` のパラメータを根拠つきで 1 個ずつ決めていくメモです。

このドキュメントの目的:

- `Town01` で実際に観測した lane 情報と route 情報を根拠にする
- 後で他マップへ移るときに、どういう観点で決めたかを再利用できるようにする

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
