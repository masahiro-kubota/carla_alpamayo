# Town01 PilotNet Route

`Town01` で最初の `PilotNet風 + steer-only + PID longitudinal` を練習するための基準 route。

## 方針

- まずは `Town01` の外周に近い大きめの loop を 1 周完走できることを目標にする
- 交差点の自由意思決定はまだ入れず、route は固定する
- まずは clockwise で回し、学習データの steering 符号バランスを取るために後で counter-clockwise も追加する

## Route File

- `configs/routes/town01_pilotnet_loop.json`

## Plot

- `docs/assets/town01_pilotnet_loop.png`
- `docs/assets/town01_pilotnet_loop_summary.json`

![Town01 PilotNet loop](assets/town01_pilotnet_loop.png)

## Route Summary

- ループ種別: clockwise perimeter loop
- 総延長: 約 `4213.08 m`
- planner waypoint 数: `2134`
- anchor 数: `4`
- 主に外周を回るが、junction では planner 上 `LEFT`, `RIGHT`, `STRAIGHT` が混ざる

anchor:

1. spawn `70` at `(-1.76, 9.56)`, yaw `90.0`
2. spawn `67` at `(2.05, 318.38)`, yaw `-90.0`
3. spawn `64` at `(396.64, 318.38)`, yaw `-90.0`
4. spawn `181` at `(392.47, 9.19)`, yaw `90.0`

segment:

1. `70 -> 67`: `1244.02 m`
2. `67 -> 64`: `1028.83 m`
3. `64 -> 181`: `898.97 m`
4. `181 -> 70`: `1032.17 m`

road option counts:

- `LANEFOLLOW`: `1891`
- `LEFT`: `90`
- `RIGHT`: `48`
- `STRAIGHT`: `105`

## 使い方

plot の再生成:

```bash
cd /media/masa/ssd_data/carla_alpamayo
PYTHONPATH="" uv run python ./scripts/plot_route.py
```

## メモ

- この route は `Town01` の四隅に近い spawn point を anchor にしている
- 実際の経路は `GlobalRoutePlanner` で anchor 間を接続している
- 逆回り route は anchor 順を逆順にすればよい
- steering の左右バランスを取るため、学習データ収集では reverse route も使う前提にする
