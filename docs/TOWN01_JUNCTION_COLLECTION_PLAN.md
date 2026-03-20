# Town01 Junction Collection Plan

`Town01` の fixed loop 失敗要因を埋めるための追加収集計画です。狙いは、`front RGB + speed + command -> steer` の junction signal を増やし、まず closed-loop `route_completion_ratio` を `0.1547` より明確に押し上げることです。

## Why

- 現状の command-conditioned run は `route_completion_ratio = 0.0797` で悪化
- 既存データの command 分布は `lanefollow = 13603`, `left = 600`, `straight = 529`, `right = 219`
- 一番弱いのは `right`、次が `straight`

## Frame Heuristic

概算には、2026-03-21 の成功 loop run を使います。

- full loop: `2134 planner waypoints`
- recorded frames: `10175`
- 近似係数: `1 waypoint ~= 4.77 frames`

以下の `estimated_*_frames_per_episode` はこの係数で見積もった値です。

## Core Routes

### Right-Focused

| Route | Anchors | Length | Road options | Estimated right frames / ep | Notes |
| --- | --- | ---: | --- | ---: | --- |
| `town01_right_focus_sw` | `191 -> 124` | `45.21 m` | `LANEFOLLOW 17`, `RIGHT 9` | `~43` | 短い純粋右折。南西 junction の基本例。 |
| `town01_right_focus_se` | `110 -> 203` | `169.67 m` | `LANEFOLLOW 63`, `RIGHT 31` | `~148` | 右折 frame を一気に増やす主力 route。 |
| `town01_right_focus_ne` | `74 -> 105` | `527.50 m` | `LANEFOLLOW 219`, `RIGHT 30`, `STRAIGHT 24` | `~143` | 右折を増やしつつ、追加の straight も回収する。 |

### Straight-Focused

| Route | Anchors | Length | Road options | Estimated straight frames / ep | Notes |
| --- | --- | ---: | --- | ---: | --- |
| `town01_straight_focus_south` | `35 -> 124` | `94.08 m` | `LANEFOLLOW 38`, `STRAIGHT 11` | `~52` | 南側の純粋 straight。 |
| `town01_straight_focus_east` | `106 -> 100` | `65.07 m` | `LANEFOLLOW 22`, `STRAIGHT 12` | `~57` | 東側の純粋 straight。 |
| `town01_straight_focus_north` | `174 -> 41` | `142.27 m` | `LANEFOLLOW 65`, `STRAIGHT 12` | `~57` | 北側の純粋 straight。 |
| `town01_straight_focus_west` | `103 -> 91` | `62.49 m` | `LANEFOLLOW 22`, `STRAIGHT 11` | `~52` | 西側の純粋 straight。 |

## Reserve Route

| Route | Anchors | Length | Road options | Estimated right frames / ep | Notes |
| --- | --- | ---: | --- | ---: | --- |
| `town01_right_focus_nw` | `107 -> 118` | `453.08 m` | `LANEFOLLOW 193`, `RIGHT 30`, `STRAIGHT 12` | `~143` | 予備。`ne` か `se` が不安定なときに使う。 |

## First Collection Pack

最初の 1 パックはこれで十分です。

- `town01_right_focus_se`: `4 episodes`
- `town01_right_focus_ne`: `4 episodes`
- `town01_right_focus_sw`: `3 episodes`
- `town01_straight_focus_south`: `5 episodes`
- `town01_straight_focus_east`: `5 episodes`
- `town01_straight_focus_north`: `5 episodes`
- `town01_straight_focus_west`: `5 episodes`

期待値:

- 追加 `right` frame: 約 `1290`
- 追加 `straight` frame: 約 `1370`

このパックで、少なくとも `right` は `219 -> 1500+`、`straight` は `529 -> 1800+` を狙えます。

## Execution

1 本ずつ回すときは、新しい wrapper を使います。

```bash
cd /media/masa/ssd_data/carla_alpamayo
./scripts/run_collect_town01_route.sh configs/routes/town01_right_focus_se.json
```

環境変数で同じ収集条件を固定できます。

```bash
cd /media/masa/ssd_data/carla_alpamayo
export CARLA_IMAGE_WIDTH=320
export CARLA_IMAGE_HEIGHT=180
export CARLA_TARGET_SPEED_KMH=30
export CARLA_WEATHER=ClearNoon
./scripts/run_collect_town01_route.sh configs/routes/town01_straight_focus_west.json
```

## Smoke Validation

2026-03-21 に 2 本だけ expert smoke run を確認済みです。

- `town01_right_focus_se_20260321_050901`
  - `route_completion_ratio = 1.0`
  - `collision_count = 0`
  - `elapsed_seconds = 21.0`
- `town01_straight_focus_east_20260321_050910`
  - `route_completion_ratio = 1.0`
  - `collision_count = 0`
  - `elapsed_seconds = 9.05`

## Route Plots

### Right-Focused

- [town01_right_focus_sw.png](assets/town01_junction_routes/town01_right_focus_sw.png)
- [town01_right_focus_se.png](assets/town01_junction_routes/town01_right_focus_se.png)
- [town01_right_focus_ne.png](assets/town01_junction_routes/town01_right_focus_ne.png)
- [town01_right_focus_nw.png](assets/town01_junction_routes/town01_right_focus_nw.png)

### Straight-Focused

- [town01_straight_focus_south.png](assets/town01_junction_routes/town01_straight_focus_south.png)
- [town01_straight_focus_east.png](assets/town01_junction_routes/town01_straight_focus_east.png)
- [town01_straight_focus_north.png](assets/town01_junction_routes/town01_straight_focus_north.png)
- [town01_straight_focus_west.png](assets/town01_junction_routes/town01_straight_focus_west.png)

## Decision Gate

次の再学習は、この追加収集パックのあとに 1 回だけ回します。

- 目標: closed-loop `route_completion_ratio > 0.30`
- できれば `> 0.50`
- ここで改善しなければ、次は model より先に evaluator と temporal context を見直す
