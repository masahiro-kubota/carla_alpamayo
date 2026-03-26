# Town01 Junction Collection Plan

注記: この document は historical note です。ここで扱う補助 route config と batch script は current minimal repo の `data_collection/` には残していません。

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
./data_collection/scripts/run_collect_town01.sh --route-config data_collection/configs/routes/town01_right_focus_se.json
```

環境変数で同じ収集条件を固定できます。

```bash
cd /media/masa/ssd_data/carla_alpamayo
export CARLA_IMAGE_WIDTH=320
export CARLA_IMAGE_HEIGHT=180
export CARLA_TARGET_SPEED_KMH=30
export CARLA_WEATHER=ClearNoon
./data_collection/scripts/run_collect_town01.sh --route-config data_collection/configs/routes/town01_straight_focus_west.json
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

## Execution Result

2026-03-21 に first pack を最後まで実行しました。

- executed episodes: `31 / 31`
- success episodes: `31 / 31`
- added command frames:
  - `right = 1006`
  - `straight = 1584`
  - `lanefollow = 10091`
- updated total command counts:
  - `lanefollow = 23694`
  - `left = 600`
  - `straight = 2113`
  - `right = 1225`

収集自体は計画どおり成功です。ただし、この追加収集のあとに回した command-conditioned 再学習では fixed-loop completion は改善しませんでした。詳細は [TOWN01_CAMERA_E2E_RESULTS.md](TOWN01_CAMERA_E2E_RESULTS.md) にまとめています。

## Upper-Band Recovery Expansion

full loop の後半 failure を切り出し直した結果、`right_focus_ne` 単体ではなく、その手前の upper-band eastbound approach を含む文脈が不足していました。そこで 2 本の recovery route を追加しました。

| Route | Anchors | Length | Road options | Purpose |
| --- | --- | ---: | --- | --- |
| `town01_right_focus_upper_band_mid` | `95 -> 105` | `245.18 m` | `LANEFOLLOW 104`, `RIGHT 10`, `STRAIGHT 12` | full-loop failure の直前から RIGHT turn までを再現する主力 route |
| `town01_right_focus_upper_band_long` | `60 -> 105` | `300.96 m` | `LANEFOLLOW 121`, `RIGHT 10`, `STRAIGHT 24` | より長い pre-turn context を含めて drift を補正する route |

2026-03-21 にこの pack を追加実行しました。

- executed episodes: `40 / 40`
- success episodes: `40 / 40`
- added command frames:
  - `lanefollow = 22497`
  - `right = 1412`
  - `straight = 3332`
- updated total command counts:
  - `lanefollow = 136514`
  - `left = 6042`
  - `right = 7048`
  - `straight = 12138`

この expansion の前は、accepted best `outputs/train/pilotnet_branch_fs3_20260321_210500/best.pt` が

- `town01_right_focus_upper_band_mid`: `route_completion_ratio = 0.6825`, `collision`
- `town01_right_focus_upper_band_long`: `route_completion_ratio = 0.7419`, `collision`

で落ちていました。追加収集と correction windows を使った fine-tune 後は、

- `town01_right_focus_upper_band_mid_pilotnet_eval_20260321_232239`: `success = true`
- `town01_right_focus_upper_band_long_pilotnet_eval_20260321_232253`: `success = true`
- `town01_pilotnet_loop_pilotnet_eval_20260321_232707`: `success = true`, `route_completion_ratio = 0.9991`, `collision_count = 0`

まで到達しています。

## Route Plots

### Right-Focused

- [town01_right_focus_sw.png](assets/town01_junction_routes/town01_right_focus_sw.png)
- [town01_right_focus_se.png](assets/town01_junction_routes/town01_right_focus_se.png)
- [town01_right_focus_ne.png](assets/town01_junction_routes/town01_right_focus_ne.png)
- [town01_right_focus_nw.png](assets/town01_junction_routes/town01_right_focus_nw.png)
- [town01_right_focus_upper_band_mid.png](assets/town01_junction_routes/town01_right_focus_upper_band_mid.png)
- [town01_right_focus_upper_band_long.png](assets/town01_junction_routes/town01_right_focus_upper_band_long.png)

### Straight-Focused

- [town01_straight_focus_south.png](assets/town01_junction_routes/town01_straight_focus_south.png)
- [town01_straight_focus_east.png](assets/town01_junction_routes/town01_straight_focus_east.png)
- [town01_straight_focus_north.png](assets/town01_junction_routes/town01_straight_focus_north.png)
- [town01_straight_focus_west.png](assets/town01_junction_routes/town01_straight_focus_west.png)

## Decision Gate

次の再学習は、この追加収集パックのあとに 1 回だけ回します。

- 目標: closed-loop `route_completion_ratio > 0.30`
- できれば `> 0.50`
- 実際には `0.0788` と `0.0797` で失敗
- 次は model より先に evaluator と temporal context を見直す
