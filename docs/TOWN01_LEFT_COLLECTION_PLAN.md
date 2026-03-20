# Town01 Left Collection Plan

`Town01` fixed loop を完走させるために、2026-03-21 時点で不足している `LEFT` supervision を増やす計画です。  
直前までの expert-only command 分布は `left = 600`, `straight = 2113`, `right = 1224`, `lanefollow ≈ 24098` で、左折だけが明確に不足しています。

## Why Pivot

- 直近の最良 run は hybrid policy で `route_completion_ratio = 0.9405`
- ただし最後の失敗は `lanefollow` 区間の understeer だった
- 一方で、元の expert data 自体は左折が薄い
- `command-conditioned` の単一 checkpoint を立て直すには、loop に近い `LEFT` scene をもっと集める必要がある

## Candidate Routes

固定ループの実走 manifest から左折区間を逆引きして、入口・出口に近い spawn pair を選びました。

| Route | Anchors | Length | Road options | Estimated left frames / ep | Notes |
| --- | --- | ---: | --- | ---: | --- |
| `town01_left_focus_south` | `111 -> 95` | `243.76 m` | `LANEFOLLOW 80`, `LEFT 31`, `STRAIGHT 12` | `~148` | 主力。短くて左折密度が高い。 |
| `town01_left_focus_sw` | `84 -> 91` | `618.74 m` | `LANEFOLLOW 269`, `LEFT 30`, `STRAIGHT 11` | `~143` | 左下 corner に対応。approach と exit が長い。 |
| `town01_left_focus_ne` | `82 -> 125` | `750.63 m` | `LANEFOLLOW 336`, `LEFT 30`, `STRAIGHT 12` | `~143` | 右上側の左折。fixed loop に近い見え方を含む。 |
| `town01_left_focus_west` | `92 -> 77` | `759.94 m` | `LANEFOLLOW 331`, `LEFT 20`, `STRAIGHT 24`, `RIGHT 10` | `~95` | 補助。純度は下がるが context が長い。 |

概算は既存の成功 loop run の係数 `1 waypoint ~= 4.77 frames` を使っています。

## First Pack

- `town01_left_focus_south`: `4 episodes`
- `town01_left_focus_sw`: `4 episodes`
- `town01_left_focus_ne`: `4 episodes`
- `town01_left_focus_west`: `3 episodes`

期待値:

- 追加 `left` frames: 約 `2000`

これで expert-only の `left` はおよそ `600 -> 2600` を狙えます。

## Execution

```bash
cd /home/masa/carla_alpamayo
./scripts/run_collect_town01_left_pack.sh
```

1 本ずつ回す場合:

```bash
cd /home/masa/carla_alpamayo
./scripts/run_collect_town01_route.sh configs/routes/town01_left_focus_south.json
```

## Smoke Validation

2026-03-21 に 2 本だけ expert smoke run を確認済みです。

- `town01_left_focus_south_20260321_060645`
  - `route_completion_ratio = 1.0`
  - `collision_count = 0`
  - `elapsed_seconds = 15.3`
- `town01_left_focus_ne_20260321_060645`
  - `route_completion_ratio = 1.0`
  - `collision_count = 0`
  - `elapsed_seconds = 158.8`

## Route Plots

- [town01_left_focus_south.png](assets/town01_left_routes/town01_left_focus_south.png)
- [town01_left_focus_sw.png](assets/town01_left_routes/town01_left_focus_sw.png)
- [town01_left_focus_ne.png](assets/town01_left_routes/town01_left_focus_ne.png)
- [town01_left_focus_west.png](assets/town01_left_routes/town01_left_focus_west.png)

## Next Gate

- まず left pack を最後まで expert 収集
- その後 `front RGB + speed + command -> steer` の単一 policy を再学習
- 主指標は fixed loop の closed-loop `route_completion_ratio`
- `0.9405` を超えたら次の failure 位置に対して同じ反復を継続
