# Town01 Movement Coverage

`Town01` の next goal は [TOWN01_INTERSECTION_GOAL.md](./TOWN01_INTERSECTION_GOAL.md) のとおり、「任意交差点で valid movement を通せること」です。この document は、そのための **movement inventory と route suite の現状**をまとめたものです。

## What Was Generated

inventory build:

```bash
cd /media/masa/ssd_data/carla_alpamayo
./scripts/run_build_town01_movement_inventory.sh
```

この処理で生成されるもの:

- inventory JSON: `docs/assets/town01_movement_inventory.json`
- inventory plot: `docs/assets/town01_movement_inventory.png`
- recommended train routes: `configs/routes/town01_movement_train/*.json`
- recommended eval routes: `configs/routes/town01_movement_eval/*.json`

## Inventory Rule

inventory に入れている movement candidate は、ordered spawn pair の route を `GlobalRoutePlanner` で引いたときに、

- `LEFT`, `RIGHT`, `STRAIGHT` のうち 1 種類だけを含む
- つまり 1 つの movement を主対象にした short route とみなせる

ものです。

さらに recommended route を選ぶときは、

- approach context `>= 20 m`
- exit context `>= 20 m`

を最低条件にしています。

## Current Result

2026-03-22 の inventory 結果:

- spawn points: `255`
- ordered spawn pairs: `64770`
- traced pairs after Euclidean filter: `36426`
- unique movement count: `72`

movement breakdown:

- `LEFT = 24`
- `RIGHT = 24`
- `STRAIGHT = 24`

recommended route counts:

- train route configs written: `52`
- eval route configs written: `48`

coverage gaps:

- no train candidate yet: `20`
  - `LEFT = 7`
  - `RIGHT = 7`
  - `STRAIGHT = 6`
- no eval candidate yet: `24`
  - `LEFT = 9`
  - `RIGHT = 9`
  - `STRAIGHT = 6`

つまり、

- Town01 全体では `72` movement を inventory 化できた
- そのうち `48` movement は train / eval の 2 route を即座に用意できる
- 残りは exit context が足りないか、distinct eval pair が不足している

## Artifacts

- inventory JSON: [town01_movement_inventory.json](assets/town01_movement_inventory.json)
- inventory plot: [town01_movement_inventory.png](assets/town01_movement_inventory.png)

generated route suite:

- train configs: [`configs/routes/town01_movement_train/`](/media/masa/ssd_data/carla_alpamayo/configs/routes/town01_movement_train)
- eval configs: [`configs/routes/town01_movement_eval/`](/media/masa/ssd_data/carla_alpamayo/configs/routes/town01_movement_eval)

suite runner:

```bash
cd /media/masa/ssd_data/carla_alpamayo
./scripts/run_evaluate_town01_movement_suite.sh \
  --checkpoint outputs/train/pilotnet_branch_fs3_20260321_231852/best.pt
```

## Interpretation

この結果で分かったのは次です。

- fixed loop の成功は、Town01 全 movement の被覆を意味しない
- ただし Town01 全体を movement 単位に分解すること自体はできる
- `LEFT / RIGHT / STRAIGHT` は数の上では均等に `24` movement ある
- すぐに held-out eval まで組める movement は `48 / 72`

不足している movement の典型は、

- turn 後すぐに route が終わってしまい exit context が `20 m` 未満
- distinct な second route が見つからず held-out eval に分けにくい

というものです。

## Immediate Next Step

この inventory を基に、次は 2 段で進める。

1. `48` movement の train/eval route suite を evaluator で回せるようにする
2. 残り `24` movement について、approach / exit を延ばした route を追加で探索する

Town01 の mainline goal に対して最初に目指すのは、

- `48 / 72` で movement-level success を確認
- その後に `72 / 72` へ coverage を埋める

です。

## Smoke Validation

2026-03-22 に suite runner の smoke を 3 本確認した。

- `LEFT`: `town01_movement_02_left_j26_eval`
  - aggregate summary: `outputs/evaluate_suites/town01_movement_smoke_left.json`
  - result: `success_rate = 1.0`
- `RIGHT`: `town01_movement_26_right_j26_eval`
  - aggregate summary: `outputs/evaluate_suites/town01_movement_smoke_right.json`
  - result: `success_rate = 1.0`
- `STRAIGHT`: `town01_movement_49_straight_j26_eval`
  - aggregate summary: `outputs/evaluate_suites/town01_movement_smoke_straight.json`
  - result: `success_rate = 1.0`

つまり、

- generated eval routes は `evaluate_pilotnet_loop` にそのまま渡せる
- accepted checkpoint は junction-level short route の smoke では `LEFT / RIGHT / STRAIGHT` を通過できる

full `48` route suite の一括評価はまだ未実施。
