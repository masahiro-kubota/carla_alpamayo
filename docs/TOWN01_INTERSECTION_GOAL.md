# Town01 Intersection Generalization Goal

`Town01` の fixed-loop 完走は baseline として達成済みです。次の main goal は、**`Town01` 内の任意交差点で、車線接続として成立する任意方向に進めること**です。

## Goal

ここでいう「任意交差点で任意方向」とは、

- `Town01` の各 junction に対して
- 各 incoming approach から到達可能な outgoing branch のうち
- planner 上 `LEFT`, `RIGHT`, `STRAIGHT` として表現される movement

を、`front RGB[t-2:t] + speed + command -> steer` の mainline policy で通せることを指します。

補足:

- `U-turn` は現時点では対象外
- `target-point` のような連続 route guidance は使わない
- longitudinal は引き続き `BasicAgent` / PID

## Why This Is The Next Goal

fixed loop は「特定 route を通せる」ことの確認には十分でしたが、

- Town01 全体の movement coverage を保証しない
- unseen route composition の評価にならない
- `command` 条件付き policy の一般化限界が見えない

という限界があります。

次の目標は、loop 完走ではなく **movement coverage と route composition** を Town01 全体で見ることです。

## Mainline Task

- input: `front RGB[t-2:t] + speed + command`
- output: `steer`
- planner keeps:
  - route intent as discrete `command`
  - longitudinal control
- disallowed:
  - planner 由来の連続 guidance
  - `target-point`
  - route progress 依存の checkpoint 切り替え

## Evaluation Levels

### Level 1: Movement Coverage

Town01 の全 junction movement を short route で評価する。

各 eval route は:

- 1 つの junction movement を主対象にする
- approach context と post-turn settling 区間を含む
- training route とは別 episode / 別 route instance にする

`success` 条件:

- `route_completion_ratio >= 0.99`
- `collision_count == 0`
- `manual_interventions == 0`
- `distance_to_goal_m <= 10`

Level 1 の合格条件:

- Town01 の全 valid movement で `success`

### Level 2: Route Composition

複数 junction を含む unseen route set を評価する。

route set の要件:

- loop training route そのものは使わない
- 2 から 5 個の junction decision を含む
- left / right / straight が偏らない

Level 2 の合格条件:

- held-out route suite 全体で高い成功率を維持する
- 最低限、各 route で movement-level の failure が再発しない

### Level 3: Town01 Arbitrary Route

Town01 内で planner が生成する新規 route に対しても、junction decision を伴う走行を継続できる状態。

これは最終的な Town01 内目標であり、Level 1 と Level 2 を通してから測る。

## Data Requirement

この goal で必要なのは、単純な総フレーム数より **movement coverage** です。

最低限そろえるもの:

- 各 junction の `LEFT`
- 各 junction の `RIGHT`
- 各 junction の `STRAIGHT`
- movement 前の approach context
- movement 後の lane settle 区間
- high-curvature `LANEFOLLOW`

特に重要なのは、

- 同じ `RIGHT` でも junction ごとに見え方が違うこと
- 曲がった直後は `LANEFOLLOW` に戻るので post-turn 収束区間が必要なこと
- unseen route composition では movement 単体の成功だけでは足りないこと

## Immediate Work Plan

1. `Town01` の valid junction movement を列挙する
2. movement coverage table を作る
3. train route と held-out eval route を分離する
4. Level 1 の movement suite を実装する
5. coverage が足りない movement を追加収集する
6. その後に Level 2 の unseen multi-junction route suite に進む

2026-03-22 の現状:

- inventory 化できた unique movement は `72`
- `LEFT / RIGHT / STRAIGHT` は各 `24`
- そのうち `48` movement は train / eval の両 route を即座に用意できる

詳細は [TOWN01_MOVEMENT_COVERAGE.md](./TOWN01_MOVEMENT_COVERAGE.md) を参照。

## Relationship To The Existing Loop Baseline

- [TOWN01_PILOTNET_ROUTE.md](./TOWN01_PILOTNET_ROUTE.md) は baseline の再現対象として残す
- fixed loop 完走は「Town01 policy の初期 sanity check」
- ただし final goal は fixed loop ではない
- 今後の mainline 成否は、この document の Level 1 / Level 2 を基準に判断する
