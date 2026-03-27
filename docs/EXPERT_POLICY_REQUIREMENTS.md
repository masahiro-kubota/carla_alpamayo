# Expert Policy Requirements

このドキュメントは、今後 `ad_stack` に実装する
「privileged state を使う expert policy」の要件定義です。

具体設計は [EXPERT_POLICY_DESIGN.md](./EXPERT_POLICY_DESIGN.md) を参照。

対象は、`CARLA` の ground-truth 情報を直接参照できる前提の
rule-based / planner-based expert です。
画像認識ベースの perception はこの段階では要求しません。

## 1. 目的

この expert policy の目的は次の 2 つです。

- 将来の学習用データ収集で使える、安定した expert driver を作る
- `simulation -> ad_stack.run(request)` の共通実行系で、そのまま closed-loop 評価にも使える基準 policy を作る

最終的に必須とする driving capability は次です。

- 信号に従って走行できる
- 自車より低速の先行車を安全に追従できる
- 必要なときに追い越しできる

## 2. 前提

- map / lane topology は `CARLA` から直接取得できる
- traffic light の状態は画像認識ではなく simulator の状態を直接取得できる
- 周辺車両の位置、速度、向きは simulator の状態を直接取得できる
- expert は `simulation/` から直接呼ばず、`ad_stack.run(request)` の内部 policy として実行する
- route は `scenarios/routes/` で与える

## 3. スコープ

この要件定義で対象にするもの:

- route-following
- traffic light handling
- lead vehicle following
- lane change
- overtake
- collision 回避のための最低限の safety guard
- 収集 / 評価で共通に使える summary と event logging

この段階では対象外:

- 画像認識ベースの信号認識
- pedestrian interaction
- unprotected complex intersection negotiation
- emergency vehicle handling
- multi-agent game-theoretic planning
- learned planner / learned overtaking policy

## 4. ユースケース

### UC-1: Route Following

- route に沿って通常走行する
- 制限速度または設定 target speed を守る
- lane center を維持する

### UC-2: Traffic Light Compliance

- 赤信号で停止する
- 黄信号では停止可能なら停止する
- 青信号で発進する
- 停止線を大きく踏み越えない

### UC-3: Lead Vehicle Following

- 同一 lane 上に低速車両があるとき追従する
- unsafe な車間にならないよう減速する
- 追い越し不可なら route progress を維持しつつ追従を継続する

### UC-4: Overtake

- 先行車が十分低速で、隣接 lane が利用可能かつ安全なら lane change して追い越す
- 追い越し後は route に復帰する
- 追い越し中に unsafe 条件が生じたら abort または保守的に減速する

## 5. 機能要件

### FR-1: Route Tracking

- expert は route 上の現在 progress を内部状態として持つ
- route progress に基づいて target lane と target speed を決める
- route から大きく外れた場合は復帰動作を行う

### FR-2: Traffic Light Observation

- expert は ego の進行先に関連する traffic light を特定できる
- `red / yellow / green` の状態を直接読める
- どの信号が ego に有効かを lane / waypoint ベースで判定できる

### FR-3: Traffic Light Behavior

- red では停止線手前で停止 command を生成する
- yellow では stopping distance と current speed に基づいて
  - stop
  - pass
  のどちらかを一貫して選ぶ
- green では通常走行へ復帰する

### FR-4: Dynamic Actor Observation

- expert は同一 lane と隣接 lane の車両を取得できる
- 各車両について最低限次を扱える
  - relative position
  - speed
  - heading
  - lane relation

### FR-5: Lead Vehicle Following

- 同一 lane 上の最重要先行車を特定する
- headway / TTC に基づいて target speed を落とす
- 先行車がいない場合は巡航速度へ戻す

### FR-6: Overtake Decision

- 追い越し開始条件を明示的に持つ
- 最低限、次が揃ったときだけ追い越しを開始する
  - 先行車が同一 lane 上にいる
  - 先行車速度が ego target speed より十分低い
  - 隣接 lane が route 上または一時的に利用可能
  - 前後安全距離を満たす
  - 近い将来の信号や交差点進入で追い越しが不適切でない

### FR-7: Overtake Execution

- lane change out
- passing
- lane return
  の状態機械を持つ
- 追い越し中は進行方向の安全確認を継続する
- lane return は十分な前方間隔が取れてから行う

### FR-8: Overtake Abort / Fallback

- 隣接 lane に新たな車両が入る
- lane return が安全でない
- route 上の交差点や信号が近い
- required gap が失われた

上記のときは次のどちらかへ安全に落ちること:

- 元 lane に戻って追従
- 現 lane で減速して安全確保

### FR-9: Safety Guard

- 最終 command の前に safety veto をかける
- 最低限次を持つ
  - frontal TTC guard
  - collision imminent brake
  - unsafe lane change reject

### FR-10: Logging / Artifact

- summary に最低限次を記録する
  - traffic_light_stop_count
  - traffic_light_violation_count
  - car_follow_event_count
  - overtake_attempt_count
  - overtake_success_count
  - overtake_abort_count
  - min_ttc
- frame manifest には可能なら次を記録する
  - planner state
  - active traffic light state
  - lead vehicle distance
  - overtake state

## 6. 非機能要件

### NFR-1: Determinism

- 同じ simulator seed / NPC config / route config なら、挙動が大きくぶれないこと

### NFR-2: Recoverability

- 一時的な route offset や先行車割り込みで即座に破綻しないこと

### NFR-3: Modularity

- `simulation/` は expert の内部実装を知らず、引き続き `ad_stack.run(request)` だけを呼ぶ
- expert の行動ロジックは `ad_stack` 内部で閉じる

### NFR-4: Extensibility

- 将来 `NPC`, `pedestrian`, `intersection negotiation` を追加しやすい構造であること
- learned policy と比較評価しやすい summary 項目を持つこと

## 7. シナリオ要件

最終的には、最低限次のシナリオを用意する。
Phase 1 の最初の実装対象は `SR-1` から `SR-3` までとし、
`SR-4` 以降は overtaking 機能を有効化した段階で追加する。

### SR-1: Red Light Stop

- route 上に red signal がある
- expert は停止線手前で停止する

### SR-2: Green Light Resume

- red で停止後、green で自然に再発進する

### SR-3: Slow Lead Vehicle Follow

- 同一 lane に低速 NPC がいる
- expert は衝突せず追従できる

### SR-4: Safe Overtake

- 隣接 lane が空いている
- expert は追い越し成功後に route へ復帰できる

### SR-5: Unsafe Overtake Rejection

- 隣接 lane に接近車両がいる
- expert は追い越しを開始しない、または abort する

### SR-6: Traffic Light + Overtake Conflict

- 前方低速車と近い信号が同時にある
- expert は信号優先で保守的に判断する

## 8. 受け入れ基準

### AC-1: Traffic Light

- red light scenario で signal violation 0
- 停止位置が停止線から大きく逸脱しない

### AC-2: Following

- slow lead vehicle scenario で collision 0
- 追従中の minimum TTC が閾値を下回らない

### AC-3: Overtake

- overtaking 機能を有効化した phase で評価する
- safe overtake scenario で少なくとも 1 回成功する
- unsafe overtake scenario で unsafe lane change を起こさない

### AC-4: Route Completion

- 専用 benchmark route で `route_completion_ratio >= 0.99`
- `collision_count == 0`

### AC-5: Simulation Integration

- `simulation.pipelines.run_route_loop --policy-kind expert` で expert 実行できる
- 同じ executor を使って benchmark 評価もできる
- summary に overtake / traffic light 指標が出る

## 9. 実装上の示唆

この repo の現状に対して、次の分割が自然です。

- `ad_stack/world_model`
  - traffic light state
  - nearby vehicle state
  - lane relation
- `ad_stack/api.py` または後継 internal module
  - expert route-loop stack
  - overtake state machine
  - longitudinal target speed planner
- `ad_stack/run.py`
  - summary / manifest への event 反映
- `scenarios/routes/`
  - route
- 将来的には `scenarios/environments/`, `scenarios/npc_profiles/`
  - NPC 配置と速度 profile

## 10. 段階的マイルストーン

### Phase 1

- traffic light direct handling
- slow lead vehicle following
- overtake なし

### Phase 2

- single-lane to adjacent-lane overtake
- overtake success / abort logging

### Phase 3

- signal proximity を考慮した overtake suppression
- 複数 NPC を含む benchmark scenario

## 11. 最初の実装対象

最初の 1 本目としては、次を作るのが妥当です。

- `Town01`
- fixed route
- 1 台の低速 lead vehicle
- traffic light direct state handling
- overtake なし

ここで

- 信号遵守
- 追従
- 低速車追従での安全性

が通れば、expert data collection 用の最小本体として扱える。

追い越しと abort は Phase 2 以降の受け入れ対象とする。
