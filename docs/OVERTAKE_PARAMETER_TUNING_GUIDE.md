# Overtake Parameter Tuning Guide

`Town01` を起点に、`overtake expert` のパラメータをどう決めるかのメモです。

目的:

- `Town01` で何を見て値を決めるかを明文化する
- そのまま他マップでも同じ観点で tuning できるようにする

前提:

- 判断は観測整合で行う
- 見えていない対向 lane は安全とみなさない
- 最初は「直線・見通し良好・junction から遠い」区間だけで追い越しを成立させる

## 1. まず map で見るもの

追い越し parameter を決める前に、少なくとも次を見る。

- 片側 1 lane の対向 2 車線道路がどこにあるか
- 長い直線区間がどこにあるか
- junction / signal の密度
- 強いカーブがどこにあるか
- 建物や遮蔽物で見通しが切れる場所
- lane marking が lane change を許すかどうか

`Town01` では、まず次の資料を起点に見る。

- route preview
  - `scenarios/routes/previews/*.png`
- signal overview
  - `outputs/inspect/traffic_light_groups/town01/town01_overview.png`

## 2. map を 3 種類の区間に分ける

parameter は map 全体で 1 回決めるのではなく、まず区間を分類して考える。

### A. 追い越しを許してよい区間

- 長い直線
- 見通しが良い
- signal / junction から遠い
- 対向 lane の可視距離が十分ある

### B. 保守的にしたい区間

- ゆるいカーブ
- 周囲に建物がある
- signal / junction がそこまで遠くない

### C. 最初は追い越し禁止にする区間

- junction 近傍
- signal 近傍
- 強いカーブ
- crest / occlusion が強い場所
- lane marking 上、追い越しが不自然な場所

最初の tuning は、`A` の区間だけで成立させるのが自然。

## 3. 各パラメータを何で決めるか

### 3.1 `overtake_trigger_distance_m`

見るもの:

- 低速車を見つけてから lane change 開始までに必要な距離
- ego の巡航速度
- Town01 の直線長

決め方:

- 追い越し対象を認識してから
  - 判定
  - lane change out
  - 加速
  の準備が間に合う最小距離より少し長めにする

Town01 では、直線区間が短いところもあるので大きくしすぎない。
最初は「早く検討しすぎて junction 近傍まで持ち越さないか」を見る。

### 3.2 `overtake_min_front_gap_m`, `overtake_min_rear_gap_m`

見るもの:

- 隣接 lane を走る車の typical spacing
- lane change の横移動時間
- Town01 の車速レンジ

決め方:

- 追い越し開始時に「隣接 lane に飛び出してすぐ詰まる」ことがない値にする
- Town01 では車速がそこまで高くないので、高速道路よりは小さくできる

### 3.3 `overtake_junction_suppression_distance_m`

見るもの:

- junction 手前で追い越しを始めると危険になる距離
- Town01 の junction 間隔
- signal 停止線の手前距離

決め方:

- lane change を始めてから戻る前に junction に入る可能性があるなら禁止
- Town01 は交差点密度が高いので、ここはかなり重要

最初は大きめに取って、危険な区間を全部禁止寄りにする。

### 3.4 `overtake_curve_suppression_curvature`

見るもの:

- どの程度のカーブで対向 lane の見通しが切れるか
- Town01 の道路で「安全に遠くまで見える直線」と「見えないカーブ」の境界

決め方:

- 追い越し可否の主判定にするというより、初期段階では「カーブなら禁止」の閾値として使う

### 3.5 `visible_oncoming_corridor_min_m`

見るもの:

- 直線区間で対向 lane がどれだけ先まで見えるか
- lane change out + pass + return に必要な距離

決め方:

- Town01 の safe overtake 区間で、実際に見えている対向 lane 長を測って下限を決める
- 最初は conservative に、明らかに十分長い区間だけ通す

### 3.6 `required_clear_distance_margin_m`

見るもの:

- lane change が完了するまでのブレ
- 追い越し対象を抜き切るのに必要な距離
- expert 制御のばらつき

決め方:

- 理論最短距離ではなく、Town01 実走での余裕を見て上乗せする

### 3.7 `oncoming_vehicle_time_gap_s`

見るもの:

- 対向車が見えている場合に、どのくらいの余裕があれば不安を感じないか
- ego と対向車の相対速度

決め方:

- 距離だけでなく時間基準でも reject するための閾値にする
- Town01 の低速環境では、高速道路ほど大きくは要らないが、短すぎると teacher が攻めすぎる

## 4. Town01 で最初に見るべき失敗

parameter tuning の初期は、次の failure を重点的に見る。

- junction 直前で追い越しを始める
- 直線が終わる前に抜き切れない
- 隣接 lane へ出た直後に戻れなくなる
- 対向 lane の見通しが足りないのに accept する
- 停止車両回避で長く対向 lane に居座る

この失敗が出たら、まず見るべき parameter は次。

- junction に近すぎる:
  - `overtake_junction_suppression_distance_m`
- 対向 lane が短すぎる:
  - `visible_oncoming_corridor_min_m`
  - `required_clear_distance_margin_m`
- 対向車に近すぎる:
  - `oncoming_vehicle_time_gap_s`
- lane へ出るのが雑:
  - `overtake_min_front_gap_m`
  - `overtake_min_rear_gap_m`
  - `lane_change_distance_m`

## 5. Town01 でのすすめ方

最初は次の順で十分。

1. `Town01` の直線 2 車線区間を 2-3 箇所決める
2. その区間だけで safe overtake scenario を作る
3. `visible_oncoming_corridor_min_m` と `overtake_junction_suppression_distance_m` を保守的に決める
4. 追い越しが成功しすぎるなら条件を少し緩める
5. 危険 accept が出るなら、まず可視距離側を厳しくする

重要なのは、最初から map 全体で万能にしようとしないこと。

## 6. 他マップへ持っていくときの観点

他の map でも、同じ順で見る。

- 直線区間の長さ
- 見通しの切れ方
- junction 密度
- signal 密度
- lane marking のルール
- 対向 lane の typical traffic speed

要するに、parameter は map 名で直接決めるのではなく、

- 直線の長さ
- 可視距離
- junction の近さ
- lane 変更余地

の 4 軸で決める。

## 7. 要点

`Town01` では

- まず「追い越してよい区間」を map 上で限定する
- その区間で必要な可視距離と suppression 距離を決める
- その後に gap と speed の値を詰める

の順が自然。
