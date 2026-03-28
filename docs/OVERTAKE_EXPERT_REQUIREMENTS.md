# Overtake Expert Requirements

`VLA` 学習用の teacher として使う `overtake expert` の要件メモです。

前提:

- `CARLA` 上で実装コストを抑えつつ、`VLA` に学ばせる価値がある追い越し挙動を作る
- teacher は不自然な「予知」をしない
- 特に、見えていない対向車線の将来状態を使って追い越し開始を決めない

## 1. 基本方針

- 行動候補の生成は観測と地図に整合する情報だけで行う
- `見えていない = 安全` とは扱わない
- `見えていない = 十分に危険なので追い越さない` を基本にする
- privileged 情報は、必要なら offline 評価や safety audit にだけ使う

要するに、teacher は

- 見えている範囲では保守的に強い
- 見えていない範囲については超能力を使わない

ことを優先する。

## 2. P0: まず成立させる追い越し

最初に対象とするのは、次のような単純なケースだけに絞る。

- 直線区間
- 見通しが良い 2 車線道路
- 信号や交差点が近くない
- lane marking 上、隣接 lane へ出られる
- 追い越し対象は 1 台の低速車、または停止車両

この範囲では

- lead vehicle を検出
- 隣接 lane が空いているか確認
- 対向 lane の見える範囲が十分長いか確認
- 入って戻るのに必要な距離を確保できるなら追い越す

で十分。

## 3. 見えてよい情報

追い越し開始の判断に使ってよいのは、次に限定する。

- ego の route
- 現在 lane と隣接 lane の幾何
- lane change が可能かどうか
- lead vehicle の現在位置・速度
- camera / planner で観測可能な対向車の現在位置・速度
- 地図から計算できる見通し距離
- 信号・交差点・カーブ・停止線までの距離

## 4. 使ってはいけない情報

追い越し開始の判断に、次は使わない。

- 見えていない対向車の現在位置
- 見えていない対向車の将来軌道
- simulator の完全状態から得た「実は先に何もいない」という情報
- occlusion の先を直接覗くような oracle 判定

これを使うと、teacher は強く見えても `VLA` に再現できない行動を出しやすい。

## 5. 追い越し開始条件

`overtake` を開始してよいのは、最低でも次を満たすときだけ。

- same lane に十分低速な lead vehicle がいる
- ego と lead vehicle の相対距離が追い越し検討レンジ内
- 交差点や signal が近すぎない
- lane marking 上、lane change が禁止されていない
- 隣接 lane に同方向の近接車両がいない
- 対向 lane に見えている範囲内で接近車両がいない
- 見通し距離が、追い越し完了に必要な距離より長い

## 6. 見通し距離ベースの保守条件

最初の実装では、追い越しを許可するのは

- `required_clear_distance_m <= visible_oncoming_corridor_m`

のときだけにする。

ここで `required_clear_distance_m` は例えば

- ego から lead vehicle を抜き切る距離
- lane change out / return に必要な距離
- 余裕マージン

の和で決める。

`visible_oncoming_corridor_m` は

- route / lane geometry
- カーブ
- crest / 建物 / occlusion
- perception の可視範囲

から決める。最初は単純化して

- 直線区間だけ
- カーブや junction 付近は一律禁止

でもよい。

## 7. 停止車両回避との関係

停止車両回避も、最初は追い越しと同じ安全条件で扱う。

- 停止車両を `very slow lead` と見なす
- 対向 lane の可視クリア距離が十分なときだけ回避する
- 見通しが足りないなら停止して待つ

これにより、停止車両回避と追い越しで別々の危険ロジックを持たずに済む。

## 8. privileged 情報の扱い

privileged 情報を完全に捨てる必要はないが、用途を限定する。

使ってよい用途:

- offline での safety audit
- scenario の難しさ分析
- teacher の false accept / false reject の評価
- benchmark の失敗理由解析

使わない用途:

- overtake 開始の直接判断
- `見えていないが安全だから行く` という実行判断

## 9. 受け入れ基準

### AC-1: 可視範囲制約

- 見通し距離が不足する scenario では追い越しを開始しない
- 見えていない対向 lane を前提にした lane change をしない

### AC-2: 安全な追い越し

- 見通しが十分ある safe scenario では少なくとも 1 回成功する
- collision 0
- unsafe lane change 0

### AC-3: 停止車両回避

- 停止車両があっても、可視条件を満たすときだけ回避する
- 可視条件を満たさないときは停止して待つ

### AC-4: ログ

- summary / manifest / MCAP で少なくとも次を残す
- overtake_state
- lead vehicle distance
- target lane
- overtake attempt / success / abort
- visible clear distance
- required clear distance
- reject reason

## 10. 実装順

### Step 1

- overtaking なし
- signal + follow を安定化

### Step 2

- 停止車両を含む単純な obstacle bypass
- 見通し距離ベースの reject を実装

### Step 3

- 1 台の低速 lead vehicle に対する safe overtake
- attempt / success / abort logging

### Step 4

- 複数 lead vehicle
- signal proximity suppression
- 右左折や交差点近傍を含む保守条件の強化

## 11. 要点

`overtake expert` は

- oracle で賢く見せる

のではなく、

- 可視範囲ベースで保守的に正しく振る舞う

teacher として作るべき。
