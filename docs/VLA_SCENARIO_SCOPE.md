# VLA Scenario Scope

`CARLA` 上で `VLA` 学習用データを集めるときの、現時点のシナリオ優先度メモです。

## P0: 最初に安定化するもの

- route following
- 信号に基づく停止・再発進
- 低速先行車への追従

理由:

- `CARLA` 側の実装コストが比較的低い
- expert の挙動をまず安定させやすい
- `VLA` の基本運転に直結する

## P1: P0 の次に足すもの

- 黄信号の判断
- 停止車両の回避
- 信号 + 先行車
- 安全な追い越し

理由:

- `route + signal + follow` の延長で作れる
- expert の設計を大きく変えずに追加できる
- 「止まるだけではない判断」を増やせる

## P2: その後に広げるもの

- 遅車両列への対応
- 交差点での右左折 route following
- merge / lane drop

理由:

- 面白いが、scenario 設計と評価が少し重くなる
- `P0/P1` が安定してからのほうが全体設計を崩しにくい

## 後回しにするもの

- 歩行者の急な飛び出し
- 複雑な pedestrian / cyclist interaction
- 多車線の曖昧な social negotiation

理由:

- scenario 作成コストが高い
- teacher 挙動の一貫性を保ちにくい
- 初期の `CARLA` 収集対象としては重すぎる

## expert 方針

- 不自然な「予知」ベースの行動は避ける
- 行動候補は観測と整合する範囲で決める
- 安全判定には simulator の privileged 情報を使ってよい

要するに、最初の teacher は

- `route following`
- `signal`
- `follow`

をまず固め、そのあとに

- `yellow`
- `stopped obstacle`
- `overtake`

を足していく。重い rare hazard は後段で別 task として扱う。
