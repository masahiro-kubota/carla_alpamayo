# VLA Scenario Scope

`CARLA` 上で `VLA` 学習用データを集めるときの、現時点のシナリオ優先度メモです。

## 優先するもの

- 信号に基づく停止・再発進
- 低速先行車への追従
- 安全な追い越し
- route following

理由:

- `CARLA` 側の実装コストが比較的低い
- expert の挙動を安定させやすい
- `VLA` に学ばせたい判断が明確

## 後回しにするもの

- 歩行者の急な飛び出し
- 複雑な pedestrian / cyclist interaction
- 多車線の曖昧な social negotiation

理由:

- scenario 作成コストが高い
- teacher 挙動の一貫性を保ちにくい
- 初期の `CARLA` 収集対象としては重すぎる

## 工数が少なく追加しやすい候補

- 停止車両の回避
- 信号 + 先行車
- 遅車両列への対応
- 黄信号の判断
- 交差点での右左折 route following

## expert 方針

- 不自然な「予知」ベースの行動は避ける
- 行動候補は観測と整合する範囲で決める
- 安全判定には simulator の privileged 情報を使ってよい

要するに、最初の teacher は

- `signal`
- `follow`
- `overtake`

を強く作ることを優先し、重い rare hazard は後段で別 task として扱う。
