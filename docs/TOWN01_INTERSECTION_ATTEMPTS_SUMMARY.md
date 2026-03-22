# Town01 Intersection Attempts Summary

2026-03-22 時点の `Town01` intersection generalization 試行の要約。

## Goal

main goal は [TOWN01_INTERSECTION_GOAL.md](./TOWN01_INTERSECTION_GOAL.md) のとおり、`Town01` の任意交差点で valid な `LEFT / RIGHT / STRAIGHT` を通せること。

制約:

- input は `front RGB + speed + command`
- `target-point` は mainline で不採用
- longitudinal は `BasicAgent` / PID
- non-essential な route-specific hack は不採用

## What Was Solved

intersection generalization の前段として、fixed loop は mainline で解けた。

- accepted loop checkpoint: `outputs/train/pilotnet_branch_fs3_20260321_231852/best.pt`
- accepted loop eval: `outputs/evaluate/town01_pilotnet_loop_pilotnet_eval_20260321_232707/summary.json`
- result: `route_completion_ratio = 0.9991`, `collision_count = 0`, `success = true`

## Movement Suite Definition

inventory と suite は [TOWN01_MOVEMENT_COVERAGE.md](./TOWN01_MOVEMENT_COVERAGE.md) のとおり。

- unique movement inventory: `72`
- breakdown: `LEFT = 24`, `RIGHT = 24`, `STRAIGHT = 24`
- current eval suite size: `72`

## Chronology

### Stage 1: Initial Full-Suite Baselines

主な結果:

- `outputs/evaluate_suites/town01_movement_all_fs3_movementall_20kmh_20260322_1655.json`: `54 / 72`
- `outputs/evaluate_suites/town01_movement_all_fs3_movementall20_20kmh_20260322_1748.json`: `56 / 72`

解釈:

- fixed-loop で効いたデータ recipe だけでは、Town01 全交差点には足りない
- failure は特定の right-family と一部 left-family に集中

### Stage 2: Hard-Failure Replay

hard failure subset を切り出して correction を追加。

主な結果:

- `outputs/evaluate_suites/town01_movement_fail_fs3_hardx3_20260322_1847.json`: `1 / 16`
- `outputs/evaluate_suites/town01_movement_fail_fs3_hardx3_corr1_20260322_1920.json`: `6 / 16`
- `outputs/evaluate_suites/town01_movement_all_fs3_hardx3_corr1_20kmh_20260322_1920.json`: `61 / 72`

解釈:

- targeted replay は効いた
- ただし broad suite の ceiling は `61 / 72` で止まった

### Stage 3: Command-Head Variants And Hold Variants

試したもの:

- lane-follow head only tuning
- command hold variants
- all-head rehearsal

主な結果:

- `outputs/evaluate_suites/town01_movement_all_fs3_corr1_lfhead_20kmh_20260322_1938.json`: `52 / 72`
- `outputs/evaluate_suites/town01_movement_fail_fs3_corr1_hold15_20260322_1920.json`: `1 / 11`
- `outputs/evaluate_suites/town01_movement_all_fs3_allheads_exact11x5_20kmh_20260322_2024.json`: `55 / 72`

解釈:

- 小手先の head 切り替えや hold 調整では改善しない
- right-family の失敗を局所的に直しても broad coverage を壊しやすい

### Stage 4: Exact Failure Rehearsal

exact failure 11 本と anchor 5 本を clean expert で再収集。

主な結果:

- `outputs/evaluate_suites/town01_movement_fail_fs3_exact11x5_clean_only_20260322_2040.json`: `4 / 11`
- `outputs/evaluate_suites/town01_movement_all_fs3_exact11x5_clean_only_20kmh_20260322_2040.json`: `60 / 72`
- `outputs/evaluate_suites/town01_movement_fail_fs3_exact11_anchor5_clean_only_20260322_2120.json`: `4 / 11`
- `outputs/evaluate_suites/town01_movement_all_fs3_exact11_anchor5_clean_only_20kmh_20260322_2120.json`: `61 / 72`

解釈:

- exact rehearsal でも ceiling は `61 / 72`
- 失敗 movement は入れ替わるが、全体値は伸びない

### Stage 5: Targeted Multi-Speed Context Expansion

repair16 exact routes と support-context routes を、`18 / 20 / 22 km/h` で追加収集。

収集量:

- targeted clean manifests: `192`
- 代表的な狙い: `right_j110`, `right_j143`, `right_j222`, `right_j255`, `right_j278`, `left_j87`, `straight_j171`

flat fine-tune の結果:

- `outputs/evaluate_suites/town01_movement_repair16_probe_fs3_repair16_multispeed_context_20260322_2238.json`: `4 / 16`

解釈:

- targeted data 自体は増えた
- ただし flat `fs3` では targeted data を混ぜると broad behavior が崩れやすい

### Stage 6: Temporal Model Upgrade

model 側に temporal fusion を追加。

実装:

- `frame_stack = 5`
- `temporal_fusion = gru`
- conv trunk を frame ごとに通し、GRU で時系列融合

最初の GRU 結果:

- train: `outputs/train/pilotnet_branch_fs5_gru_exact11_anchor5_20260322_2258/summary.json`
- probe: `outputs/evaluate_suites/town01_movement_repair16_probe_fs5_gru_exact11_anchor5_20260322_2258.json`: `7 / 16`
- full: `outputs/evaluate_suites/town01_movement_all_fs5_gru_exact11_anchor5_20kmh_20260322_2258.json`: `35 / 72`

解釈:

- GRU 自体は hard subset では効いた
- ただし full 72 では broad coverage が大きく落ちた

### Stage 7: GRU + Multi-Speed Context

現在の最新候補:

- train: `outputs/train/pilotnet_branch_fs5_gru_repair16_multispeed_context_20260322_2330/summary.json`
- probe: `outputs/evaluate_suites/town01_movement_repair16_probe_fs5_gru_repair16_multispeed_context_20260322_2330.json`: `8 / 16`
- full: `outputs/evaluate_suites/town01_movement_all_fs5_gru_repair16_multispeed_context_20kmh_20260322_2330.json`: `40 / 48`

この run の特徴:

- `fs5 + GRU`
- base `600` manifests
- targeted multispeed/support-context `192` manifests
- total frame count: `452127`

途中評価:

- repair16 probe は `7 / 16 -> 8 / 16` に改善
- ただし current eval suite では `40 / 48` で止まり、best 更新には至らなかった

## Best Result So Far

2026-03-22 時点の best full-suite result はこれ。

- `outputs/evaluate_suites/town01_movement_all_fs3_hardx3_corr1_20kmh_20260322_1920.json`
- success: `61 / 72`

同率の別 run:

- `outputs/evaluate_suites/town01_movement_all_fs3_exact11_anchor5_clean_only_20kmh_20260322_2120.json`

48-route eval suite ベースの best はこれ。

- `outputs/evaluate_suites/town01_movement_full_intersection_20260322_1240.json`
- success: `41 / 48`

つまり、

- fixed loop は mainline で解けた
- Town01 arbitrary intersection goal は未達
- ceiling はいまのところ `61 / 72`

## Reflection

今回の進め方には、はっきりした反省点がある。

### 1. Goal に対して data design が後手だった

最終目標は `Town01` の arbitrary intersection coverage だったが、初期の data collection は fixed-loop 完走を主眼にしたものだった。

そのため、

- まず loop を通すための route family を追加し
- 次に壊れた箇所を局所的に補修し
- さらに別の failure に応じて route を増築する

という進め方になった。

これは fixed-loop を解くには有効だったが、`72 movement` 全体を最初から均等に被覆する設計ではなかった。

### 2. 総量ではなく coverage を先に揃えるべきだった

振り返ると問題は「データ量不足」だけではなく、「データ分布が局所 route family に偏っていたこと」だった。

best run の学習データは約 `3.95 時間` あるが、その内訳はかなり不均質だった。

- `movement_short_routes`: 約 `99.8 分`
- `left_focus`: 約 `38.5 分`
- `right_focus_upper_band`: 約 `22.7 分`
- `right_focus_other`: 約 `17.7 分`
- `fixed loop` 系: 約 `31.4 分`
- correction replay: 約 `2.5 分` 相当

つまり「最初から Town01 全交差点を coverage-first で広く集めた」のではなく、「失敗した family を順次厚くした」構成になっていた。

### 3. targeted correction 自体は間違いではないが、順番が悪かった

hard failure replay や correction replay は、imitation learning / DAgger 系では普通の手段であり、それ自体は不適切ではない。

ただし今回は、

- inventory / movement coverage の基盤が十分でない段階で
- targeted replay を強く回した

ため、局所改善はしても broad suite の天井が伸びにくかった。

本来の順番は、

1. Town01 の movement inventory を先に固定する
2. 各 `LEFT / RIGHT / STRAIGHT` を広く clean expert で集める
3. その baseline を学習する
4. その後で hard failures に対して correction を足す

だった。

### 4. route family の増築で experiment management が悪化した

`right_focus_*`, `left_focus_*`, `upper_band_*`, `curve_focus_*`, `movement_*`, correction windows という形で collection family が増え続けた結果、

- どの family が mainline か分かりにくくなった
- best model の data recipe が複雑になった
- reproduction のために守るべき correction manifest を消してしまった

という運用上の問題も生んだ。

これは単なる整理不足ではなく、`goal` と `data recipe` の関係が文書化されないまま experiment を積み増したことが原因だった。

### 5. 小手先の tuning に時間を使いすぎた

途中で試した

- command hold
- head-only tuning
- per-command checkpoint 差し替え
- 局所 route 向けの特殊 fine-tune

の多くは、最終 goal に対して本質的ではなかった。

この時間を、

- movement coverage の初期収集
- eval suite 全体の systematic collection
- route family ごとの train/eval 分離

に使うべきだった。

## What Should Have Been Done Instead

今回の goal に対して、最初からやり直すなら次の順で進めるべきだった。

1. `Town01` の全 valid movement を inventory 化する
2. 各 movement に対して train / held-out eval route を先に固定する
3. 各 movement を最低本数ずつ clean expert で収集する
4. first baseline を広い coverage で 1 回学習する
5. suite 全体を評価し、残った hard failures にだけ targeted correction を足す

要するに、

- fixed-loop のための recipe を育ててから arbitrary intersection に拡張する

のではなく、

- arbitrary intersection の評価設計を先に作り、その coverage に沿ってデータを集める

べきだった。

## What Helped

- hard failure route を明示的に切り出すこと
- failure point だけではなく、その手前の approach context を増やすこと
- right-family failure を isolated route で評価すること
- frame stack を増やして時系列を見ること

## What Did Not Help Enough

- command hold の調整
- head-only の小修正
- lane-follow head だけの tuning
- exact failure だけを増やすこと
- flat `fs3` に targeted multispeed correction をそのまま混ぜること

## Current Diagnosis

主ボトルネックは依然として right-family。

特に不安定な movement:

- `right_j110_68_238`
- `right_j143_92_37`
- `right_j222_107_6`
- `right_j255_83_23`
- `right_j278_99_133`
- `right_j54_116_14`

加えて、一部 left-family は collision ではなく stall / completion 不足で落ちる。

代表例:

- `left_j222_95_109`

## Exit Condition For This Effort

この effort を「成功」と呼ぶ条件:

- full `72 / 72` suite で success

この effort を「今回は失敗」と整理する条件:

- 最新候補が `61 / 72` を超えない
- かつ追加の本質的変更なしでは同じ失敗を繰り返すだけと判断できる

2026-03-23 00:xx の結論:

- 最新候補 `fs5 + GRU + targeted multispeed/support-context` は hard subset では改善した
- しかし full eval では `40 / 48` に留まり、既存 best `41 / 48` を超えられなかった
- この turn の範囲では goal 未達のまま終了
