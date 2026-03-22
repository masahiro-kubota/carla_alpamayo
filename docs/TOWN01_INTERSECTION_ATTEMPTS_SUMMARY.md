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
