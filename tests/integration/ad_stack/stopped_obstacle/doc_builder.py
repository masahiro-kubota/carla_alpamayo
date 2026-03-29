from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Literal
import sys

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
    from tests.integration.ad_stack._shared import ManifestExpectation, ScenarioSummaryExpectation
    from tests.integration.ad_stack.stopped_obstacle.scenario_matrix import (
        INSPECT_ONLY_SCENARIOS,
        ROUTE_LOOP_SCENARIOS,
        SUITE_DIR,
    )
else:
    from tests.integration.ad_stack._shared import ManifestExpectation, ScenarioSummaryExpectation
    from .scenario_matrix import INSPECT_ONLY_SCENARIOS, ROUTE_LOOP_SCENARIOS, SUITE_DIR


DocGroup = Literal["verified", "inspect_only", "exploratory"]


@dataclass(frozen=True, slots=True)
class ScenarioDocSeed:
    name: str
    title: str
    group: DocGroup
    summary: str
    contract: tuple[str, ...]
    target_actor: tuple[str, ...] = ()
    reject_wait: tuple[str, ...] = ()
    pass_rejoin: tuple[str, ...] = ()
    why_matters: tuple[str, ...] = ()
    planned_run_config_name: str | None = None


_ROUTE_LOOP_DOCS: tuple[ScenarioDocSeed, ...] = (
    ScenarioDocSeed(
        name="clear",
        title="Clear Scenario",
        group="verified",
        summary="停止障害物だけが前方にいて、隣接 lane が空いている場合の baseline です。",
        contract=(
            "same-lane に停止障害物が 1 台",
            "adjacent lane は clear",
            "route 完走まで十分な corridor がある",
        ),
        target_actor=(
            "`follow_target_id` は停止障害物 actor に収束する",
            "run 中に別 actor へふらつかない",
        ),
        reject_wait=(
            "trigger 距離に入るまでは `target_out_of_range` で抑制される",
            "trigger 後は `lane_change_out` に遷移する",
        ),
        pass_rejoin=(
            "`lane_change_out -> pass_vehicle -> lane_change_back -> nominal_cruise`",
            "target を抜いたあと元 lane に戻る",
        ),
        why_matters=(
            "停止障害物回避の最小 baseline です。",
        ),
    ),
    ScenarioDocSeed(
        name="blocked_static",
        title="Blocked Static Scenario",
        group="verified",
        summary="停止障害物はあるが、隣接 lane に静的 blocker がいて出られない scenario です。",
        contract=(
            "same-lane に停止障害物が 1 台",
            "adjacent lane front gap を静的 blocker が塞ぐ",
        ),
        target_actor=(
            "`follow_target_id` は停止障害物 actor に収束する",
            "blocker actor を追い越し target と取り違えない",
        ),
        reject_wait=(
            "`adjacent_front_gap_insufficient` または等価な gap reject で抑制される",
            "`lane_change_out` に入らない",
        ),
        pass_rejoin=(
            "route-loop は stall で終わる",
            "collision は起こさない",
        ),
        why_matters=(
            "gap 不足で出ないことを保証する negative baseline です。",
        ),
    ),
    ScenarioDocSeed(
        name="blocked_oncoming",
        title="Blocked Oncoming Scenario",
        group="verified",
        summary="対向車がいる間は出ず、通過後に追い越す scenario です。",
        contract=(
            "same-lane に停止障害物が 1 台",
            "adjacent lane に oncoming actor が一時的に存在する",
        ),
        target_actor=(
            "`follow_target_id` は停止障害物 actor に収束する",
            "oncoming actor を追い越し target と取り違えない",
        ),
        reject_wait=(
            "対向車が近い間は lane gap reject が立つ",
            "gap 解放後に `lane_change_out` へ入る",
        ),
        pass_rejoin=(
            "待機後に 1 回の追い越しで完走する",
        ),
        why_matters=(
            "“待ってから抜く” を見る baseline です。",
        ),
    ),
    ScenarioDocSeed(
        name="double_stopped_separated",
        title="Double Stopped Separated Scenario",
        group="verified",
        summary="停止障害物が離れて 2 台あり、1 台目を抜いて rejoin した後に 2 台目を再取得する scenario です。",
        contract=(
            "same-lane に停止障害物が 2 台",
            "相互 gap は `cluster_merge_gap_m` より十分大きい",
        ),
        target_actor=(
            "1 台目を追い越し target として取得し、その後 2 台目へ切り替わる",
        ),
        reject_wait=(
            "2 台目のために cluster 扱いへ崩れない",
        ),
        pass_rejoin=(
            "1 台目で `lane_change_out -> pass_vehicle -> lane_change_back`",
            "その後 2 台目に対して再度同じ flow が発生する",
        ),
        why_matters=(
            "multi-obstacle を single-target の積み重ねとして扱えることを確認します。",
        ),
    ),
    ScenarioDocSeed(
        name="double_stopped_clustered",
        title="Double Stopped Clustered Scenario",
        group="verified",
        summary="停止障害物が近接し、1 つの `cluster` として同時に追い越す scenario です。",
        contract=(
            "same-lane に停止障害物が 2 台以上",
            "相互 gap は `cluster_merge_gap_m` 以下",
            "adjacent lane は clear",
        ),
        target_actor=(
            "`overtake_target_kind = cluster` を維持する",
            "`overtake_target_member_actor_ids` は複数 actor を保持する",
        ),
        reject_wait=(
            "cluster 途中で別 target へ切り替えず corridor を維持する",
        ),
        pass_rejoin=(
            "cluster 全体に対して 1 回の `lane_change_out -> pass_vehicle -> lane_change_back` で終える",
        ),
        why_matters=(
            "複数停止車両の同時追い越しを state machine 肥大化なしで扱えるかを見る scenario です。",
        ),
    ),
    ScenarioDocSeed(
        name="signal_suppressed",
        title="Signal Suppressed Scenario",
        group="verified",
        summary="停止障害物はあるが、近い赤信号を優先して追い越しを開始しない scenario です。",
        contract=(
            "same-lane に停止障害物が 1 台",
            "近傍 signal が red で、suppression distance 内にある",
        ),
        target_actor=(
            "`follow_target_id` は停止障害物 actor に収束する",
        ),
        reject_wait=(
            "`signal_suppressed` で抑制される",
            "`lane_change_out` に入らない",
        ),
        pass_rejoin=(
            "route-loop は stall で終わる",
        ),
        why_matters=(
            "停止障害物より信号規制を優先することを確認します。",
        ),
    ),
    ScenarioDocSeed(
        name="adjacent_lane_closed",
        title="Adjacent Lane Closed Scenario",
        group="verified",
        summary="停止障害物はあるが、隣接 lane 自体が利用不可で出られない scenario です。",
        contract=(
            "same-lane に停止障害物が 1 台",
            "route-aligned adjacent lane が `Driving` でないか corridor として利用不可",
        ),
        target_actor=(
            "`follow_target_id` は停止障害物 actor に収束する",
        ),
        reject_wait=(
            "`adjacent_lane_closed` で抑制される",
            "gap の大小ではなく lane availability で reject される",
        ),
        pass_rejoin=(
            "route-loop は stall で終わる",
        ),
        why_matters=(
            "gap 不足と lane unavailable を分けて扱えているかを見る scenario です。",
        ),
    ),
    ScenarioDocSeed(
        name="curve_clear",
        title="Curve Clear Scenario",
        group="verified",
        summary="緩いカーブ上の停止障害物を clear 条件で追い越す scenario です。",
        contract=(
            "route 進行先の curve 上に停止障害物がある",
            "adjacent lane は clear",
        ),
        target_actor=(
            "same-lane でまだ見えていなくても route-aligned target として観測できる",
        ),
        reject_wait=(
            "curve 進入前後で target acquisition が崩れない",
        ),
        pass_rejoin=(
            "カーブでも `lane_change_out -> pass_vehicle -> lane_change_back` が成立する",
        ),
        why_matters=(
            "直線 corridor だけに過学習していないことを確認します。",
        ),
    ),
    ScenarioDocSeed(
        name="rejoin_blocked_then_release",
        title="Rejoin Blocked Then Release Scenario",
        group="verified",
        summary="追い越し後、rejoin gap が一時的に不足し、解放後に戻る scenario です。",
        contract=(
            "same-lane に停止障害物が 1 台",
            "元 lane 側に moving blocker がいて rejoin gap を一時的に塞ぐ",
        ),
        target_actor=(
            "`follow_target_id` は停止障害物 actor に収束する",
        ),
        reject_wait=(
            "`lane_change_out` は成立する",
            "`pass_vehicle` 中、gap 不足の間は `lane_change_back` に入らない",
        ),
        pass_rejoin=(
            "target を抜いた後に positive な wait があり、gap 解放後に `lane_change_back` に入る",
        ),
        why_matters=(
            "`pass 完了` と `rejoin 開始` が gap 基準で分かれているかを見る scenario です。",
        ),
    ),
)

_INSPECT_ONLY_DOCS: tuple[ScenarioDocSeed, ...] = (
    ScenarioDocSeed(
        name="near_junction_preflight_reject",
        title="Near Junction Preflight Reject Scenario",
        group="inspect_only",
        summary="停止障害物はあるが junction / signal が近すぎるため、preflight で invalid にする scenario です。",
        contract=(
            "same-lane に停止障害物が 1 台",
            "近傍に junction または signal があり、overtake corridor として unsafe",
        ),
        reject_wait=(
            "`scenario_validation.valid = false`",
            "`junction_nearby` または `signal_nearby` を含む",
        ),
        why_matters=(
            "“走らせない” ことも suite contract に含めるための scenario です。",
        ),
    ),
)

_EXPLORATORY_DOCS: tuple[ScenarioDocSeed, ...] = (
    ScenarioDocSeed(
        name="right_first_clear",
        title="Right-First Clear Scenario",
        group="exploratory",
        summary="左右どちらも成立する状況で、`preferred_direction = right_first` を runtime で確認する scenario です。",
        contract=(
            "left / right の両方が `Driving`",
            "front / rear gap が両側とも十分",
            "`preferred_direction = right_first`",
        ),
        target_actor=(
            "`follow_target_id` は停止障害物 actor に収束する",
        ),
        reject_wait=(
            "reject は解消される",
            "`target_lane_id` は右 lane を選ぶ",
        ),
        pass_rejoin=(
            "右側へ `lane_change_out`",
            "`pass_vehicle -> lane_change_back`",
        ),
        why_matters=(
            "Town01 で自然な両側成立 corridor が見つかったら昇格する候補です。",
        ),
        planned_run_config_name="town01_stopped_obstacle_right_first_clear_long_expert.json",
    ),
    ScenarioDocSeed(
        name="temporary_target_occlusion",
        title="Temporary Target Occlusion Scenario",
        group="exploratory",
        summary="追い越し対象 actor が一時的に tracking から消えても、即 `passed` 扱いしないことを確認する scenario です。",
        contract=(
            "same-lane 停止障害物が一時的に tracked object から消える条件を作る",
            "actor 自体は存在し続ける",
        ),
        target_actor=(
            "`target_actor_last_seen_s` が保持される",
            "actor 欠落だけで target が切り替わらない",
        ),
        reject_wait=(
            "`pass_vehicle` 中に target 見失いが起きても即 `lane_change_back` しない",
        ),
        pass_rejoin=(
            "visibility timeout 前は `target_passed = false`",
            "reacquisition か actor が本当に後方へ回った時点でのみ pass 完了する",
        ),
        why_matters=(
            "runtime より unit test 主戦場ですが、将来 deterministic に作れたら integration でも押さえたい edge case です。",
        ),
        planned_run_config_name="town01_stopped_obstacle_temporary_target_occlusion_long_expert.json",
    ),
)


def generate_suite_docs() -> dict[Path, str]:
    outputs: dict[Path, str] = {
        SUITE_DIR / "README.md": render_suite_index(),
    }
    for scenario in ROUTE_LOOP_SCENARIOS:
        doc = _doc_seed(scenario.case.name, group="verified")
        outputs[SUITE_DIR / "verified" / f"{scenario.case.name}.md"] = render_route_loop_doc(
            doc,
            run_config_path=scenario.case.run_config_path,
            summary_expectation=scenario.summary_expectation,
            manifest_expectations=scenario.manifest_expectations,
        )
    for case in INSPECT_ONLY_SCENARIOS:
        doc = _doc_seed(case.name, group="inspect_only")
        outputs[SUITE_DIR / "inspect_only" / f"{case.name}.md"] = render_inspect_only_doc(
            doc,
            command=case.command,
        )
    for doc in _EXPLORATORY_DOCS:
        outputs[SUITE_DIR / "exploratory" / f"{doc.name}.md"] = render_exploratory_doc(doc)
    return outputs


def render_suite_index() -> str:
    lines: list[str] = [
        "# Stopped Obstacle Integration Suite",
        "",
        "停止障害物回避の `CARLA` 結合シナリオテスト資産です。",
        "",
        "この `README` は `scenario_matrix.py` から生成されています。suite の正本は matrix と shared assertion です。",
        "",
        "## Suite Assets",
        "",
        "- run-configs:",
        f"  - [{_link_label(SUITE_DIR / 'run_configs')}]({_link_path(SUITE_DIR / 'run_configs')})",
        "- regression runner:",
        f"  - [{_link_label(SUITE_DIR / 'run_stopped_obstacle_regressions.sh')}]({_link_path(SUITE_DIR / 'run_stopped_obstacle_regressions.sh')})",
        "- scenario inspector:",
        f"  - [{_link_label(SUITE_DIR / 'inspect_scenarios.py')}]({_link_path(SUITE_DIR / 'inspect_scenarios.py')})",
        "- scenario matrix:",
        f"  - [{_link_label(SUITE_DIR / 'scenario_matrix.py')}]({_link_path(SUITE_DIR / 'scenario_matrix.py')})",
        "",
        "## Verified Scenarios",
        "",
    ]
    for scenario in ROUTE_LOOP_SCENARIOS:
        path = SUITE_DIR / "verified" / f"{scenario.case.name}.md"
        lines.append(f"- [{path.name}]({_link_path(path)})")
    lines.extend(
        [
            "",
            f"この {len(ROUTE_LOOP_SCENARIOS)} 本が現在の verified route-loop integration set です。",
            "",
            "## Inspect-Only Scenarios",
            "",
        ]
    )
    for case in INSPECT_ONLY_SCENARIOS:
        path = SUITE_DIR / "inspect_only" / f"{case.name}.md"
        lines.append(f"- [{path.name}]({_link_path(path)})")
    lines.extend(
        [
            "",
            "この group は route-loop 成否ではなく inspector contract で検証します。",
            "",
            "## Deferred / Exploratory Scenarios",
            "",
        ]
    )
    for doc in _EXPLORATORY_DOCS:
        path = SUITE_DIR / "exploratory" / f"{doc.name}.md"
        lines.append(f"- [{path.name}]({_link_path(path)})")
    lines.extend(
        [
            "",
            "## Related Design Docs",
            "",
            f"- [STOPPED_OBSTACLE_LOGIC_DESIGN.md]({_link_path(Path('docs/STOPPED_OBSTACLE_LOGIC_DESIGN.md').resolve())})",
            f"- [STOPPED_OBSTACLE_TEST_DESIGN.md]({_link_path(Path('docs/STOPPED_OBSTACLE_TEST_DESIGN.md').resolve())})",
            f"- [STOPPED_OBSTACLE_SCENARIO_CONTRACT_DESIGN.md]({_link_path(Path('docs/STOPPED_OBSTACLE_SCENARIO_CONTRACT_DESIGN.md').resolve())})",
            f"- [STOPPED_OBSTACLE_SCENARIO_CONTRACT_TEST_DESIGN.md]({_link_path(Path('docs/STOPPED_OBSTACLE_SCENARIO_CONTRACT_TEST_DESIGN.md').resolve())})",
            f"- [NEXT_STEPS.md]({_link_path(Path('docs/stopped_obstacle/NEXT_STEPS.md').resolve())})",
            f"- [REFACTOR_PLAN.md]({_link_path(Path('docs/stopped_obstacle/REFACTOR_PLAN.md').resolve())})",
            "",
            "## Usage",
            "",
            "```bash",
            "./tests/integration/ad_stack/stopped_obstacle/run_stopped_obstacle_regressions.sh",
            "```",
            "",
            "`near_junction_preflight_reject` は route-loop summary ではなく inspector contract で検証します。",
            "",
            "```bash",
            "python tests/integration/ad_stack/stopped_obstacle/inspect_scenarios.py \\",
            "  tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_clear_long_expert.json",
            "```",
            "",
        ]
    )
    return "\n".join(lines)


def render_route_loop_doc(
    doc: ScenarioDocSeed,
    *,
    run_config_path: Path,
    summary_expectation: ScenarioSummaryExpectation,
    manifest_expectations: tuple[ManifestExpectation, ...],
) -> str:
    lines = _base_doc_header(doc)
    lines.extend(
        [
            "## Run Config",
            "",
            f"- [{run_config_path.name}]({_link_path(run_config_path)})",
            "",
            "## Scenario Contract",
            "",
        ]
    )
    lines.extend(f"- {line}" for line in doc.contract)
    lines.extend(_render_expectation_sections(doc))
    lines.extend(
        [
            "### Summary Acceptance",
            "",
        ]
    )
    lines.extend(_render_summary_acceptance(summary_expectation))
    if manifest_expectations:
        lines.extend(
            [
                "",
                "### Manifest Acceptance",
                "",
            ]
        )
        lines.extend(_render_manifest_acceptance(manifest_expectations))
    lines.extend(
        [
            "",
            "## Source Of Truth",
            "",
            f"- scenario matrix: [{_link_label(SUITE_DIR / 'scenario_matrix.py')}]({_link_path(SUITE_DIR / 'scenario_matrix.py')})",
            f"- summary / manifest assertions: [{_link_label(SUITE_DIR / 'assertions.py')}]({_link_path(SUITE_DIR / 'assertions.py')})",
            "",
        ]
    )
    return "\n".join(lines)


def render_inspect_only_doc(doc: ScenarioDocSeed, *, command: tuple[str, ...]) -> str:
    lines = _base_doc_header(doc)
    lines.extend(
        [
            "## Inspection Command",
            "",
            "```bash",
            " ".join(command),
            "```",
            "",
            "## Scenario Contract",
            "",
        ]
    )
    lines.extend(f"- {line}" for line in doc.contract)
    if doc.reject_wait:
        lines.extend(["", "## Expectations", "", "### Preflight", ""])
        lines.extend(f"- {line}" for line in doc.reject_wait)
    lines.extend(
        [
            "",
            "## Source Of Truth",
            "",
            f"- scenario matrix: [{_link_label(SUITE_DIR / 'scenario_matrix.py')}]({_link_path(SUITE_DIR / 'scenario_matrix.py')})",
            f"- inspector contract: [{_link_label((SUITE_DIR.parent / '_shared' / 'overtake_scenario_contract.py'))}]({_link_path((SUITE_DIR.parent / '_shared' / 'overtake_scenario_contract.py'))})",
            "",
        ]
    )
    return "\n".join(lines)


def render_exploratory_doc(doc: ScenarioDocSeed) -> str:
    lines = _base_doc_header(doc)
    if doc.planned_run_config_name is not None:
        planned_path = SUITE_DIR / "run_configs" / doc.planned_run_config_name
        lines.extend(
            [
                "## Planned Run Config",
                "",
                f"- `{planned_path}`",
                "",
            ]
        )
    lines.extend(["## Scenario Contract", ""])
    lines.extend(f"- {line}" for line in doc.contract)
    lines.extend(_render_expectation_sections(doc))
    lines.extend(
        [
            "### Summary Acceptance",
            "",
            "- `collision_count = 0`",
            "- `overtake_success_count >= 1`",
            "",
            "## Why Deferred",
            "",
        ]
    )
    lines.extend(f"- {line}" for line in doc.why_matters)
    lines.append("")
    return "\n".join(lines)


def write_suite_docs() -> None:
    for path, content in generate_suite_docs().items():
        path.write_text(content, encoding="utf-8")


def _base_doc_header(doc: ScenarioDocSeed) -> list[str]:
    status = "verified" if doc.group in {"verified", "inspect_only"} else "deferred"
    return [
        f"# {doc.title}",
        "",
        doc.summary,
        "",
        "## Status",
        "",
        f"- {status}",
        "",
    ]


def _render_expectation_sections(doc: ScenarioDocSeed) -> list[str]:
    lines = ["", "## Expectations", ""]
    if doc.target_actor:
        lines.extend(["### Target Actor", ""])
        lines.extend(f"- {line}" for line in doc.target_actor)
        lines.append("")
    if doc.reject_wait:
        lines.extend(["### Reject / Wait", ""])
        lines.extend(f"- {line}" for line in doc.reject_wait)
        lines.append("")
    if doc.pass_rejoin:
        lines.extend(["### Pass / Rejoin", ""])
        lines.extend(f"- {line}" for line in doc.pass_rejoin)
        lines.append("")
    if doc.why_matters:
        lines.extend(["## Why This Matters", ""])
        lines.extend(f"- {line}" for line in doc.why_matters)
        lines.append("")
    return lines


def _render_summary_acceptance(expectation: ScenarioSummaryExpectation) -> list[str]:
    lines: list[str] = []
    if expectation.success is not None:
        lines.append(f"- `success = {str(expectation.success).lower()}`")
    if expectation.failure_reason is not None:
        lines.append(f"- `failure_reason = {expectation.failure_reason}`")
    if expectation.collision_count is not None:
        lines.append(f"- `collision_count = {expectation.collision_count}`")
    if expectation.min_overtake_attempt_count is not None:
        lines.append(f"- `overtake_attempt_count >= {expectation.min_overtake_attempt_count}`")
    if expectation.exact_overtake_attempt_count is not None:
        lines.append(f"- `overtake_attempt_count = {expectation.exact_overtake_attempt_count}`")
    if expectation.min_overtake_success_count is not None:
        lines.append(f"- `overtake_success_count >= {expectation.min_overtake_success_count}`")
    if expectation.exact_overtake_success_count is not None:
        lines.append(f"- `overtake_success_count = {expectation.exact_overtake_success_count}`")
    if expectation.min_unsafe_lane_change_reject_count is not None:
        lines.append(
            f"- `unsafe_lane_change_reject_count >= {expectation.min_unsafe_lane_change_reject_count}`"
        )
    if expectation.require_positive_rejoin_wait:
        lines.append("- `rejoin_wait_after_target_passed_s > 0`")
    return lines


def _render_manifest_acceptance(expectations: tuple[ManifestExpectation, ...]) -> list[str]:
    rendered: list[str] = []
    for expectation in expectations:
        if expectation.kind == "min_unique_non_null":
            rendered.append(
                f"- `{expectation.field}` has at least {expectation.min_unique} unique non-null values"
            )
        elif expectation.kind == "any_equals":
            rendered.append(f"- some manifest row has `{expectation.field} = {expectation.expected}`")
        elif expectation.kind == "any_sequence_len_at_least":
            rendered.append(
                f"- some manifest row has `{expectation.field}` length >= {expectation.min_len}"
            )
    return rendered


def _doc_seed(name: str, *, group: DocGroup) -> ScenarioDocSeed:
    seed_sets = {
        "verified": _ROUTE_LOOP_DOCS,
        "inspect_only": _INSPECT_ONLY_DOCS,
        "exploratory": _EXPLORATORY_DOCS,
    }
    for doc in seed_sets[group]:
        if doc.name == name:
            return doc
    raise KeyError(f"missing stopped-obstacle doc seed: {name}")


def _link_label(path: Path) -> str:
    return path.name


def _link_path(path: Path) -> str:
    return str(path.resolve())


if __name__ == "__main__":
    write_suite_docs()
