from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable
from typing import Literal

from .summary_tools import require


ManifestExpectationKind = Literal[
    "min_unique_non_null",
    "any_equals",
    "none_equals",
    "any_sequence_len_at_least",
]


@dataclass(frozen=True, slots=True)
class ManifestExpectation:
    field: str
    kind: ManifestExpectationKind
    message: str
    expected: Any | None = None
    min_unique: int | None = None
    min_len: int | None = None


def non_null_field_values(rows: Iterable[dict[str, Any]], *, field: str) -> list[Any]:
    return [row[field] for row in rows if row.get(field) is not None]


def assert_min_unique_non_null_values(
    rows: Iterable[dict[str, Any]],
    *,
    field: str,
    min_unique: int,
    message: str,
) -> None:
    values = non_null_field_values(rows, field=field)
    require(len(set(values)) >= min_unique, message)


def assert_any_field_equals(
    rows: Iterable[dict[str, Any]],
    *,
    field: str,
    expected: Any,
    message: str,
) -> None:
    require(any(row.get(field) == expected for row in rows), message)


def assert_no_field_equals(
    rows: Iterable[dict[str, Any]],
    *,
    field: str,
    expected: Any,
    message: str,
) -> None:
    require(not any(row.get(field) == expected for row in rows), message)


def assert_any_sequence_len_at_least(
    rows: Iterable[dict[str, Any]],
    *,
    field: str,
    min_len: int,
    message: str,
) -> None:
    require(
        any(len(row.get(field) or []) >= min_len for row in rows),
        message,
    )


def assert_manifest_expectations(
    rows: Iterable[dict[str, Any]],
    expectations: Iterable[ManifestExpectation],
) -> None:
    materialized_rows = list(rows)
    for expectation in expectations:
        if expectation.kind == "min_unique_non_null":
            require(
                expectation.min_unique is not None,
                f"manifest expectation for {expectation.field} is missing min_unique",
            )
            assert_min_unique_non_null_values(
                materialized_rows,
                field=expectation.field,
                min_unique=expectation.min_unique,
                message=expectation.message,
            )
            continue
        if expectation.kind == "any_equals":
            assert_any_field_equals(
                materialized_rows,
                field=expectation.field,
                expected=expectation.expected,
                message=expectation.message,
            )
            continue
        if expectation.kind == "none_equals":
            assert_no_field_equals(
                materialized_rows,
                field=expectation.field,
                expected=expectation.expected,
                message=expectation.message,
            )
            continue
        if expectation.kind == "any_sequence_len_at_least":
            require(
                expectation.min_len is not None,
                f"manifest expectation for {expectation.field} is missing min_len",
            )
            assert_any_sequence_len_at_least(
                materialized_rows,
                field=expectation.field,
                min_len=expectation.min_len,
                message=expectation.message,
            )
            continue
        raise SystemExit(f"unsupported manifest expectation kind: {expectation.kind}")
