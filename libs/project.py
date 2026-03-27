from __future__ import annotations

from datetime import datetime
from pathlib import Path
import subprocess


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def relative_to_project(path: Path) -> str:
    return str(path.resolve().relative_to(PROJECT_ROOT))


def current_git_commit_short(length: int = 12) -> str:
    completed = subprocess.run(
        ["git", "rev-parse", f"--short={length}", "HEAD"],
        cwd=PROJECT_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return completed.stdout.strip()


def ensure_clean_git_worktree(*, action_label: str = "Run") -> str:
    completed = subprocess.run(
        ["git", "status", "--porcelain=v1"],
        cwd=PROJECT_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    status_lines = [line for line in completed.stdout.splitlines() if line.strip()]
    if status_lines:
        preview = "\n".join(status_lines[:20])
        if len(status_lines) > 20:
            preview = f"{preview}\n..."
        raise SystemExit(
            f"{action_label} requires a clean git worktree. Commit or stash your changes before running.\n"
            f"Pending changes:\n{preview}"
        )
    return current_git_commit_short()


def ensure_clean_git_worktree_for_evaluation() -> str:
    return ensure_clean_git_worktree(action_label="Evaluation")


def build_versioned_run_id(prefix: str, *, commit_id: str) -> str:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{prefix.lower()}_{timestamp}_{commit_id}"
