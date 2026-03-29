#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
cd "${ROOT_DIR}"

PYTHONPATH="" uv run python tests/integration/ad_stack/stopped_obstacle/run_suite.py "$@"
