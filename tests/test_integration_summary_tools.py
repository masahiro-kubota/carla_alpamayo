from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from tests.integration.ad_stack._shared import (
    load_manifest,
    load_summary,
    require,
    summary_path_from_run_output,
)


class IntegrationSummaryToolsTests(unittest.TestCase):
    def test_summary_path_from_run_output(self) -> None:
        path = summary_path_from_run_output(json.dumps({"episode_id": "ep123"}))
        self.assertEqual(path, Path("outputs/evaluate/ep123/summary.json"))

    def test_load_summary_and_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest_path = root / "manifest.jsonl"
            manifest_path.write_text('{"frame_id": 1}\n{"frame_id": 2}\n', encoding="utf-8")
            summary_path = root / "summary.json"
            summary_path.write_text(
                json.dumps({"manifest_path": str(manifest_path)}),
                encoding="utf-8",
            )

            summary = load_summary(summary_path)
            manifest = load_manifest(summary)

            self.assertEqual(manifest, [{"frame_id": 1}, {"frame_id": 2}])

    def test_require_raises_system_exit(self) -> None:
        with self.assertRaises(SystemExit):
            require(False, "failure")


if __name__ == "__main__":
    unittest.main()
