from __future__ import annotations

import unittest

from tests.integration.ad_stack.stopped_obstacle.doc_builder import generate_suite_docs


class StoppedObstacleSuiteDocsTests(unittest.TestCase):
    def test_generated_docs_match_checked_in_files(self) -> None:
        for path, expected_content in generate_suite_docs().items():
            self.assertTrue(path.exists(), f"missing generated suite doc: {path}")
            self.assertEqual(path.read_text(encoding="utf-8"), expected_content, str(path))


if __name__ == "__main__":
    unittest.main()
