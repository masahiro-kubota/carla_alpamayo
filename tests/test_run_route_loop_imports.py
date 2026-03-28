from __future__ import annotations

import unittest

from simulation.pipelines.run_route_loop import run


class RunRouteLoopImportsTest(unittest.TestCase):
    def test_run_symbol_is_callable(self) -> None:
        self.assertTrue(callable(run))


if __name__ == "__main__":
    unittest.main()
