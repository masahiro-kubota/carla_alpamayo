from __future__ import annotations

import os
import sys
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[4]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from tests.integration.ad_stack._shared import (
    CarlaHarness,
    CarlaHarnessConfig,
    CarlaIntegrationSuiteSpec,
    execute_carla_integration_suite,
)
from tests.integration.ad_stack.stopped_obstacle.assertions import (
    assert_near_junction_preflight_contract,
    assert_stopped_obstacle_suite,
    load_stopped_obstacle_summaries,
)
from tests.integration.ad_stack.stopped_obstacle.scenario_matrix import (
    INSPECT_ONLY_SCENARIOS,
    ROUTE_LOOP_SCENARIOS,
)
SUITE_DIR = Path(__file__).resolve().parent
SUITE_SPEC = CarlaIntegrationSuiteSpec(
    name="stopped_obstacle",
    route_loop_cases=tuple(scenario.case for scenario in ROUTE_LOOP_SCENARIOS),
    inspect_only_cases=INSPECT_ONLY_SCENARIOS,
)


def build_harness() -> CarlaHarness:
    launch_args = tuple(
        os.environ.get("CARLA_LAUNCH_ARGS", "-quality-level=Low -RenderOffScreen -nosound").split()
    )
    return CarlaHarness(
        CarlaHarnessConfig(
            root_dir=ROOT_DIR,
            carla_root=Path(
                os.environ.get("CARLA_ROOT", "/media/masa/ssd_data/sim/carla-0.9.16")
            ),
            launch_args=launch_args,
            warmup_seconds=float(os.environ.get("CARLA_WARMUP_SECONDS", "2")),
            scenario_max_attempts=int(os.environ.get("SCENARIO_MAX_ATTEMPTS", "2")),
            log_path=Path("/tmp/carla_stopped_obstacle_regression.log"),
        )
    )


def main() -> None:
    harness = build_harness()
    suite_result = execute_carla_integration_suite(harness, SUITE_SPEC)

    assert_near_junction_preflight_contract(
        suite_result.inspection_payloads["near_junction_preflight_reject"]["scenario_validation"]
    )
    assert_stopped_obstacle_suite(load_stopped_obstacle_summaries(list(suite_result.summary_paths.values())))

    print("stopped-obstacle regressions passed")
    for path in suite_result.summary_paths.values():
        print(path)


if __name__ == "__main__":
    main()
