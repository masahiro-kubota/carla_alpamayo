# carla_alpamayo

This is the `CARLA`-side workspace for `alpamayo` / `minipamayo`.
Its primary purpose is to collect expert route-loop data used for `minipamayo` training and evaluation, and to develop the policy / scenario / artifact tooling that supports that workflow. The `PilotNet` code in this repository is kept as a small sample for exercising the training flow in a minimal setup.

This is a minimal repository for running the following on `CARLA`:

- expert route-loop execution
- `PilotNet` training as a lightweight learning example
- closed-loop evaluation of trained checkpoints
- interactive test driving with manual commands

The current execution architecture is summarized in [docs/DIRECTORY_RELATIONSHIPS.md](docs/DIRECTORY_RELATIONSHIPS.md). `simulation/` is a thin CLI wrapper, and the main execution path is centralized in `ad_stack.run(request)`.
Future privileged expert policy requirements, including traffic-light compliance and overtaking, are documented in [docs/EXPERT_POLICY_REQUIREMENTS.md](docs/EXPERT_POLICY_REQUIREMENTS.md).
Detailed internal design is documented in [docs/EXPERT_POLICY_DESIGN.md](docs/EXPERT_POLICY_DESIGN.md).

## Requirements

- `uv` must be available
- `CARLA 0.9.16` must be accessible at `~/sim/carla-0.9.16`
- Python 3.10 or newer

## Setup

```bash
cd /home/masa/carla_alpamayo
uv sync
```

The `carla` wheel is referenced from `pyproject.toml`. If you move the `CARLA` installation, update the wheel path as well.

## Starting CARLA

```bash
cd ~/sim/carla-0.9.16
export DISPLAY=:1
./CarlaUE4.sh -quality-level=Low -RenderOffScreen -nosound
```

## Parallel Collection

On this machine, measuring a `60s` expert route loop with settings equivalent to `policy.ignore_traffic_lights=true` and `artifacts.record_video=false` showed that the practical upper limit for data collection was `3` parallel runs.

- `1` parallel run: about `2.67x realtime`
- `2` parallel runs: about `4.42x realtime`
- `3` parallel runs: about `4.85x realtime`
- `4` parallel runs: about `3.30x realtime`

For long-running collection, `3` parallel runs are therefore recommended. At `4` parallel runs, GPU usage and VRAM usage on the `RTX 4070 Ti 12GB` were nearly saturated, and aggregate throughput dropped instead.

When launching additional independent CARLA servers, do not only shift the `RPC port`; also pass `-carla-streaming-port=0`. In this environment, the following port families were stable:

- `2000` / `2002`
- `2004` / `2006`
- `2008` / `2010`

Launch example:

```bash
cd ~/sim/carla-0.9.16
export DISPLAY=:1

./CarlaUE4.sh -quality-level=Low -RenderOffScreen -nosound
./CarlaUE4.sh -quality-level=Low -RenderOffScreen -nosound -carla-rpc-port=2004 -carla-streaming-port=0
./CarlaUE4.sh -quality-level=Low -RenderOffScreen -nosound -carla-rpc-port=2008 -carla-streaming-port=0
```

Assign `runtime.port` in each run config JSON to `2000`, `2004`, and `2008` respectively.

## Expert Route Loop

Route-loop execution is JSON-only. Do not use CLI overrides or environment-variable overrides; put all configuration in `simulation/run_configs/*.json`.

Single expert run:

```bash
cd /home/masa/carla_alpamayo
PYTHONPATH="" uv run python -m simulation.pipelines.run_route_loop \
  simulation/run_configs/town01_perimeter_cw_expert.json
```

Internally, `simulation.pipelines.run_route_loop` builds `RunRequest(mode="evaluate", policy.kind=..., ...)` from the run config JSON and then calls `ad_stack.run(...)`.
Routes are managed in `scenarios/routes/*.json`, surrounding environment and stopping conditions in `scenarios/environments/*.json`, and ego-side expert policy thresholds in `ad_stack/configs/expert/*.json`.

Main outputs:

- manifest: `outputs/evaluate/<run_id>/manifest.jsonl`
- summary: `outputs/evaluate/<run_id>/summary.json`
- video: `outputs/evaluate/<run_id>/front_rgb.mp4`
- mcap: `outputs/evaluate/<run_id>/telemetry/segment_0000.mcap`
- raw argv: `outputs/evaluate/<run_id>/cli_args.json`
- input run config: `outputs/evaluate/<run_id>/run_config.json`
- resolved request: `outputs/evaluate/<run_id>/run_request.json`

To run three in parallel, just pass three config JSON files:

```bash
cd /home/masa/carla_alpamayo
PYTHONPATH="" uv run python -m simulation.pipelines.run_route_loop \
  simulation/run_configs/town01_perimeter_cw_collection.json \
  simulation/run_configs/town01_intersection_weave_cw_collection.json \
  simulation/run_configs/town01_intersection_weave_ccw_collection.json
```

In this case, the original command plus each worker's config / log / exit code are recorded in `outputs/launcher_runs/<run_id>/launcher_manifest.json`.

## Route Preview

Because `scenarios/routes/*.json` is hard for humans to read as a list of anchor indices alone, keep a preview PNG with the same name alongside each route JSON under `scenarios/routes/previews/`.

To regenerate all route previews:

```bash
cd /home/masa/carla_alpamayo
PYTHONPATH="" uv run python -m simulation.pipelines.plot_route_map --all
```

To update only one route:

```bash
cd /home/masa/carla_alpamayo
PYTHONPATH="" uv run python -m simulation.pipelines.plot_route_map \
  --route-config scenarios/routes/town01_perimeter_cw.json
```

Output goes to `scenarios/routes/previews/<route_name>.png`.

To regenerate the town-wide top-down map asset:

```bash
cd /home/masa/carla_alpamayo
PYTHONPATH="" uv run python -m simulation.pipelines.render_town_topdown_asset --town Town01
```

Outputs go to `scenarios/maps/town01_topdown.png` and `scenarios/maps/town01_topdown.json`.
This `town01_topdown.png` is not a lane diagram. It is an RGB bird's-eye image of the entire town captured from directly above.

## Training

Training `PilotNet`:

```bash
cd /home/masa/carla_alpamayo
./learning/scripts/run_train_pilotnet.sh
```

To try `front RGB + speed + command -> steer`:

```bash
cd /home/masa/carla_alpamayo
PILOTNET_COMMAND_CONDITIONING=embedding ./learning/scripts/run_train_pilotnet.sh
```

Training artifacts are written to `outputs/train/<run_id>/{config.json,summary.json,best.pt}`.

## Evaluation

Closed-loop evaluation works the same way: pass one learned run config JSON that includes a checkpoint.

`simulation.pipelines.run_route_loop` requires a clean git worktree. Outputs are written to `outputs/evaluate/<date>_<time>_<memo>_<commit>/`.
By default, it writes segmented MCAP starting from `telemetry/segment_0000.mcap`, records front-camera JPEGs, `/ego/state`, `/ego/control`, `/ego/planning`, `/tf`, and Foxglove `SceneUpdate` static route / lane centerline data. If a town-specific top-down asset exists under `scenarios/maps/`, `/map/topdown/compressed` and `/map/topdown/camera_info` are also inserted at the start of each segment. The segment list is written to `telemetry/index.json`, and the default split interval is `600s`. If needed, change it with `artifacts.mcap_segment_seconds` in the run config JSON. By default, the map covers the entire town; if that is too heavy, set `artifacts.mcap_map_scope` to `near_route`. If you do not need MCAP output, set `artifacts.record_mcap` to `false`. Numbered PNGs under `front_rgb/` are not kept permanently; `front_rgb.mp4` is generated from temporary frames only when `artifacts.record_video=true`.
The default front camera is `1280x720`, and the artifact recording rate is `10Hz`. If needed, adjust `runtime.camera_width`, `runtime.camera_height`, and `artifacts.record_hz` in the run config JSON.
The raw argv used for route-loop execution is saved as `cli_args.json`, the input run config as `run_config.json`, and the resolved request as `run_request.json` in the output directory.
Expert policy settings are recorded in `summary.json` as `expert_config_path` and `expert_config`.

To display the live front camera, set `preview.show_front_camera` to `true` in the run config JSON and set `DISPLAY`.

Interactive test drive:

```bash
cd /home/masa/carla_alpamayo
export DISPLAY=:1
PYTHONPATH="" uv run python -m simulation.pipelines.interactive_command_drive
```

Controls:

- `w`: `lanefollow`
- `a`: `left`
- `s`: `straight`
- `d`: `right`
- `q`: quit

## Current Responsibility Split

- `simulation/`
  - CLI entrypoints for collect / evaluate / interactive
  - builds `RunRequest`
- `ad_stack/`
  - provides the `run(request)` single entrypoint
  - owns CARLA world setup, actor / sensor lifecycle, simulation loop, and artifact output
  - stores ego expert policy thresholds in `configs/expert/*.json`
- `learning/`
  - `PilotNet` model / training / inference runtime
- `libs/`
  - route helpers, schemas, project helpers

## Repository Layout

```text
carla_alpamayo/
  ad_stack/
    configs/
      expert/
  data/
    manifests/
  docs/
    AD_STACK_SINGLE_ENTRYPOINT_PLAN.md
    DIRECTORY_RELATIONSHIPS.md
  learning/
    libs/
    pipelines/
    scripts/
  libs/
    carla_utils/
    project.py
    schemas/
    utils/
  outputs/
    collect/
    evaluate/
    train/
  simulation/
    pipelines/
    scripts/
  scenarios/
    environments/
    npc_profiles/
    routes/
```
