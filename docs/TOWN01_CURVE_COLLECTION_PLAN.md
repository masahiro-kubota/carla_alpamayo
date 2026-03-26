# Town01 High-Curvature Lanefollow Collection Plan

Note: this is a historical note. The auxiliary route configs and batch scripts described here are not kept in the current minimal `data_collection/` layout.

## Why this pack exists

The current `front RGB + speed + command + target-point -> steer` policy still fails because `command` switches back to `lanefollow` before some of the hardest exit curves are over. In the clean expert set, `lanefollow` with `|steer| >= 0.7` is almost absent, so the model has very little supervision for "keep curving after the junction decision is already made."

This pack targets those missing scenes directly. It does not simplify the evaluation route or add route-progress-dependent logic. It only increases expert coverage for high-curvature `lanefollow` segments that already exist on the fixed loop.

## Hotspots

| Route | Anchors | Purpose |
| --- | --- | --- |
| [`town01_curve_focus_early_left_exit`](./assets/curve_routes/town01_curve_focus_early_left_exit.png) | `222 -> 166` | Early west-side left branch followed by the strong southbound exit curve where the latest route-conditioned run failed. |
| [`town01_curve_focus_late_right_exit`](./assets/curve_routes/town01_curve_focus_late_right_exit.png) | `170 -> 54` | Late northwest right branch plus the strong exit curve near the previous `0.90`-completion failure. |
| [`town01_curve_focus_bottom_right`](./assets/curve_routes/town01_curve_focus_bottom_right.png) | `118 -> 183` | Bottom-right perimeter curve with sustained positive `lanefollow` steering. |
| [`town01_curve_focus_top_right`](./assets/curve_routes/town01_curve_focus_top_right.png) | `174 -> 244` | Upper-right turn continuation with right-turn intent and positive exit curvature. |
| [`town01_curve_focus_top_left_entry`](./assets/curve_routes/town01_curve_focus_top_left_entry.png) | `52 -> 67` | Top-left perimeter entry curve with sustained negative `lanefollow` steering. |
| [`town01_curve_focus_top_left_exit`](./assets/curve_routes/town01_curve_focus_top_left_exit.png) | `52 -> 119` | Top-left perimeter exit curve with sustained positive `lanefollow` steering. |
| [`town01_curve_focus_lower_left`](./assets/curve_routes/town01_curve_focus_lower_left.png) | `214 -> 205` | Lower-left perimeter curve with sustained positive `lanefollow` steering. |

## Collection policy

- Use the existing expert collector with `BasicAgent`.
- Keep the same low-resolution setup: `320x180`, `30 km/h`, `ClearNoon`.
- Collect repeated short runs instead of only one full loop, because the missing distribution is local and rare.
- Default batch size: `5` repetitions per route via [run_collect_town01_curve_pack.sh](/home/masa/carla_alpamayo/data_collection/scripts/run_collect_town01_curve_pack.sh).

## Success criterion for this pack

Before retraining, the clean expert set should have materially more high-curvature `lanefollow` frames than the current baseline, especially for `|steer| >= 0.5` and `|steer| >= 0.7`.

## Result

- batch run: `35 / 35` success
- `lanefollow` with `|steer| >= 0.5`: `90 -> 203`
- `lanefollow` with `|steer| >= 0.7`: `19 -> 36`
- this pack was used in `outputs/train/pilotnet_cmd_tp_20260321_075034/`
- the resulting learned policy succeeded on the fixed loop at [summary.json](/home/masa/carla_alpamayo/outputs/evaluate/town01_pilotnet_loop_pilotnet_eval_20260321_075722/summary.json)
