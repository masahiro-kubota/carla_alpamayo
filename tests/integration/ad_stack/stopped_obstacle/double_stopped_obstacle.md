# Double Stopped Obstacle Family

停止障害物が 2 台以上あるケースの family index です。

この family は 1 本の scenario にまとめると acceptance が曖昧になるので、今後は次の 2 本に分けます。

- [double_stopped_separated.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/double_stopped_separated.md)
- [double_stopped_clustered.md](/home/masa/carla_alpamayo/tests/integration/ad_stack/stopped_obstacle/double_stopped_clustered.md)

## Rationale

- `separated`
  - 1 台目を抜いたら一度 rejoin し、2 台目を別イベントとして再取得する
- `clustered`
  - 2 台が近接していて、1 つの `obstacle cluster` として同時に追い越す

この分割により、code path は

- `single_actor`
- `obstacle_cluster`

の 2 形態だけで維持できる。

## Legacy Asset

現在の legacy run-config はこれです。

- `tests/integration/ad_stack/stopped_obstacle/run_configs/town01_stopped_obstacle_double_stopped_long_expert.json`

これは `separated / clustered` を混在させた暫定 asset なので、baseline acceptance には使わない。
