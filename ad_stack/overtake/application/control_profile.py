from __future__ import annotations


def speed_control(
    *,
    current_speed_mps: float,
    target_speed_kmh: float,
    previous_error_kmh: float,
    max_throttle: float = 0.75,
    max_brake: float = 1.0,
) -> tuple[float, float, float]:
    current_speed_kmh = current_speed_mps * 3.6
    error_kmh = target_speed_kmh - current_speed_kmh
    derivative_kmh = error_kmh - previous_error_kmh

    if error_kmh >= 0.0:
        throttle = min(
            max_throttle,
            max(0.0, (0.06 * error_kmh) + (0.01 * derivative_kmh)),
        )
        return throttle, 0.0, error_kmh

    brake = min(max_brake, max(0.0, 0.08 * (-error_kmh)))
    return 0.0, brake, error_kmh


def stopping_distance_m(
    *,
    current_speed_mps: float,
    reaction_margin_m: float,
    preferred_deceleration_mps2: float,
) -> float:
    return reaction_margin_m + (
        (current_speed_mps * current_speed_mps)
        / max(2.0 * preferred_deceleration_mps2, 1e-3)
    )


def traffic_light_stop_target_distance_m(
    *,
    distance_m: float | None,
    stop_buffer_m: float,
) -> float | None:
    if distance_m is None:
        return None
    return max(0.0, distance_m - stop_buffer_m)


def should_stop_for_light(
    *,
    ignore_traffic_lights: bool,
    light_state: str | None,
    raw_stop_distance_m: float | None,
    current_speed_mps: float,
    stop_buffer_m: float,
    reaction_margin_m: float,
    preferred_deceleration_mps2: float,
    yellow_stop_margin_seconds: float,
) -> bool:
    if ignore_traffic_lights or light_state is None or raw_stop_distance_m is None:
        return False
    if light_state == "red":
        return True
    if raw_stop_distance_m < 0.5:
        return False
    target_stop_distance = max(0.0, raw_stop_distance_m - stop_buffer_m)
    if light_state != "yellow":
        return False

    stopping_distance = stopping_distance_m(
        current_speed_mps=current_speed_mps,
        reaction_margin_m=reaction_margin_m,
        preferred_deceleration_mps2=preferred_deceleration_mps2,
    )
    margin_distance = current_speed_mps * yellow_stop_margin_seconds
    return target_stop_distance >= (stopping_distance + margin_distance)


def traffic_light_stop_target_speed_kmh(
    *,
    stop_target_distance_m: float | None,
    current_speed_mps: float,
    target_speed_kmh: float,
    brake_start_distance_m: float,
    creep_resume_distance_m: float,
    creep_speed_kmh: float,
) -> float:
    if stop_target_distance_m is None or stop_target_distance_m <= 0.0:
        return 0.0

    effective_brake_start_distance_m = max(brake_start_distance_m, 1e-3)
    if stop_target_distance_m >= effective_brake_start_distance_m:
        resolved_target_speed_kmh = target_speed_kmh
    else:
        resolved_target_speed_kmh = target_speed_kmh * max(
            0.0, min(1.0, stop_target_distance_m / effective_brake_start_distance_m)
        )

    if current_speed_mps <= 0.2 and stop_target_distance_m > creep_resume_distance_m:
        resolved_target_speed_kmh = max(resolved_target_speed_kmh, creep_speed_kmh)

    return min(target_speed_kmh, max(0.0, resolved_target_speed_kmh))


def traffic_light_stop_control(
    *,
    current_speed_mps: float,
    light_state: str | None,
    stop_target_distance_m: float | None,
    target_speed_kmh: float,
    previous_error_kmh: float,
    preferred_deceleration_mps2: float,
) -> tuple[float, float, float]:
    if light_state is None:
        return 0.0, 0.0, 0.0

    if stop_target_distance_m is None:
        return 0.0, (0.45 if current_speed_mps > 0.1 else 0.0), 0.0

    if light_state == "red":
        if stop_target_distance_m <= 0.5:
            return (
                0.0,
                1.0 if current_speed_mps > 2.0 else 0.45 if current_speed_mps > 0.1 else 0.0,
                0.0,
            )

        stopping_distance = stopping_distance_m(
            current_speed_mps=current_speed_mps,
            reaction_margin_m=0.0,
            preferred_deceleration_mps2=preferred_deceleration_mps2,
        )
        if stop_target_distance_m <= stopping_distance:
            late_ratio = 1.0 - min(1.0, stop_target_distance_m / max(stopping_distance, 1e-3))
            return 0.0, min(1.0, max(0.45, 0.45 + (0.55 * late_ratio))), 0.0

    return speed_control(
        current_speed_mps=current_speed_mps,
        target_speed_kmh=target_speed_kmh,
        previous_error_kmh=previous_error_kmh,
        max_throttle=0.75,
        max_brake=1.0,
    )


def is_traffic_light_violation(
    *,
    light_state: str | None,
    stop_line_distance_m: float | None,
    current_speed_mps: float,
) -> bool:
    return bool(
        light_state == "red"
        and stop_line_distance_m is not None
        and stop_line_distance_m <= 0.5
        and current_speed_mps > 2.0
    )
