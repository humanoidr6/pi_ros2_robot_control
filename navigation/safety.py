"""Hardware-independent navigation validation and coordinate helpers."""
import math


def missing_calibration(settings):
    missing = []
    for name in ('width_m', 'length_m', 'wheel_base_m', 'measured_speed_at_full_pwm_mps'):
        value = settings.get(name)
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
            missing.append(name)
    if settings.get('directions_verified') is not True:
        missing.append('directions_verified')
    cap = settings.get('autonomous_max_pwm')
    if not isinstance(cap, (int, float)) or not math.isfinite(cap) or not 0 < cap <= 40:
        missing.append('autonomous_max_pwm (must be 0–40)')
    return missing


def velocity_to_pwm(v, w, settings):
    if not math.isfinite(v) or not math.isfinite(w):
        raise ValueError('Nonfinite velocity')
    if missing_calibration(settings):
        raise ValueError('Robot calibration incomplete')
    half = w * settings['wheel_base_m'] / 2
    scale = 100 / settings['measured_speed_at_full_pwm_mps']
    left, right = (v-half)*scale, (v+half)*scale
    limit = settings['autonomous_max_pwm']
    divisor = max(1.0, abs(left)/limit, abs(right)/limit)
    left, right = left/divisor, right/divisor
    return (left * (-1 if settings['invert_left'] else 1),
            right * (-1 if settings['invert_right'] else 1))


def gate_reason(armed, command_age, scan_age, transform_age, nearest):
    if not armed:
        return 'Disarmed'
    if command_age > 0.35:
        return 'Command timeout'
    if scan_age > 0.4:
        return 'Lidar timeout'
    if transform_age > 0.5:
        return 'Localization transform stale'
    if nearest is None or not math.isfinite(nearest) or nearest < 0.5:
        return 'Obstacle within 0.5 m or insufficient returns'
    return ''


def pixel_to_world(col, row, width, height, resolution, origin):
    if not 0 <= col < width or not 0 <= row < height:
        raise ValueError('Outside map')
    x, y = (col+0.5)*resolution, (height-row-0.5)*resolution
    yaw = origin[2]
    return (origin[0]+math.cos(yaw)*x-math.sin(yaw)*y,
            origin[1]+math.sin(yaw)*x+math.cos(yaw)*y)


def world_to_pixel(x, y, height, resolution, origin):
    dx, dy = x-origin[0], y-origin[1]
    yaw = origin[2]
    return ((math.cos(yaw)*dx+math.sin(yaw)*dy)/resolution-0.5,
            height-0.5-(-math.sin(yaw)*dx+math.cos(yaw)*dy)/resolution)
