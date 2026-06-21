# -*- coding: utf-8 -*-
"""
Directional clearance scanning via in-place rotation.

scan_for_best_heading() performs a full 360-degree CW sweep in equal angular
steps.  At each position it collects ToF readings for SAMPLE_WINDOW seconds
and computes the *median* of valid readings (>= MIN_VALID_M) to suppress
transient sensor noise.  After the sweep the robot is back at its original
heading; it then rotates to face the direction of greatest clearance.

Typical timing at ROTATION_SPEED=90 deg/s:
    8 steps x (0.5 s rotation + SETTLE_SECS + SAMPLE_WINDOW) ~= 8.4 s

The Moorebot Scout / roller_eye has a single forward-facing ToF sensor with a
~27-42 degree field of view, so in-place rotation is the only way to sense in
multiple directions.

Compatible with the Python 2 ROS runtime on the roller_eye robot.
"""

import math
import time

# ── tunable scan parameters ───────────────────────────────────────────────────
STEP_DEG        = 45    # degrees per rotation step
N_STEPS         = 8     # steps for a full circle  (N_STEPS * STEP_DEG must == 360)
SETTLE_SECS     = 0.15  # pause after each rotation before sampling starts
SAMPLE_WINDOW   = 0.40  # seconds to collect samples at each angular position
SAMPLE_INTERVAL = 0.08  # seconds between individual sensor reads within window
MIN_VALID_M     = 0.15  # readings below this are floor / charger surface noise

# Readings at or above this value are treated as "fully clear" (including Inf)
_INF_PROXY = 9.99


def _median(values):
    """Return the median of a non-empty sorted list (no numpy dependency)."""
    s = sorted(values)
    n = len(s)
    mid = n // 2
    if n % 2 == 1:
        return s[mid]
    return (s[mid - 1] + s[mid]) / 2.0


def _sample_clearance(scout):
    """Collect ToF readings for SAMPLE_WINDOW seconds; return median clearance.

    Classification:
      Infinity / > _INF_PROXY  ->  _INF_PROXY  (path is clear)
      < MIN_VALID_M             ->  discarded   (floor / charger noise)
      everything else           ->  used as-is

    Returns _INF_PROXY when all readings are clear.
    Returns 0.0 when no valid reading was collected (total noise).
    """
    valid = []
    deadline = time.time() + SAMPLE_WINDOW
    while time.time() < deadline:
        readings = scout.get_proximity_readings()
        for v in readings.values():
            if math.isinf(v) or v > _INF_PROXY:
                valid.append(_INF_PROXY)
            elif v >= MIN_VALID_M:
                valid.append(v)
        time.sleep(SAMPLE_INTERVAL)
    return _median(valid) if valid else 0.0


def scan_for_best_heading(scout, logger=None):
    """Rotate CW through N_STEPS * STEP_DEG = 360 degrees, sampling clearance
    at each position with noise-reducing time-averaging.  Completes the circle
    back to the original heading, then rotates to face the clearest direction.

    Returns (best_cw_offset_deg, scan_results) where:
      best_cw_offset_deg -- CW degrees from the pre-scan heading that was
                            selected as the escape direction (0 = stay forward).
      scan_results       -- list of (angle_cw_deg, clearance_m) in sweep order.

    After calling, update the working heading in the caller:
        heading = (heading + best_cw_offset_deg) % 360
    """
    scan_results = []

    for step in range(N_STEPS):
        angle = step * STEP_DEG
        if step > 0:
            scout.set_rotate_3(2, STEP_DEG)   # direction 2 = CW
        time.sleep(SETTLE_SECS)
        clearance = _sample_clearance(scout)
        scan_results.append((angle, clearance))

    # One final step completes the 360 degrees back to the original heading
    scout.set_rotate_3(2, STEP_DEG)
    time.sleep(SETTLE_SECS)

    best_angle, best_clearance = max(scan_results, key=lambda x: x[1])

    if logger is not None:
        display = []
        for a, c in scan_results:
            display.append((a, 'Inf' if c >= _INF_PROXY else round(c, 3)))
        logger.log(
            'direction_scan',
            scan_results=display,
            best_cw_offset_deg=best_angle,
            best_clearance_m=('Inf' if best_clearance >= _INF_PROXY
                              else round(best_clearance, 3)),
        )

    # Rotate to face the best direction using the shorter arc
    if best_angle > 0:
        if best_angle <= 180:
            scout.set_rotate_3(2, best_angle)           # CW
        else:
            scout.set_rotate_3(1, 360 - best_angle)     # CCW (shorter)

    return best_angle, scan_results
