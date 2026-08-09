# -*- coding: utf-8 -*-
"""
Directional clearance scanning via in-place rotation.

scan_for_best_heading() performs a short forward-cone scan around the current
heading (left/right offsets only), not a full 360 sweep.  At each offset it
collects ToF readings for SAMPLE_WINDOW seconds and computes the *median* of
valid readings (>= MIN_VALID_M) to suppress transient sensor noise.

The Moorebot Scout / roller_eye has a single forward-facing ToF sensor with a
~27-42 degree field of view, so in-place rotation is the only way to sense in
nearby directions.

Compatible with the Python 2 ROS runtime on the roller_eye robot.
"""

import math
import time

# ── tunable scan parameters ───────────────────────────────────────────────────
SCAN_HALF_ANGLE_DEG = 40  # scan range is [-SCAN_HALF_ANGLE_DEG, +SCAN_HALF_ANGLE_DEG]
SCAN_STEP_DEG       = 10  # angular spacing between sampled offsets
SETTLE_SECS         = 0.15
SAMPLE_WINDOW       = 0.40
SAMPLE_INTERVAL     = 0.08
MIN_VALID_M         = 0.15  # readings below this are floor / charger surface noise

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


def _build_scan_offsets():
    """Return signed offsets in degrees, centered on 0 with near-forward priority."""
    offsets = [0]
    step = SCAN_STEP_DEG
    while step <= SCAN_HALF_ANGLE_DEG:
        offsets.append(-step)
        offsets.append(step)
        step += SCAN_STEP_DEG
    return offsets


def _rotate_by_delta(scout, delta_deg):
    """Rotate by signed delta degrees. Positive = CW, negative = CCW."""
    if delta_deg == 0:
        return
    magnitude = int(abs(delta_deg))
    if magnitude <= 0:
        return
    direction = 2 if delta_deg > 0 else 1
    scout.set_rotate_3(direction, magnitude)


def scan_for_best_heading(scout, logger=None):
    """Perform a short forward-cone scan and rotate to the clearest direction.

    Returns (best_delta_deg, scan_results) where:
      best_delta_deg -- signed heading delta from pre-scan heading:
                        negative = left/CCW, positive = right/CW.
      scan_results   -- list of (offset_deg, clearance_m) in scan order.

    After calling, update the working heading in the caller:
        heading = (heading + best_delta_deg) % 360
    """
    offsets = _build_scan_offsets()
    scan_results = []
    current_offset = 0

    for target_offset in offsets:
        delta = target_offset - current_offset
        _rotate_by_delta(scout, delta)
        time.sleep(SETTLE_SECS)
        clearance = _sample_clearance(scout)
        scan_results.append((target_offset, clearance))
        current_offset = target_offset

    # Prefer larger clearance; if equal, prefer smaller steering magnitude.
    best_offset, best_clearance = max(scan_results, key=lambda x: (x[1], -abs(x[0])))

    if logger is not None:
        display = []
        for off, c in scan_results:
            display.append((off, 'Inf' if c >= _INF_PROXY else round(c, 3)))
        logger.log(
            'direction_scan_forward',
            scan_results=display,
            scan_half_angle_deg=SCAN_HALF_ANGLE_DEG,
            scan_step_deg=SCAN_STEP_DEG,
            best_delta_deg=best_offset,
            best_clearance_m=('Inf' if best_clearance >= _INF_PROXY
                              else round(best_clearance, 3)),
        )

    # We are currently at current_offset; rotate directly to best_offset.
    _rotate_by_delta(scout, best_offset - current_offset)
    time.sleep(SETTLE_SECS)

    return best_offset, scan_results
