# -*- coding: utf-8 -*-
"""
Lightweight grayscale camera monitoring for the roller_eye robot.

Subscribes to /CoreNode/grey_img (sensor_msgs/Image) and keeps a recent
brightness summary that can be sampled by scripts such as runtime_probe.py.

The monitor is intentionally simple and Python-2-compatible:
  - no numpy / cv_bridge dependency
  - samples a sparse grid of pixels for low CPU overhead
  - stores mean brightness for the whole frame, left/center/right thirds,
    and the lower rim where forward-floor cues are more likely to appear

BrightnessWindow is a rolling buffer used by navigation scripts to detect
brightness deltas (approaching obstacles) and stuck conditions (scene not
changing while drive commands are being issued).
"""

import threading
import time

GREY_IMAGE_TOPIC = "/CoreNode/grey_img"
PROCESS_INTERVAL_SECS = 0.25
LOWER_RIM_START_NUM = 3
LOWER_RIM_START_DEN = 4


def _safe_round(value):
    if value is None:
        return None
    return round(float(value), 3)


def _byte_value(raw):
    """Return integer 0..255 from a Python 2/3 bytes element."""
    if isinstance(raw, int):
        return raw
    return ord(raw)


def _sample_image_stats(msg):
    """Return a sparse-sampled brightness summary for a sensor_msgs/Image."""
    width = int(getattr(msg, "width", 0) or 0)
    height = int(getattr(msg, "height", 0) or 0)
    step = int(getattr(msg, "step", 0) or 0)
    encoding = str(getattr(msg, "encoding", "") or "")
    data = getattr(msg, "data", None)

    if width <= 0 or height <= 0 or step <= 0 or not data:
        return None

    bytes_per_pixel = step // width if width > 0 else 1
    if bytes_per_pixel <= 0:
        bytes_per_pixel = 1

    row_stride = max(1, height // 24)
    col_stride = max(1, width // 32)

    total_sum = 0.0
    total_count = 0
    left_sum = 0.0
    left_count = 0
    center_sum = 0.0
    center_count = 0
    right_sum = 0.0
    right_count = 0
    lower_sum = 0.0
    lower_count = 0
    lower_left_sum = 0.0
    lower_left_count = 0
    lower_center_sum = 0.0
    lower_center_count = 0
    lower_right_sum = 0.0
    lower_right_count = 0

    one_third = width // 3
    two_third = (2 * width) // 3
    lower_row_start = (height * LOWER_RIM_START_NUM) // LOWER_RIM_START_DEN

    row = 0
    while row < height:
        row_offset = row * step
        col = 0
        while col < width:
            idx = row_offset + (col * bytes_per_pixel)
            if idx >= len(data):
                break
            val = float(_byte_value(data[idx]))
            total_sum += val
            total_count += 1

            if col < one_third:
                left_sum += val
                left_count += 1
            elif col < two_third:
                center_sum += val
                center_count += 1
            else:
                right_sum += val
                right_count += 1

            if row >= lower_row_start:
                lower_sum += val
                lower_count += 1
                if col < one_third:
                    lower_left_sum += val
                    lower_left_count += 1
                elif col < two_third:
                    lower_center_sum += val
                    lower_center_count += 1
                else:
                    lower_right_sum += val
                    lower_right_count += 1
            col += col_stride
        row += row_stride

    if total_count == 0:
        return None

    forward_mean = None
    forward_source = None
    forward_count = 0
    if lower_center_count:
        forward_mean = lower_center_sum / lower_center_count
        forward_source = "lower_center"
        forward_count = lower_center_count
    elif lower_count:
        forward_mean = lower_sum / lower_count
        forward_source = "lower_rim"
        forward_count = lower_count
    elif center_count:
        forward_mean = center_sum / center_count
        forward_source = "center"
        forward_count = center_count

    return {
        "encoding": encoding,
        "width": width,
        "height": height,
        "sample_points": total_count,
        "mean_brightness": _safe_round(total_sum / total_count),
        "left_mean_brightness": _safe_round(left_sum / left_count) if left_count else None,
        "center_mean_brightness": _safe_round(center_sum / center_count) if center_count else None,
        "right_mean_brightness": _safe_round(right_sum / right_count) if right_count else None,
        "lower_rim_sample_points": lower_count,
        "lower_rim_mean_brightness": _safe_round(lower_sum / lower_count) if lower_count else None,
        "lower_left_mean_brightness": _safe_round(lower_left_sum / lower_left_count) if lower_left_count else None,
        "lower_center_mean_brightness": _safe_round(lower_center_sum / lower_center_count) if lower_center_count else None,
        "lower_right_mean_brightness": _safe_round(lower_right_sum / lower_right_count) if lower_right_count else None,
        "forward_brightness_source": forward_source,
        "forward_sample_points": forward_count,
        "forward_mean_brightness": _safe_round(forward_mean),
    }


class GreyImageMonitor(object):
    """Thread-safe brightness monitor for /CoreNode/grey_img."""

    def __init__(self):
        self._lock = threading.Lock()
        self._sub = None
        self._logger = None
        self._latest = None
        self._msg_ts = None
        self._first_logged = False
        self._last_processed_ts = 0.0

    def start(self, logger=None):
        self._logger = logger
        try:
            import rospy
            from sensor_msgs.msg import Image

            def _cb(msg):
                now = time.time()
                with self._lock:
                    if now - self._last_processed_ts < PROCESS_INTERVAL_SECS:
                        return
                    self._last_processed_ts = now

                stats = _sample_image_stats(msg)
                if stats is None:
                    return

                with self._lock:
                    self._latest = stats
                    self._msg_ts = now
                    first = not self._first_logged
                    self._first_logged = True

                if first and self._logger is not None:
                    self._logger.log(
                        "camera_msg_schema",
                        topic=GREY_IMAGE_TOPIC,
                        encoding=stats.get("encoding"),
                        width=stats.get("width"),
                        height=stats.get("height"),
                        step=getattr(msg, "step", None),
                        is_bigendian=getattr(msg, "is_bigendian", None),
                        sample_points=stats.get("sample_points"),
                    )

            self._sub = rospy.Subscriber(GREY_IMAGE_TOPIC, Image, _cb)
        except Exception:
            pass

    def stop(self):
        if self._sub is not None:
            try:
                self._sub.unregister()
            except Exception:
                pass
            self._sub = None

    def has_data(self):
        with self._lock:
            return self._msg_ts is not None

    def get_snapshot(self):
        with self._lock:
            if self._latest is None:
                return None
            return dict(self._latest)


class BrightnessWindow(object):
    """Rolling window of (fwd, left, right) brightness tuples for navigation.

    Used by exploration scripts to:
      - Compute brightness deltas (rate of change) to detect approaching walls
      - Detect stuck conditions when brightness barely varies over many readings

    Python-2-compatible; no external dependencies.
    """

    def __init__(self, size=20):
        self._buf = []          # list of (fwd, left, right) floats
        self._size = size

    def push(self, fwd, left, right):
        """Append a new brightness reading."""
        self._buf.append((float(fwd), float(left), float(right)))
        if len(self._buf) > self._size:
            del self._buf[0]

    def get_delta(self, span=5):
        """Return (delta_fwd, delta_left, delta_right) change over the last
        ``span`` readings (positive = getting brighter = approaching object).
        Returns (0, 0, 0) if fewer than 2 readings are available.
        """
        if len(self._buf) < 2:
            return 0.0, 0.0, 0.0
        n = min(span, len(self._buf))
        old = self._buf[-n]
        new = self._buf[-1]
        return new[0] - old[0], new[1] - old[1], new[2] - old[2]

    def is_stuck(self, min_readings=None, spread_threshold=5.0):
        """Return True when the forward brightness spread over the last
        ``min_readings`` samples is below ``spread_threshold``.

        A small spread while drive commands are being issued suggests the
        scene is not changing and the robot may be stuck.
        """
        n = min_readings if min_readings is not None else self._size
        if len(self._buf) < n:
            return False
        fwds = [r[0] for r in self._buf[-n:]]
        return (max(fwds) - min(fwds)) < spread_threshold

    def full(self):
        """Return True once the window has accumulated ``size`` readings."""
        return len(self._buf) >= self._size

    def reset(self):
        """Clear all stored readings."""
        del self._buf[:]
