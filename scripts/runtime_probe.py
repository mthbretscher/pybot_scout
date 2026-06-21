# -*- coding: utf-8 -*-
"""Collect runtime feedback from the robot and write it to a file."""

import os
import signal
import sys
import time
import math

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from pybot_scout.camera import GreyImageMonitor
from pybot_scout.feedback import FeedbackLogger
from pybot_scout.proximity import discover_proximity_topics
from pybot_scout.ros_inventory import log_ros_inventory
from pybot_scout.scout import pybot_scout

SAMPLE_INTERVAL_SECS = 1.0
MIN_VALID_TOF_M = 0.15
PROBE_DURATION_ENV = "PYBOT_SCOUT_PROBE_DURATION_SECS"

LOGGER = FeedbackLogger("runtime_probe", output_dir=os.path.join(REPO_ROOT, "run_feedback"))
CAMERA = GreyImageMonitor()
STOP_REQUESTED = False


def _signal_handler(signum, frame):
    global STOP_REQUESTED
    STOP_REQUESTED = True
    LOGGER.log("signal_received", signum=signum)


def _subscribe_topics(topics):
    for topic in topics:
        try:
            pybot_scout.subscribe_proximity(topic)
            LOGGER.log("sensor_subscribed", topic=topic)
        except Exception as exc:
            LOGGER.log("sensor_subscribe_failed", topic=topic, error=str(exc))


def _get_probe_duration_secs():
    raw = os.environ.get(PROBE_DURATION_ENV, "").strip()
    if not raw:
        return None
    try:
        duration = float(raw)
    except ValueError:
        LOGGER.log("probe_duration_invalid", env=PROBE_DURATION_ENV, value=raw)
        return None
    if duration <= 0.0:
        return None
    return duration


def _safe_float(value):
    try:
        return float(value)
    except Exception:
        return None


def _valid_tof_value(readings):
    if not readings:
        return None
    val = _safe_float(readings.get("/SensorNode/tof"))
    if val is None or math.isinf(val) or val < MIN_VALID_TOF_M:
        return None
    return val


def _pearson(pairs):
    n = len(pairs)
    if n < 2:
        return None
    sum_x = 0.0
    sum_y = 0.0
    sum_xx = 0.0
    sum_yy = 0.0
    sum_xy = 0.0
    for x, y in pairs:
        sum_x += x
        sum_y += y
        sum_xx += x * x
        sum_yy += y * y
        sum_xy += x * y
    num = (n * sum_xy) - (sum_x * sum_y)
    den_x = (n * sum_xx) - (sum_x * sum_x)
    den_y = (n * sum_yy) - (sum_y * sum_y)
    if den_x <= 0.0 or den_y <= 0.0:
        return None
    return num / math.sqrt(den_x * den_y)


def _build_correlation_summary(pairs_by_metric):
    result = {}
    for metric, pairs in pairs_by_metric.items():
        result[metric] = {
            "sample_count": len(pairs),
            "pearson_r": round(_pearson(pairs), 4) if _pearson(pairs) is not None else None,
        }
    return result


def start():
    duration_secs = _get_probe_duration_secs()
    CAMERA.start(logger=LOGGER)
    log_ros_inventory(LOGGER)
    topics = discover_proximity_topics(LOGGER)
    LOGGER.log(
        "probe_started",
        duration_secs=duration_secs,
        sample_interval_secs=SAMPLE_INTERVAL_SECS,
        proximity_topics=topics,
        camera_topic="/CoreNode/grey_img",
        stop_mode=("duration" if duration_secs is not None else "ctrl_c"),
    )

    _subscribe_topics(topics)

    deadline = (time.time() + duration_secs) if duration_secs is not None else None
    sample_index = 0
    pairs_by_metric = {
        "mean_brightness": [],
        "left_mean_brightness": [],
        "center_mean_brightness": [],
        "right_mean_brightness": [],
    }

    while not STOP_REQUESTED and (deadline is None or time.time() < deadline):
        sample_index += 1
        readings = pybot_scout.get_proximity_readings()
        camera_stats = CAMERA.get_snapshot()

        tof_m = _valid_tof_value(readings)
        if tof_m is not None and camera_stats is not None:
            for metric in pairs_by_metric:
                val = _safe_float(camera_stats.get(metric))
                if val is not None:
                    pairs_by_metric[metric].append((tof_m, val))

        LOGGER.log(
            "proximity_sample",
            sample_index=sample_index,
            readings=readings,
            camera_brightness=camera_stats,
        )
        time.sleep(SAMPLE_INTERVAL_SECS)

    LOGGER.log(
        "probe_completed",
        total_samples=sample_index,
        stopped_by_signal=STOP_REQUESTED,
        camera_has_data=CAMERA.has_data(),
    )
    LOGGER.log(
        "probe_correlation",
        tof_topic="/SensorNode/tof",
        min_valid_tof_m=MIN_VALID_TOF_M,
        metrics=_build_correlation_summary(pairs_by_metric),
    )


if __name__ == "__main__":
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGHUP, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    pybot_scout.start()
    try:
        start()
    except Exception as exc:
        LOGGER.log("probe_exception", error_type=exc.__class__.__name__, error=str(exc))
        pybot_scout.handle_exception(exc.__class__.__name__ + ': ' + str(exc))

    CAMERA.stop()
    LOGGER.close()
    pybot_scout.stop()
