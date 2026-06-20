# -*- coding: utf-8 -*-
"""Collect runtime feedback from the robot and write it to a file."""

import os
import signal
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from pybot_scout.feedback import FeedbackLogger
from pybot_scout.proximity import discover_proximity_topics
from pybot_scout.ros_inventory import log_ros_inventory
from pybot_scout.scout import pybot_scout

PROBE_DURATION_SECS = 60
SAMPLE_INTERVAL_SECS = 1.0

LOGGER = FeedbackLogger("runtime_probe")


def _signal_handler(signum, frame):
    LOGGER.log("signal_received", signum=signum)
    pybot_scout.stop()


def _subscribe_topics(topics):
    for topic in topics:
        try:
            pybot_scout.subscribe_proximity(topic)
            LOGGER.log("sensor_subscribed", topic=topic)
        except Exception as exc:
            LOGGER.log("sensor_subscribe_failed", topic=topic, error=str(exc))


def start():
    log_ros_inventory(LOGGER)
    topics = discover_proximity_topics(LOGGER)
    LOGGER.log(
        "probe_started",
        duration_secs=PROBE_DURATION_SECS,
        sample_interval_secs=SAMPLE_INTERVAL_SECS,
        proximity_topics=topics,
    )

    _subscribe_topics(topics)

    deadline = time.time() + PROBE_DURATION_SECS
    sample_index = 0
    while time.time() < deadline:
        sample_index += 1
        LOGGER.log(
            "proximity_sample",
            sample_index=sample_index,
            readings=pybot_scout.get_proximity_readings(),
        )
        time.sleep(SAMPLE_INTERVAL_SECS)

    LOGGER.log("probe_completed", total_samples=sample_index)


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

    LOGGER.close()
    pybot_scout.stop()
