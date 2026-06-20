# -*- coding: utf-8 -*-
"""
Collect runtime feedback from the robot and write it to a file that can be
committed back for analysis.
"""

import signal
import time

from feedback_utils import FeedbackLogger
from rollereye import rollereye

PROXIMITY_TOPICS = [
    "/proximity_sensor",
    "/ultrasonic_front",
    "/ultrasonic_rear",
    "/ultrasonic_left",
    "/ultrasonic_right",
]

DISCOVERY_PATTERNS = [
    "proximity",
    "ultrasonic",
    "range",
    "dock",
    "charger",
    "apriltag",
    "tag",
    "aruco",
]

PROBE_DURATION_SECS = 60
SAMPLE_INTERVAL_SECS = 1.0

LOGGER = FeedbackLogger("runtime_probe")


def _signal_handler(signum, frame):
    LOGGER.log("signal_received", signum=signum)
    rollereye.stop()


def _discover_topics():
    try:
        import rospy
        topics = rospy.get_published_topics()
    except Exception as exc:
        LOGGER.log("topic_discovery_failed", error=str(exc))
        return

    LOGGER.log("topic_discovery_total", total=len(topics))
    for topic, topic_type in topics:
        lowered = topic.lower()
        if any(p in lowered for p in DISCOVERY_PATTERNS):
            LOGGER.log("topic_discovery_match", topic=topic, topic_type=topic_type)


def start():
    LOGGER.log(
        "probe_started",
        duration_secs=PROBE_DURATION_SECS,
        sample_interval_secs=SAMPLE_INTERVAL_SECS,
        proximity_topics=PROXIMITY_TOPICS,
    )

    for topic in PROXIMITY_TOPICS:
        try:
            rollereye.subscribe_proximity(topic)
            LOGGER.log("sensor_subscribed", topic=topic)
        except Exception as exc:
            LOGGER.log("sensor_subscribe_failed", topic=topic, error=str(exc))

    _discover_topics()

    deadline = time.time() + PROBE_DURATION_SECS
    sample_index = 0
    while time.time() < deadline:
        sample_index += 1
        LOGGER.log(
            "proximity_sample",
            sample_index=sample_index,
            readings=rollereye.get_proximity_readings(),
        )
        time.sleep(SAMPLE_INTERVAL_SECS)

    LOGGER.log("probe_completed", total_samples=sample_index)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGHUP, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    rollereye.start()
    try:
        start()
    except Exception as exc:
        LOGGER.log("probe_exception", error_type=exc.__class__.__name__, error=str(exc))
        rollereye.handle_exception(exc.__class__.__name__ + ': ' + str(exc))

    LOGGER.close()
    rollereye.stop()
