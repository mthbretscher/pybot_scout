# -*- coding: utf-8 -*-
"""Helpers for finding usable proximity topics on the robot."""

import os

DEFAULT_PROXIMITY_TOPICS = [
    "/SensorNode/tof",
    "/SensorNode/ibeacon",
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
    "sonar",
    "tof",
]

RANGE_MESSAGE_TYPES = set([
    "sensor_msgs/Range",
])


def _unique(items):
    seen = set()
    result = []
    for item in items:
        if item and item not in seen:
            seen.add(item)
            result.append(item)
    return result


def _log(logger, event, **fields):
    if logger is not None:
        logger.log(event, **fields)


def get_configured_proximity_topics():
    raw = os.environ.get("PYBOT_SCOUT_PROXIMITY_TOPICS", "")
    configured = [item.strip() for item in raw.split(",") if item.strip()]
    return _unique(configured + DEFAULT_PROXIMITY_TOPICS)


def discover_proximity_topics(logger=None):
    configured_topics = get_configured_proximity_topics()
    discovered_topics = []
    range_topics = []
    matching_topics = []

    try:
        import rospy
        published_topics = rospy.get_published_topics()
    except Exception as exc:
        _log(logger, "topic_discovery_failed", error=str(exc))
        return configured_topics

    published_set = {topic for topic, _type in published_topics}
    _log(logger, "topic_discovery_total", total=len(published_topics))

    for topic, topic_type in published_topics:
        lowered = topic.lower()
        if topic_type in RANGE_MESSAGE_TYPES:
            range_topics.append(topic)
            discovered_topics.append(topic)
        elif any(pattern in lowered for pattern in DISCOVERY_PATTERNS):
            matching_topics.append({"topic": topic, "topic_type": topic_type})
            discovered_topics.append(topic)

    if range_topics:
        _log(logger, "topic_discovery_range_topics", topics=range_topics)
    for match in matching_topics:
        _log(logger, "topic_discovery_match", topic=match["topic"], topic_type=match["topic_type"])

    # Only include configured/default topics that are actually published on the master.
    validated_configured = [t for t in configured_topics if t in published_set]

    selected_topics = _unique(validated_configured + discovered_topics)
    _log(logger, "topic_discovery_selected", topics=selected_topics)
    return selected_topics
