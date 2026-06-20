# -*- coding: utf-8 -*-

import os
import random
import signal
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from pybot_scout.feedback import FeedbackLogger
from pybot_scout.proximity import discover_proximity_topics
from pybot_scout.scout import pybot_scout

SPEED = 0.3
ROTATION_SPEED = 90
MIN_STEP_SECS = 2
MAX_STEP_SECS = 5
PAUSE_SECS = 0.5
OBSTACLE_THRESHOLD_M = 0.4
SENSOR_TIMEOUT_SECS = 5.0
CHECK_INTERVAL_SECS = 0.35
REDISCOVERY_INTERVAL_SECS = 15.0
REFRESH_SENSOR_WAIT_SECS = 2.0
ALLOW_SENSORLESS_FALLBACK = os.environ.get("PYBOT_SCOUT_ALLOW_SENSORLESS_FALLBACK", "0") == "1"

LOGGER = FeedbackLogger("obstacle_avoidance")


def _signal_handler(signum, frame):
    print("\nInterrupt received – stopping robot.")
    LOGGER.log("signal_received", signum=signum)
    pybot_scout.stop()


def _wait_for_sensor_data(timeout_secs):
    deadline = time.time() + timeout_secs
    while time.time() < deadline:
        readings = pybot_scout.get_proximity_readings()
        if any(value >= 0.0 for value in readings.values()):
            return True
        time.sleep(0.1)
    return False


def _subscribe_topics(subscribed_topics):
    discovered_topics = discover_proximity_topics(LOGGER)
    new_topics = []
    for topic in discovered_topics:
        if topic in subscribed_topics:
            continue
        try:
            pybot_scout.subscribe_proximity(topic)
            LOGGER.log("sensor_subscribed", topic=topic)
            subscribed_topics.add(topic)
            new_topics.append(topic)
        except Exception as exc:
            LOGGER.log("sensor_subscribe_failed", topic=topic, error=str(exc))
    if new_topics:
        LOGGER.log("sensor_topics_added", topics=new_topics)
    return subscribed_topics


def _rotate_away():
    direction = random.choice([1, 2])
    degree = random.randint(90, 180)
    print("Obstacle detected – rotating %d deg (dir=%d)" % (degree, direction))
    LOGGER.log("rotate_away", direction=direction, degree=degree)
    pybot_scout.set_rotate_3(direction, degree)
    time.sleep(0.3)


def _step_with_avoidance(direction, duration_secs, sensor_active):
    remaining_secs = duration_secs

    while remaining_secs > 0:
        if sensor_active and pybot_scout.is_obstacle_ahead(OBSTACLE_THRESHOLD_M):
            pybot_scout.stop_move()
            LOGGER.log(
                "obstacle_detected_mid_step",
                threshold_m=OBSTACLE_THRESHOLD_M,
                readings=pybot_scout.get_proximity_readings(),
            )
            return False

        burst_secs = min(CHECK_INTERVAL_SECS, remaining_secs)
        pybot_scout.set_translate_2(direction, burst_secs)
        remaining_secs = max(0.0, remaining_secs - burst_secs)
        LOGGER.log(
            "move_burst",
            direction_deg=direction,
            burst_secs=burst_secs,
            remaining_secs=remaining_secs,
            sensor_active=sensor_active,
            readings=pybot_scout.get_proximity_readings() if sensor_active else {},
        )

    return True


def start():
    pybot_scout.set_rotationSpeed(ROTATION_SPEED)
    pybot_scout.set_translationSpeed(SPEED)

    subscribed_topics = _subscribe_topics(set())
    LOGGER.log(
        "run_started",
        speed=SPEED,
        rotation_speed=ROTATION_SPEED,
        min_step_secs=MIN_STEP_SECS,
        max_step_secs=MAX_STEP_SECS,
        pause_secs=PAUSE_SECS,
        obstacle_threshold_m=OBSTACLE_THRESHOLD_M,
        sensor_timeout_secs=SENSOR_TIMEOUT_SECS,
        check_interval_secs=CHECK_INTERVAL_SECS,
        rediscovery_interval_secs=REDISCOVERY_INTERVAL_SECS,
        allow_sensorless_fallback=ALLOW_SENSORLESS_FALLBACK,
        proximity_topics=sorted(subscribed_topics),
    )

    print("Waiting up to %.0f s for sensor data..." % SENSOR_TIMEOUT_SECS)
    sensor_active = _wait_for_sensor_data(SENSOR_TIMEOUT_SECS)
    LOGGER.log(
        "sensor_wait_completed",
        sensor_active=sensor_active,
        readings=pybot_scout.get_proximity_readings(),
    )

    if sensor_active:
        print("Proximity sensor data received – obstacle avoidance ENABLED.")
    else:
        print("WARNING: No proximity sensor data received after %.0f s." % SENSOR_TIMEOUT_SECS)
        if ALLOW_SENSORLESS_FALLBACK:
            print("         Running in sensor-less random-walk fallback mode.")
            LOGGER.log("sensor_fallback_enabled")
        else:
            print("         Sensor-less fallback DISABLED; waiting for valid sensor data.")
            LOGGER.log("sensor_fallback_disabled")

    print("Obstacle avoidance walk started. Press Ctrl-C to stop.")

    next_discovery_at = time.time() + REDISCOVERY_INTERVAL_SECS

    while True:
        if not sensor_active and time.time() >= next_discovery_at:
            subscribed_topics = _subscribe_topics(subscribed_topics)
            sensor_active = _wait_for_sensor_data(REFRESH_SENSOR_WAIT_SECS)
            LOGGER.log(
                "sensor_refresh_completed",
                sensor_active=sensor_active,
                readings=pybot_scout.get_proximity_readings(),
                proximity_topics=sorted(subscribed_topics),
            )
            next_discovery_at = time.time() + REDISCOVERY_INTERVAL_SECS

        if not sensor_active and not ALLOW_SENSORLESS_FALLBACK:
            LOGGER.log("waiting_for_sensor_data", proximity_topics=sorted(subscribed_topics))
            time.sleep(PAUSE_SECS)
            continue

        direction = random.randint(0, 360)
        duration = random.uniform(MIN_STEP_SECS, MAX_STEP_SECS)
        LOGGER.log(
            "step_selected",
            direction_deg=direction,
            requested_duration_s=duration,
            sensor_active=sensor_active,
        )

        if sensor_active and pybot_scout.is_obstacle_ahead(OBSTACLE_THRESHOLD_M):
            LOGGER.log(
                "obstacle_detected_pre_step",
                threshold_m=OBSTACLE_THRESHOLD_M,
                readings=pybot_scout.get_proximity_readings(),
            )
            _rotate_away()
            direction = random.randint(0, 360)
            LOGGER.log("step_direction_reselected", direction_deg=direction)

        print("Moving direction=%d deg for %.1f s (sensors=%s)" % (direction, duration, sensor_active))
        completed = _step_with_avoidance(direction, duration, sensor_active)

        if not completed:
            _rotate_away()
            LOGGER.log("step_interrupted_for_obstacle")

        time.sleep(PAUSE_SECS)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGHUP, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    pybot_scout.start()

    try:
        start()
    except Exception as exc:
        LOGGER.log("run_exception", error=str(exc), error_type=exc.__class__.__name__)
        pybot_scout.handle_exception(exc.__class__.__name__ + ': ' + str(exc))

    LOGGER.log("run_stopped")
    LOGGER.close()
    pybot_scout.stop()
