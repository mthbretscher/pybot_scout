# -*- coding: utf-8 -*-
# obstacle_avoidance.py
#
# Makes the Moorebot Scout drive randomly through an apartment while using
# proximity / range sensors to detect and avoid obstacles.
#
# Behaviour:
#   1. Subscribe to proximity sensor ROS topics.
#   2. Pick a random direction and drive.
#   3. Before and during each movement step, check whether any sensor reports
#      a reading below OBSTACLE_THRESHOLD_M.
#   4. If an obstacle is detected: stop, rotate a random amount (90-180 deg),
#      re-check, then resume.
#   5. If the proximity topic is unavailable (no messages received within
#      SENSOR_TIMEOUT_SECS after subscribing) fall back to pure random walk
#      and log a warning.
#
# Usage:
#   python obstacle_avoidance.py
#
# Press Ctrl-C (or send SIGTERM) to stop the robot cleanly.

import signal
import random
import time

from rollereye import rollereye

# ---------------------------------------------------------------------------
# Tuneable constants
# ---------------------------------------------------------------------------
SPEED = 0.3                     # translation speed in m/s (0 – 1)
ROTATION_SPEED = 90             # rotation speed in degree/s
MIN_STEP_SECS = 2               # minimum time to drive in one direction (s)
MAX_STEP_SECS = 5               # maximum time to drive in one direction (s)
PAUSE_SECS = 0.5                # pause between steps (s)
OBSTACLE_THRESHOLD_M = 0.4     # stop if any sensor reads closer than this (m)
SENSOR_TIMEOUT_SECS = 5.0      # seconds to wait for first sensor data before
                                # switching to sensor-less fallback mode

# Proximity sensor ROS topics to subscribe to.
# Add / remove topics to match the actual sensors on your Scout.
# Common names used on Moorebot / RoboMaster-style robots:
PROXIMITY_TOPICS = [
    "/proximity_sensor",       # generic single sensor
    "/ultrasonic_front",
    "/ultrasonic_rear",
    "/ultrasonic_left",
    "/ultrasonic_right",
]
# ---------------------------------------------------------------------------


def _signal_handler(signum, frame):
    print("\nInterrupt received – stopping robot.")
    rollereye.stop()


def _wait_for_sensor_data(timeout_secs):
    """Return True if at least one proximity topic has returned a reading."""
    deadline = time.time() + timeout_secs
    while time.time() < deadline:
        readings = rollereye.get_proximity_readings()
        if any(v >= 0.0 for v in readings.values()):
            return True
        time.sleep(0.1)
    return False


def _rotate_away():
    """Rotate a random amount (90–180 degrees) in a random direction."""
    direction = random.choice([1, 2])   # 1=left, 2=right
    degree = random.randint(90, 180)
    print("Obstacle detected – rotating %d deg (dir=%d)" % (degree, direction))
    rollereye.set_rotate_3(direction, degree)
    time.sleep(0.3)  # brief settle after rotation


def _step_with_avoidance(direction, duration_secs, sensor_active):
    """
    Drive in *direction* for up to *duration_secs* seconds, checking for
    obstacles every CHECK_INTERVAL seconds if *sensor_active* is True.

    Returns True if the step completed without an obstacle, False if the
    robot had to stop early due to an obstacle.
    """
    CHECK_INTERVAL = 0.25  # seconds between sensor polls mid-move
    steps = int(duration_secs / CHECK_INTERVAL)

    for _ in range(max(steps, 1)):
        if sensor_active and rollereye.is_obstacle_ahead(OBSTACLE_THRESHOLD_M):
            rollereye.stop_move()
            return False
        # Drive one short burst in the chosen direction
        rollereye.set_translate_2(direction, 1)

    return True


def start():
    rollereye.set_rotationSpeed(ROTATION_SPEED)
    rollereye.set_translationSpeed(SPEED)

    # Subscribe to all configured proximity topics.
    print("Subscribing to proximity topics...")
    for topic in PROXIMITY_TOPICS:
        try:
            rollereye.subscribe_proximity(topic)
            print("  subscribed: %s" % topic)
        except Exception as exc:
            print("  could not subscribe to %s: %s" % (topic, exc))

    # Wait briefly to see if any sensor actually sends data.
    print("Waiting up to %.0f s for sensor data..." % SENSOR_TIMEOUT_SECS)
    sensor_active = _wait_for_sensor_data(SENSOR_TIMEOUT_SECS)

    if sensor_active:
        print("Proximity sensor data received – obstacle avoidance ENABLED.")
    else:
        print("WARNING: No proximity sensor data received after %.0f s." % SENSOR_TIMEOUT_SECS)
        print("         Running in sensor-less random-walk fallback mode.")

    print("Obstacle avoidance walk started. Press Ctrl-C to stop.")

    while True:
        direction = random.randint(0, 360)
        duration = random.uniform(MIN_STEP_SECS, MAX_STEP_SECS)

        # Pre-move obstacle check
        if sensor_active and rollereye.is_obstacle_ahead(OBSTACLE_THRESHOLD_M):
            _rotate_away()
            # Re-check before driving; pick a new direction too
            direction = random.randint(0, 360)

        print("Moving direction=%d deg for %.1f s (sensors=%s)" % (
            direction, duration, sensor_active))

        completed = _step_with_avoidance(direction, duration, sensor_active)

        if not completed:
            _rotate_away()

        time.sleep(PAUSE_SECS)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGHUP, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    rollereye.start()

    try:
        start()
    except Exception as e:
        rollereye.handle_exception(e.__class__.__name__ + ': ' + str(e))

    rollereye.stop()
