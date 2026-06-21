# -*- coding: utf-8 -*-
"""
Pong-ball explorer.

The robot travels in a straight line until a proximity sensor reports an
obstacle within OBSTACLE_THRESHOLD_M.  It then "bounces" – rotating by a
random angle between BOUNCE_MIN_DEG and BOUNCE_MAX_DEG in a random direction –
and continues in the new heading.  The effect is similar to a Pong ball
bouncing off walls, but the robot always turns *before* hitting anything.

At startup the script detects whether the robot is sitting on its charging
station (via /SensorNode/simple_battery_status).  If so it drives straight
forward for CHARGER_EXIT_SECS to clear the dock before beginning exploration.

This script replaces the former random_walk.py and obstacle_avoidance.py pair.

Usage:
    python scripts/obstacle_avoidance.py

Environment:
    PYBOT_SCOUT_ALLOW_SENSORLESS_FALLBACK  – set to "1" to allow movement
                                              without proximity sensor data
    PYBOT_SCOUT_PROXIMITY_TOPICS           – override discovered topics
"""

import os
import random
import signal
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT   = os.path.dirname(SCRIPT_DIR)
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from pybot_scout.charging_pile import ChargingPileDetector, ChargingStatusDetector
from pybot_scout.feedback import FeedbackLogger
from pybot_scout.odometry import OdometryTracker
from pybot_scout.proximity import discover_proximity_topics
from pybot_scout.scout import pybot_scout

# ── tunable constants ──────────────────────────────────────────────────────────
SPEED                = 0.3     # m/s forward speed
ROTATION_SPEED       = 90      # deg/s rotation speed
OBSTACLE_THRESHOLD_M = 0.25    # only bounce when within 25 cm; 0.25–0.40 m is ok to drive through
MIN_VALID_M          = 0.15    # ignore readings below this (floor / charger noise)
CHECK_INTERVAL_SECS  = 0.30    # drive-burst length; sensor is checked between bursts
PAUSE_SECS           = 0.30    # brief stop between a bounce and the next move
BOUNCE_MIN_DEG       = 110     # minimum rotation on a bounce
BOUNCE_MAX_DEG       = 170     # maximum rotation on a bounce
SENSOR_TIMEOUT_SECS  = 5.0     # wait this long for the first valid sensor reading
CHARGER_EXIT_SECS    = 3.0     # drive straight for this long to clear the dock
REDISCOVERY_SECS     = 15.0    # re-scan for proximity topics this often

# Stuck detection: if tof readings stay within STUCK_EPSILON m for STUCK_BURST_COUNT
# consecutive drive bursts the robot is probably stuck (wheels spinning against a wall).
STUCK_BURST_COUNT    = 5
STUCK_EPSILON        = 0.010   # m

ALLOW_SENSORLESS = os.environ.get("PYBOT_SCOUT_ALLOW_SENSORLESS_FALLBACK", "0") == "1"

FEEDBACK_DIR   = os.path.join(REPO_ROOT, "run_feedback")
LOGGER         = FeedbackLogger("obstacle_avoidance", output_dir=FEEDBACK_DIR)
ODOM_TRACKER   = OdometryTracker()
PILE_DETECTOR  = ChargingPileDetector()
CHARGER_STATUS = ChargingStatusDetector()


# ── signal handling ────────────────────────────────────────────────────────────

def _signal_handler(signum, frame):
    print("\nInterrupt received – stopping robot.")
    LOGGER.log("signal_received", signum=signum)
    pybot_scout.stop()


# ── sensor helpers ─────────────────────────────────────────────────────────────

def _subscribe_topics(subscribed):
    """Discover and subscribe to any new proximity topics; return updated set."""
    for topic in discover_proximity_topics(LOGGER):
        if topic in subscribed:
            continue
        try:
            pybot_scout.subscribe_proximity(topic)
            LOGGER.log("sensor_subscribed", topic=topic)
            subscribed.add(topic)
        except Exception as exc:
            LOGGER.log("sensor_subscribe_failed", topic=topic, error=str(exc))
    return subscribed


def _wait_for_sensor_data(timeout_secs):
    """Return True once any proximity reading is valid (>= 0)."""
    deadline = time.time() + timeout_secs
    while time.time() < deadline:
        if any(v >= 0.0 for v in pybot_scout.get_proximity_readings().values()):
            return True
        time.sleep(0.1)
    return False


def _nearest_obstacle(readings):
    """Return the closest valid obstacle distance, or None if path is clear.

    Readings below MIN_VALID_M (floor / dock surface noise) are ignored.
    Only distances within [MIN_VALID_M, OBSTACLE_THRESHOLD_M) are treated as
    obstacles; space between OBSTACLE_THRESHOLD_M and infinity is clear to drive.
    """
    nearest = None
    for dist in readings.values():
        if MIN_VALID_M <= dist < OBSTACLE_THRESHOLD_M:
            if nearest is None or dist < nearest:
                nearest = dist
    return nearest


def _raw_tof(readings):
    """Return the closest non-negative reading across all topics, or None."""
    vals = [v for v in readings.values() if v >= 0.0]
    return min(vals) if vals else None


def _check_stuck(reading_buf):
    """Return True when the last STUCK_BURST_COUNT entries span <= STUCK_EPSILON.

    Wheels spinning against a wall produce a constant tof reading even though
    motors are commanded to move.
    """
    if len(reading_buf) < STUCK_BURST_COUNT:
        return False
    tail = reading_buf[-STUCK_BURST_COUNT:]
    return (max(tail) - min(tail)) <= STUCK_EPSILON


# ── charger exit ───────────────────────────────────────────────────────────────

def _exit_charger_if_needed():
    """Drive straight off the charging station if we appear to be docked.

    Returns True if a charger-exit move was performed.
    """
    # Primary check: battery status topic
    charging = CHARGER_STATUS.wait_for_status(timeout_secs=2.5)

    # Fallback: if battery status unavailable, check tof heuristic
    if charging is None:
        readings = pybot_scout.get_proximity_readings()
        valid = [d for d in readings.values() if 0.0 <= d < 0.15]
        total = [d for d in readings.values() if d >= 0.0]
        charging = bool(total) and len(valid) >= len(total) * 0.8

    LOGGER.log("charger_status_check", charging=charging)

    if not charging:
        return False

    print("On charging station – driving straight forward to clear dock.")
    LOGGER.log("charger_exit_started", exit_secs=CHARGER_EXIT_SECS)
    pybot_scout.set_translate_smooth(0, CHARGER_EXIT_SECS)
    time.sleep(CHARGER_EXIT_SECS + 0.2)
    LOGGER.log("charger_exit_completed")
    return True


# ── main loop ──────────────────────────────────────────────────────────────────

def start():
    ODOM_TRACKER.start()
    PILE_DETECTOR.start()
    CHARGER_STATUS.start()

    pybot_scout.set_rotationSpeed(ROTATION_SPEED)
    pybot_scout.set_translationSpeed(SPEED)

    subscribed = _subscribe_topics(set())
    LOGGER.log(
        "run_started",
        speed=SPEED,
        rotation_speed=ROTATION_SPEED,
        obstacle_threshold_m=OBSTACLE_THRESHOLD_M,
        min_valid_m=MIN_VALID_M,
        check_interval_secs=CHECK_INTERVAL_SECS,
        bounce_min_deg=BOUNCE_MIN_DEG,
        bounce_max_deg=BOUNCE_MAX_DEG,
        sensor_timeout_secs=SENSOR_TIMEOUT_SECS,
        charger_exit_secs=CHARGER_EXIT_SECS,
        allow_sensorless=ALLOW_SENSORLESS,
        proximity_topics=sorted(subscribed),
    )

    print("Waiting up to %.0f s for proximity sensor data…" % SENSOR_TIMEOUT_SECS)
    sensor_active = _wait_for_sensor_data(SENSOR_TIMEOUT_SECS)
    LOGGER.log(
        "sensor_wait_completed",
        sensor_active=sensor_active,
        readings=pybot_scout.get_proximity_readings(),
    )

    if sensor_active:
        print("Proximity sensor data received – obstacle avoidance ENABLED.")
    else:
        print("WARNING: No proximity sensor data after %.0f s." % SENSOR_TIMEOUT_SECS)
        if not ALLOW_SENSORLESS:
            print("         Set PYBOT_SCOUT_ALLOW_SENSORLESS_FALLBACK=1 to run without sensors.")
            LOGGER.log("sensor_fallback_disabled")

    # ── leave the dock first ───────────────────────────────────────────────────
    _exit_charger_if_needed()

    # ── pick a random starting heading ────────────────────────────────────────
    heading = random.randint(0, 359)
    print("Pong explorer started.  Initial heading: %d°.  Press Ctrl-C to stop." % heading)

    next_rediscovery = time.time() + REDISCOVERY_SECS
    tof_buf = []   # rolling raw tof readings for stuck detection

    while True:
        # ── periodic topic rediscovery (helps if sensors come online late) ────
        if time.time() >= next_rediscovery:
            subscribed = _subscribe_topics(subscribed)
            if not sensor_active:
                sensor_active = _wait_for_sensor_data(1.0)
            next_rediscovery = time.time() + REDISCOVERY_SECS

        if not sensor_active and not ALLOW_SENSORLESS:
            LOGGER.log("waiting_for_sensor_data")
            time.sleep(0.5)
            continue

        readings = pybot_scout.get_proximity_readings() if sensor_active else {}
        obstacle_dist = _nearest_obstacle(readings) if sensor_active else None

        if obstacle_dist is not None:
            # ── bounce ────────────────────────────────────────────────────────
            pybot_scout.stop_move()
            del tof_buf[:]   # reset stuck buffer after any direction change
            deviation = random.randint(BOUNCE_MIN_DEG, BOUNCE_MAX_DEG)
            rotate_dir = random.choice([1, 2])   # 1 = CCW/left, 2 = CW/right
            new_heading = (heading + (deviation if rotate_dir == 1 else -deviation)) % 360

            print("Bounce!  %d° → %d°  (obstacle %.2f m,  rotating %d° %s)" % (
                heading, new_heading, obstacle_dist, deviation,
                "left" if rotate_dir == 1 else "right"))
            LOGGER.log(
                "pong_bounce",
                old_heading_deg=heading,
                new_heading_deg=new_heading,
                deviation_deg=deviation,
                rotate_dir=rotate_dir,
                obstacle_m=round(obstacle_dist, 3),
                readings=readings,
            )

            pybot_scout.set_rotate_3(rotate_dir, deviation)
            heading = new_heading

            # Log the new heading as a breadcrumb pose for return_home
            pose = ODOM_TRACKER.get_pose()
            LOGGER.log("step_selected",
                       direction_deg=heading,
                       pose=pose,
                       sensor_active=sensor_active)

            if PILE_DETECTOR.was_recently_seen():
                LOGGER.log("charging_pile_sighted", pose=pose,
                           sighting=PILE_DETECTOR.get_last_sighting())

            time.sleep(PAUSE_SECS)
            continue

        # ── drive one burst in the current heading ────────────────────────────
        pybot_scout.set_translate_2(heading, CHECK_INTERVAL_SECS)
        LOGGER.log(
            "move_burst",
            direction_deg=heading,
            burst_secs=CHECK_INTERVAL_SECS,
            sensor_active=sensor_active,
            readings=readings,
        )

        # Update stuck-detection buffer
        raw = _raw_tof(readings)
        if raw is not None:
            tof_buf.append(raw)
            if len(tof_buf) > STUCK_BURST_COUNT + 2:
                tof_buf.pop(0)

        # ── stuck detection ───────────────────────────────────────────────────
        if sensor_active and _check_stuck(tof_buf):
            print("Stuck detected (tof flat for %d bursts) – rotating to escape." %
                  STUCK_BURST_COUNT)
            LOGGER.log("stuck_detected",
                       readings=readings,
                       tof_buf=list(tof_buf),
                       heading_deg=heading)
            pybot_scout.stop_move()
            del tof_buf[:]
            rotate_dir = random.choice([1, 2])
            escape_deg = random.randint(150, 210)   # roughly 180°
            pybot_scout.set_rotate_3(rotate_dir, escape_deg)
            heading = (heading + (escape_deg if rotate_dir == 1 else -escape_deg)) % 360
            time.sleep(PAUSE_SECS)


# ── entry point ────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    signal.signal(signal.SIGINT,  _signal_handler)
    signal.signal(signal.SIGHUP,  _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    pybot_scout.start()

    try:
        start()
    except Exception as exc:
        LOGGER.log("run_exception", error=str(exc), error_type=exc.__class__.__name__)
        pybot_scout.handle_exception(exc.__class__.__name__ + ': ' + str(exc))

    stats = ODOM_TRACKER.get_stats()
    LOGGER.log("run_summary", **stats)
    ODOM_TRACKER.stop()
    PILE_DETECTOR.stop()
    CHARGER_STATUS.stop()
    LOGGER.log("run_stopped")
    LOGGER.close()
    pybot_scout.stop()
