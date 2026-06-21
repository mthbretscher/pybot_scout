# -*- coding: utf-8 -*-
"""
Pong-ball explorer with directional-scan avoidance.

The robot travels in a straight line until the proximity sensor reports an
obstacle within OBSTACLE_THRESHOLD_M.  It then stops and performs a short
forward-cone scan (left/right around current heading), sampling ToF with
time-averaging to suppress noise, then nudges toward the clearest direction.

When an obstacle is detected in the wider WARNING_THRESHOLD_M zone the robot
slows to CRAWL_SPEED before the hard stop, giving a smooth deceleration.

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

import math
import os
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
from pybot_scout.dashboard import ScriptDashboard
from pybot_scout import scanner as _scanner
from pybot_scout.scout import pybot_scout

# ── tunable constants ──────────────────────────────────────────────────────────
SPEED                = 0.3     # m/s forward speed (normal)
CRAWL_SPEED          = 0.1     # m/s in the warning zone (smooth deceleration)
ROTATION_SPEED       = 90      # deg/s rotation speed
OBSTACLE_THRESHOLD_M = 0.25    # stop and scan when obstacle is within this distance
WARNING_THRESHOLD_M  = 0.50    # slow to CRAWL_SPEED when obstacle is within this distance
MIN_VALID_M          = 0.15    # ignore readings below this (floor / charger noise)
CHECK_INTERVAL_SECS  = 0.30    # drive-burst length; sensor checked between bursts
PAUSE_SECS           = 0.30    # brief stop between a scan-turn and the next move
SENSOR_TIMEOUT_SECS  = 5.0     # wait this long for the first valid sensor reading
CHARGER_EXIT_SECS    = 3.0     # drive straight for this long to clear the dock
REDISCOVERY_SECS     = 15.0    # re-scan for proximity topics this often

# Stuck detection: if tof readings stay within STUCK_EPSILON m for STUCK_BURST_COUNT
# consecutive normal-speed drive bursts the robot is probably stuck.
STUCK_BURST_COUNT    = 5
STUCK_EPSILON        = 0.010   # m

ALLOW_SENSORLESS = os.environ.get("PYBOT_SCOUT_ALLOW_SENSORLESS_FALLBACK", "0") == "1"

FEEDBACK_DIR   = os.path.join(REPO_ROOT, "run_feedback")
LOGGER         = FeedbackLogger("obstacle_avoidance", output_dir=FEEDBACK_DIR)
ODOM_TRACKER   = OdometryTracker()
PILE_DETECTOR  = ChargingPileDetector()
CHARGER_STATUS = ChargingStatusDetector()
DASHBOARD      = ScriptDashboard("obstacle_avoidance", logger=LOGGER)


# ── signal handling ────────────────────────────────────────────────────────────

def _signal_handler(signum, frame):
    print("\nInterrupt received – stopping robot.")
    LOGGER.log("signal_received", signum=signum)
    DASHBOARD.close()
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
    """Return the closest valid obstacle distance below OBSTACLE_THRESHOLD_M,
    or None if the danger zone is clear.

    Readings below MIN_VALID_M (floor / dock surface noise) are ignored.
    """
    nearest = None
    for dist in readings.values():
        if MIN_VALID_M <= dist < OBSTACLE_THRESHOLD_M:
            if nearest is None or dist < nearest:
                nearest = dist
    return nearest


def _nearest_valid(readings):
    """Return the closest non-noise, finite reading across all topics, or None.

    Used to detect the warning zone (OBSTACLE_THRESHOLD_M .. WARNING_THRESHOLD_M).
    """
    nearest = None
    for dist in readings.values():
        if dist >= MIN_VALID_M and not math.isinf(dist):
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
    DASHBOARD.start()

    subscribed = _subscribe_topics(set())
    LOGGER.log(
        "run_started",
        speed=SPEED,
        crawl_speed=CRAWL_SPEED,
        rotation_speed=ROTATION_SPEED,
        obstacle_threshold_m=OBSTACLE_THRESHOLD_M,
        warning_threshold_m=WARNING_THRESHOLD_M,
        min_valid_m=MIN_VALID_M,
        check_interval_secs=CHECK_INTERVAL_SECS,
        sensor_timeout_secs=SENSOR_TIMEOUT_SECS,
        charger_exit_secs=CHARGER_EXIT_SECS,
        allow_sensorless=ALLOW_SENSORLESS,
        scan_half_angle_deg=_scanner.SCAN_HALF_ANGLE_DEG,
        scan_step_deg=_scanner.SCAN_STEP_DEG,
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
    DASHBOARD.update_state(
        heading_deg=0,
        mode="sensor_wait_done",
        sensor_active=sensor_active,
        allow_sensorless=ALLOW_SENSORLESS,
    )
    DASHBOARD.update_sensors(pybot_scout.get_proximity_readings())
    DASHBOARD.tick(force=True)

    # ── leave the dock first ───────────────────────────────────────────────────
    _exit_charger_if_needed()

    # ── set initial heading (forward / 0°) ───────────────────────────────────────
    # The first scan will immediately orient to the clearest direction if needed.
    heading = 0
    print("Pong explorer started.  Initial heading: %d°.  Press Ctrl-C to stop." % heading)

    next_rediscovery = time.time() + REDISCOVERY_SECS
    tof_buf = []   # rolling raw tof readings for stuck detection (normal bursts only)

    while True:
        # ── periodic topic rediscovery (helps if sensors come online late) ────
        if time.time() >= next_rediscovery:
            subscribed = _subscribe_topics(subscribed)
            if not sensor_active:
                sensor_active = _wait_for_sensor_data(1.0)
            next_rediscovery = time.time() + REDISCOVERY_SECS

        if not sensor_active and not ALLOW_SENSORLESS:
            LOGGER.log("waiting_for_sensor_data")
            DASHBOARD.update_state(
                mode="waiting_for_sensor_data",
                heading_deg=heading,
                sensor_active=sensor_active,
                subscribed_topics=len(subscribed),
            )
            DASHBOARD.update_sensors(pybot_scout.get_proximity_readings())
            DASHBOARD.tick()
            time.sleep(0.5)
            continue

        readings      = pybot_scout.get_proximity_readings() if sensor_active else {}
        obstacle_dist = _nearest_obstacle(readings)           if sensor_active else None
        warning_dist  = _nearest_valid(readings)              if sensor_active else None
        DASHBOARD.update_sensors(readings)
        DASHBOARD.update_state(
            mode="drive_loop",
            heading_deg=heading,
            sensor_active=sensor_active,
            subscribed_topics=len(subscribed),
            obstacle_m=obstacle_dist,
            warning_m=warning_dist,
        )
        DASHBOARD.tick()

        if obstacle_dist is not None:
            # ── danger zone: stop, scan, turn to clearest heading ─────────────
            pybot_scout.stop_move()
            del tof_buf[:]
            print("Obstacle at %.2f m – scanning for clearest direction…" % obstacle_dist)
            LOGGER.log("scan_triggered",
                       obstacle_m=round(obstacle_dist, 3),
                       heading_deg=heading,
                       readings=readings)
            DASHBOARD.update_state(mode="scan_triggered", obstacle_m=obstacle_dist)
            DASHBOARD.tick()

            best_delta, scan_results = _scanner.scan_for_best_heading(pybot_scout, LOGGER)
            old_heading = heading
            heading = (heading + best_delta) % 360

            print("Scan: %d° → %d°  (best clearance at delta %+d°)" % (
                old_heading, heading, best_delta))
            LOGGER.log("scan_bounce",
                       old_heading_deg=old_heading,
                       new_heading_deg=heading,
                       best_heading_delta_deg=best_delta,
                       obstacle_m=round(obstacle_dist, 3))
            DASHBOARD.update_state(mode="scan_bounce", heading_deg=heading)
            DASHBOARD.tick()

            pose = ODOM_TRACKER.get_pose()
            LOGGER.log("step_selected", direction_deg=heading, pose=pose,
                       sensor_active=sensor_active)

            if PILE_DETECTOR.was_recently_seen():
                LOGGER.log("charging_pile_sighted", pose=pose,
                           sighting=PILE_DETECTOR.get_last_sighting())

            time.sleep(PAUSE_SECS)
            continue

        if warning_dist is not None and warning_dist < WARNING_THRESHOLD_M:
            # ── warning zone: slow approach (smooth deceleration) ─────────────
            del tof_buf[:]   # don't mix crawl bursts into stuck-detection buffer
            pybot_scout.set_translationSpeed(CRAWL_SPEED)
            pybot_scout.set_translate_2(heading, CHECK_INTERVAL_SECS)
            pybot_scout.set_translationSpeed(SPEED)
            LOGGER.log("move_burst_crawl",
                       direction_deg=heading,
                       warning_dist_m=round(warning_dist, 3),
                       readings=readings)
            DASHBOARD.update_state(mode="crawl", heading_deg=heading, warning_m=warning_dist)
            DASHBOARD.tick()
            continue

        # ── clear path: normal drive burst ────────────────────────────────────
        pybot_scout.set_translate_2(heading, CHECK_INTERVAL_SECS)
        LOGGER.log(
            "move_burst",
            direction_deg=heading,
            burst_secs=CHECK_INTERVAL_SECS,
            sensor_active=sensor_active,
            readings=readings,
        )
        DASHBOARD.update_state(mode="move_burst", heading_deg=heading)
        DASHBOARD.tick()

        # Update stuck-detection buffer with the raw tof value seen this burst
        raw = _raw_tof(readings)
        if raw is not None:
            tof_buf.append(raw)
            if len(tof_buf) > STUCK_BURST_COUNT + 2:
                tof_buf.pop(0)

        # ── stuck detection ───────────────────────────────────────────────────
        if sensor_active and _check_stuck(tof_buf):
            print("Stuck detected – scanning for escape direction.")
            LOGGER.log("stuck_detected",
                       readings=readings,
                       tof_buf=list(tof_buf),
                       heading_deg=heading)
            DASHBOARD.update_state(mode="stuck_detected", heading_deg=heading)
            DASHBOARD.tick()
            pybot_scout.stop_move()
            del tof_buf[:]
            best_delta, _ = _scanner.scan_for_best_heading(pybot_scout, LOGGER)
            old_heading = heading
            heading = (heading + best_delta) % 360
            LOGGER.log("stuck_escape",
                       old_heading_deg=old_heading,
                       new_heading_deg=heading,
                       best_heading_delta_deg=best_delta)
            DASHBOARD.update_state(mode="stuck_escape", heading_deg=heading)
            DASHBOARD.tick()
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
    DASHBOARD.close()
    ODOM_TRACKER.stop()
    PILE_DETECTOR.stop()
    CHARGER_STATUS.stop()
    LOGGER.log("run_stopped")
    LOGGER.close()
    pybot_scout.stop()
