# -*- coding: utf-8 -*-
"""
Long-running autonomous patrol script.

Implements a continuous charge cycle:
  1. If docked: wait until battery >= DEPART_PCT (default 80 %)
  2. Drive off the charger and explore using pong-ball obstacle avoidance
  3. Monitor battery every BATTERY_CHECK_SECS while exploring
  4. When battery drops below RETURN_PCT (default 50 %): stop exploring,
     navigate back to the charging station, and dock
  5. Repeat from step 1

Return-home strategy (tried in order):
  a. Publish to /navBackup (the same trigger the app "go home" button uses)
     and wait for /CoreNode/going_home_status to confirm completion.
  b. Replay the odometry breadcrumbs recorded during exploration in reverse
     (same approach as return_home.py).
  c. Visual scan: keep driving around until the charging pile is detected,
     then stop to allow the robot's own docking to take over.

All transitions and key events are written to run_feedback/ as JSONL so that
individual-script runs (obstacle_avoidance.py, return_home.py) can still be
used to refine each phase independently.

Usage:
    python scripts/patrol.py

Environment:
    PYBOT_SCOUT_DEPART_PCT                – min battery % before departing
                                            (default: 80)
    PYBOT_SCOUT_RETURN_PCT                – battery % that triggers return home
                                            (default: 50)
    PYBOT_SCOUT_BATTERY_CHECK_SECS        – how often to check battery while
                                            exploring (default: 30)
    PYBOT_SCOUT_ALLOW_SENSORLESS_FALLBACK – set to "1" to explore without
                                            proximity sensors
"""

import math
import os
import random
import signal
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT   = os.path.dirname(SCRIPT_DIR)
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from pybot_scout.battery        import BatteryMonitor
from pybot_scout.charging_pile  import ChargingPileDetector, ChargingStatusDetector
from pybot_scout.feedback       import FeedbackLogger
from pybot_scout.odometry       import OdometryTracker
from pybot_scout.proximity      import discover_proximity_topics
from pybot_scout import scanner as _scanner
from pybot_scout.scout          import pybot_scout

# ── tunable constants ──────────────────────────────────────────────────────────
DEPART_PCT           = float(os.environ.get("PYBOT_SCOUT_DEPART_PCT",          "80"))
RETURN_PCT           = float(os.environ.get("PYBOT_SCOUT_RETURN_PCT",          "50"))
BATTERY_CHECK_SECS   = float(os.environ.get("PYBOT_SCOUT_BATTERY_CHECK_SECS",  "30"))
ALLOW_SENSORLESS     = os.environ.get("PYBOT_SCOUT_ALLOW_SENSORLESS_FALLBACK", "0") == "1"

# Explore (pong) parameters
SPEED                = 0.3
CRAWL_SPEED          = 0.1     # m/s when obstacle is in warning zone
ROTATION_SPEED       = 90
OBSTACLE_THRESHOLD_M = 0.25   # stop and scan when obstacle within this distance
WARNING_THRESHOLD_M  = 0.50   # slow to CRAWL_SPEED when obstacle within this distance
MIN_VALID_M          = 0.15
CHECK_INTERVAL_SECS  = 0.30
PAUSE_SECS           = 0.30
SENSOR_TIMEOUT_SECS  = 5.0
REDISCOVERY_SECS     = 15.0

# Stuck detection: if tof readings stay within STUCK_EPSILON m for STUCK_BURST_COUNT
# consecutive normal-speed drive bursts while the motors are running, the robot is
# probably stuck (wheels spinning against a wall).  React by scanning for a way out.
STUCK_BURST_COUNT    = 5
STUCK_EPSILON        = 0.010  # m

# Charger exit / entry
CHARGER_EXIT_SECS    = 3.0    # drive straight to clear the dock on departure
RETURN_TIMEOUT_SECS  = 300.0  # give up on return-home after this many seconds

# Waypoint recording: only keep every Nth pose to avoid huge lists
WAYPOINT_STRIDE      = 5      # record one pose per N pong bursts

# How long to poll for going_home_status after triggering /navBackup
NAVBACKUP_WAIT_SECS  = 90.0

FEEDBACK_DIR  = os.path.join(REPO_ROOT, "run_feedback")
LOGGER        = FeedbackLogger("patrol", output_dir=FEEDBACK_DIR)
ODOM          = OdometryTracker()
PILE          = ChargingPileDetector()
CHARGER_IO    = ChargingStatusDetector()
BATTERY       = BatteryMonitor()

_shutdown = False


# ── signal handling ────────────────────────────────────────────────────────────

def _signal_handler(signum, frame):
    global _shutdown
    print("\nInterrupt received – stopping robot.")
    LOGGER.log("signal_received", signum=signum)
    _shutdown = True
    pybot_scout.stop()


# ── sensor helpers ─────────────────────────────────────────────────────────────

def _subscribe_topics(subscribed):
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
    deadline = time.time() + timeout_secs
    while time.time() < deadline:
        if any(v >= 0.0 for v in pybot_scout.get_proximity_readings().values()):
            return True
        time.sleep(0.1)
    return False


def _nearest_obstacle(readings):
    """Return closest valid obstacle distance (<OBSTACLE_THRESHOLD_M), or None if clear.

    Readings below MIN_VALID_M (floor / dock surface reflections) are ignored.
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
    """Return True when the last STUCK_BURST_COUNT valid tof readings span <= STUCK_EPSILON.

    This detects wheels spinning in place against an obstacle while the motors
    are running: the sensor distance stays constant even though we commanded motion.
    """
    if len(reading_buf) < STUCK_BURST_COUNT:
        return False
    tail = reading_buf[-STUCK_BURST_COUNT:]
    return (max(tail) - min(tail)) <= STUCK_EPSILON


# ── battery helpers ────────────────────────────────────────────────────────────

def _battery_ok_to_depart():
    """Return True when battery is known and >= DEPART_PCT, or unknown (fail-open)."""
    pct = BATTERY.get_percent()
    if pct is None:
        return True   # can't read level – don't wait indefinitely
    return pct >= DEPART_PCT


def _battery_low():
    """Return True when battery is known and < RETURN_PCT."""
    pct = BATTERY.get_percent()
    if pct is None:
        return False  # unknown – keep exploring
    return pct < RETURN_PCT


# ── phase 1 : wait for charge ─────────────────────────────────────────────────

def _wait_for_charge():
    """Block until battery level is >= DEPART_PCT or sensor gives no data."""
    LOGGER.log("wait_for_charge_started", target_pct=DEPART_PCT)
    print("Waiting on charger until battery >= %.0f %% …" % DEPART_PCT)
    poll_interval = 60.0   # check every minute
    last_logged   = 0.0

    while not _shutdown:
        pct = BATTERY.get_percent()
        chg = BATTERY.is_charging()

        if time.time() - last_logged >= poll_interval:
            LOGGER.log("charge_waiting",
                       battery_pct=pct,
                       charging=chg,
                       target_pct=DEPART_PCT)
            if pct is not None:
                print("  Battery: %.0f %%  (charging=%s,  target %.0f %%)" % (
                    pct, chg, DEPART_PCT))
            else:
                print("  Battery level unknown – waiting…")
            last_logged = time.time()

        if pct is None or pct >= DEPART_PCT:
            break

        time.sleep(5.0)

    pct = BATTERY.get_percent()
    LOGGER.log("wait_for_charge_done", battery_pct=pct)
    print("Ready to depart (battery=%.0f %%)." % (pct or 0))


# ── phase 2 : explore until battery low ──────────────────────────────────────

def _explore(subscribed, sensor_active):
    """Run the pong-ball exploration loop.

    Returns a list of (x_m, y_m, heading_deg) waypoints recorded during the
    run, for use by the return-home phase.
    """
    LOGGER.log("explore_started",
               depart_pct=DEPART_PCT,
               return_pct=RETURN_PCT,
               battery_check_secs=BATTERY_CHECK_SECS)
    print("Exploring…  will return home when battery < %.0f %%." % RETURN_PCT)

    heading           = random.randint(0, 359)
    waypoints         = []
    burst_count       = 0
    tof_buf           = []   # rolling raw tof readings during drive bursts (stuck detection)
    next_battery_check= time.time() + BATTERY_CHECK_SECS
    next_rediscovery  = time.time() + REDISCOVERY_SECS

    # Log initial pose as first waypoint
    waypoints.append(dict(ODOM.get_pose()))
    LOGGER.log("step_selected",
               direction_deg=heading,
               pose=waypoints[-1],
               sensor_active=sensor_active)

    while not _shutdown:
        # ── periodic battery check ────────────────────────────────────────────
        if time.time() >= next_battery_check:
            pct = BATTERY.get_percent()
            chg = BATTERY.is_charging()
            LOGGER.log("battery_check", battery_pct=pct, charging=chg,
                       heading_deg=heading, waypoints_recorded=len(waypoints))
            print("Battery: %s %%  (charging=%s)" % (
                "%.0f" % pct if pct is not None else "?", chg))
            next_battery_check = time.time() + BATTERY_CHECK_SECS

            if _battery_low():
                print("Battery low – stopping exploration to return home.")
                LOGGER.log("explore_stopped_low_battery", battery_pct=pct)
                pybot_scout.stop_move()
                break

        # ── periodic topic rediscovery ─────────────────────────────────────
        if time.time() >= next_rediscovery:
            subscribed = _subscribe_topics(subscribed)
            if not sensor_active:
                sensor_active = _wait_for_sensor_data(1.0)
            next_rediscovery = time.time() + REDISCOVERY_SECS

        if not sensor_active and not ALLOW_SENSORLESS:
            LOGGER.log("waiting_for_sensor_data")
            time.sleep(0.5)
            continue

        readings      = pybot_scout.get_proximity_readings() if sensor_active else {}
        obstacle_dist = _nearest_obstacle(readings) if sensor_active else None
        warning_dist  = _nearest_valid(readings)    if sensor_active else None

        if obstacle_dist is not None:
            # ── danger zone: stop, scan, turn to clearest heading ─────────────
            pybot_scout.stop_move()
            del tof_buf[:]
            print("Obstacle at %.2f m – scanning for clearest direction…" % obstacle_dist)
            LOGGER.log("scan_triggered",
                       obstacle_m=round(obstacle_dist, 3),
                       heading_deg=heading,
                       readings=readings)

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

            # Record new heading as a breadcrumb waypoint
            pose = dict(ODOM.get_pose())
            waypoints.append(pose)
            LOGGER.log("step_selected",
                       direction_deg=heading,
                       pose=pose,
                       sensor_active=sensor_active)

            if PILE.was_recently_seen():
                LOGGER.log("charging_pile_sighted", pose=pose,
                           sighting=PILE.get_last_sighting())

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
            continue

        # ── clear path: normal drive burst ────────────────────────────────────
        pybot_scout.set_translate_2(heading, CHECK_INTERVAL_SECS)
        burst_count += 1
        LOGGER.log("move_burst",
                   direction_deg=heading,
                   burst_secs=CHECK_INTERVAL_SECS,
                   sensor_active=sensor_active,
                   readings=readings)

        # Update stuck-detection buffer with the raw tof value seen this burst
        raw = _raw_tof(readings)
        if raw is not None:
            tof_buf.append(raw)
            if len(tof_buf) > STUCK_BURST_COUNT + 2:
                tof_buf.pop(0)

        # ── stuck detection ────────────────────────────────────────────────
        if sensor_active and _check_stuck(tof_buf):
            print("Stuck detected – scanning for escape direction.")
            LOGGER.log("stuck_detected",
                       readings=readings,
                       tof_buf=list(tof_buf),
                       heading_deg=heading)
            pybot_scout.stop_move()
            del tof_buf[:]
            best_delta, _ = _scanner.scan_for_best_heading(pybot_scout, LOGGER)
            old_heading = heading
            heading = (heading + best_delta) % 360
            LOGGER.log("stuck_escape",
                       old_heading_deg=old_heading,
                       new_heading_deg=heading,
                       best_heading_delta_deg=best_delta)
            time.sleep(PAUSE_SECS)

        # Record a waypoint every WAYPOINT_STRIDE bursts
        if burst_count % WAYPOINT_STRIDE == 0:
            waypoints.append(dict(ODOM.get_pose()))

    LOGGER.log("explore_finished",
               waypoints_recorded=len(waypoints),
               battery_pct=BATTERY.get_percent())
    return waypoints


# ── phase 3 : return home ─────────────────────────────────────────────────────

def _trigger_navbackup():
    """Publish to /navBackup to trigger the robot's built-in go-home.

    Returns True if the topic was published successfully.
    """
    try:
        import rospy
        from std_msgs.msg import Int32
        pub = rospy.Publisher("/navBackup", Int32, queue_size=1, latch=True)
        time.sleep(0.3)   # let the publisher register
        pub.publish(Int32(data=1))
        LOGGER.log("navbackup_triggered")
        return True
    except Exception as exc:
        LOGGER.log("navbackup_failed", error=str(exc))
        return False


def _wait_for_going_home_done(timeout_secs):
    """Subscribe to /CoreNode/going_home_status; block until non-zero or timeout."""
    done_flag = [False]
    status_val = [None]
    sub = None
    try:
        import rospy
        from std_msgs.msg import Int32

        def _cb(msg):
            status_val[0] = msg.data
            # 0 = idle/not started, non-zero usually means active or done
            if msg.data != 0:
                done_flag[0] = True

        sub = rospy.Subscriber("/CoreNode/going_home_status", Int32, _cb)
        deadline = time.time() + timeout_secs
        while time.time() < deadline and not done_flag[0] and not _shutdown:
            time.sleep(0.5)
    except Exception:
        pass
    finally:
        if sub is not None:
            try:
                sub.unregister()
            except Exception:
                pass

    LOGGER.log("going_home_status_result",
               status=status_val[0],
               done=done_flag[0],
               timeout=not done_flag[0])
    return done_flag[0]


def _dist(a, b):
    return math.sqrt((a["x_m"] - b["x_m"]) ** 2 + (a["y_m"] - b["y_m"]) ** 2)


def _normalise_angle(deg):
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


def _waypoint_return(waypoints):
    """Drive back along recorded waypoints in reverse order.

    Mirrors the logic in return_home.py.  Stops early when:
      - The charging pile is visually detected.
      - The robot is within 0.25 m of the first recorded waypoint.
    """
    if len(waypoints) < 2:
        LOGGER.log("waypoint_return_skipped", reason="too_few_waypoints",
                   count=len(waypoints))
        return False

    goal = waypoints[0]
    LOGGER.log("waypoint_return_started",
               waypoint_count=len(waypoints),
               goal_x=goal.get("x_m"), goal_y=goal.get("y_m"))
    print("Replaying %d waypoints in reverse to reach home." % len(waypoints))

    GOAL_RADIUS = 0.25
    MIN_SEGMENT = 0.05
    pybot_scout.set_translationSpeed(0.2)
    pybot_scout.set_rotationSpeed(60)

    reverse = list(reversed(waypoints))
    for idx in range(len(reverse) - 1):
        if _shutdown:
            break

        if PILE.was_recently_seen():
            LOGGER.log("pile_detected_during_return",
                       live_pose=dict(ODOM.get_pose()))
            print("Charging pile detected – stopping for docking.")
            break

        live = ODOM.get_pose()
        if _dist(live, goal) <= GOAL_RADIUS:
            LOGGER.log("waypoint_return_arrived",
                       dist_m=round(_dist(live, goal), 3))
            print("Arrived within %.2f m of start." % GOAL_RADIUS)
            break

        cur = reverse[idx]
        nxt = reverse[idx + 1]
        dx  = nxt.get("x_m", 0) - cur.get("x_m", 0)
        dy  = nxt.get("y_m", 0) - cur.get("y_m", 0)
        seg = math.sqrt(dx * dx + dy * dy)
        if seg < MIN_SEGMENT:
            continue

        target_hdg = math.degrees(math.atan2(dy, dx))
        delta      = _normalise_angle(target_hdg - live.get("heading_deg", 0.0))
        if abs(delta) >= 3.0:
            pybot_scout.set_rotate_3(1 if delta > 0 else 2, int(abs(delta)))

        pybot_scout.set_translate_3(0, seg)
        LOGGER.log("waypoint_segment", idx=idx, dist_m=round(seg, 3))
        time.sleep(0.3)

    LOGGER.log("waypoint_return_finished")
    return True


def _visual_scan_for_charger(timeout_secs):
    """Drive around looking for the charging pile (visual scan fallback).

    Uses pong-ball movement.  Stops when the pile is detected or timeout
    expires.  Returns True if the pile was detected.
    """
    LOGGER.log("visual_scan_started", timeout_secs=timeout_secs)
    print("Visual scan: driving around to find the charging pile…")

    heading   = random.randint(0, 359)
    deadline  = time.time() + timeout_secs
    sensor_ok = any(v >= 0.0 for v in pybot_scout.get_proximity_readings().values())

    pybot_scout.set_translationSpeed(SPEED)
    pybot_scout.set_rotationSpeed(ROTATION_SPEED)

    while time.time() < deadline and not _shutdown:
        if PILE.was_recently_seen():
            LOGGER.log("visual_scan_found_pile")
            return True

        readings      = pybot_scout.get_proximity_readings() if sensor_ok else {}
        obstacle_dist = _nearest_obstacle(readings) if sensor_ok else None
        warning_dist  = _nearest_valid(readings)    if sensor_ok else None

        if obstacle_dist is not None:
            LOGGER.log("visual_scan_obstacle", obstacle_m=round(obstacle_dist, 3))
            pybot_scout.stop_move()
            best_delta, _ = _scanner.scan_for_best_heading(pybot_scout, LOGGER)
            heading = (heading + best_delta) % 360
            time.sleep(PAUSE_SECS)
            continue

        if warning_dist is not None and warning_dist < WARNING_THRESHOLD_M:
            pybot_scout.set_translationSpeed(CRAWL_SPEED)
            pybot_scout.set_translate_2(heading, CHECK_INTERVAL_SECS)
            pybot_scout.set_translationSpeed(SPEED)
            continue

        pybot_scout.set_translate_2(heading, CHECK_INTERVAL_SECS)

    LOGGER.log("visual_scan_timeout")
    return False


def _return_home(waypoints):
    """Orchestrate the full return-to-charger sequence."""
    LOGGER.log("return_home_started", battery_pct=BATTERY.get_percent())
    print("Starting return-home sequence.")
    pybot_scout.stop_move()

    # ── step 1: try built-in /navBackup ──────────────────────────────────────
    triggered = _trigger_navbackup()
    if triggered:
        print("Triggered built-in go-home via /navBackup.  Waiting…")
        done = _wait_for_going_home_done(NAVBACKUP_WAIT_SECS)
        if done:
            LOGGER.log("return_home_via_navbackup_succeeded")
            return
        print("Built-in go-home did not confirm success within %.0f s." %
              NAVBACKUP_WAIT_SECS)

    # ── step 2: waypoint replay ───────────────────────────────────────────────
    pybot_scout.set_translationSpeed(SPEED)
    _waypoint_return(waypoints)

    # ── step 3: visual scan if still not docked ───────────────────────────────
    if not CHARGER_IO.wait_for_status(timeout_secs=2.0):
        _visual_scan_for_charger(timeout_secs=120.0)

    LOGGER.log("return_home_finished",
               charging=BATTERY.is_charging(),
               battery_pct=BATTERY.get_percent())
    print("Return-home sequence complete.")


# ── main cycle ─────────────────────────────────────────────────────────────────

def run_patrol():
    # ── setup ─────────────────────────────────────────────────────────────────
    ODOM.start()
    PILE.start()
    CHARGER_IO.start()
    BATTERY.start(logger=LOGGER)

    pybot_scout.set_rotationSpeed(ROTATION_SPEED)
    pybot_scout.set_translationSpeed(SPEED)

    subscribed = _subscribe_topics(set())
    LOGGER.log(
        "patrol_started",
        depart_pct=DEPART_PCT,
        return_pct=RETURN_PCT,
        battery_check_secs=BATTERY_CHECK_SECS,
        obstacle_threshold_m=OBSTACLE_THRESHOLD_M,
        warning_threshold_m=WARNING_THRESHOLD_M,
        min_valid_m=MIN_VALID_M,
        crawl_speed=CRAWL_SPEED,
        allow_sensorless=ALLOW_SENSORLESS,
        scan_half_angle_deg=_scanner.SCAN_HALF_ANGLE_DEG,
        scan_step_deg=_scanner.SCAN_STEP_DEG,
        proximity_topics=sorted(subscribed),
    )

    print("Patrol started.  Depart at %.0f %% / Return at %.0f %%.  "
          "Press Ctrl-C to stop." % (DEPART_PCT, RETURN_PCT))

    print("Waiting up to %.0f s for proximity sensor data…" % SENSOR_TIMEOUT_SECS)
    sensor_active = _wait_for_sensor_data(SENSOR_TIMEOUT_SECS)
    LOGGER.log("sensor_wait_completed",
               sensor_active=sensor_active,
               readings=pybot_scout.get_proximity_readings())

    # ── determine initial state ───────────────────────────────────────────────
    pct, charging = BATTERY.wait_for_status(timeout_secs=3.0)
    LOGGER.log("initial_battery_status", battery_pct=pct, charging=charging)

    # If battery data unavailable, check charger via tof heuristic
    on_charger = charging
    if on_charger is None:
        chg_status = CHARGER_IO.wait_for_status(timeout_secs=2.0)
        on_charger = bool(chg_status)

    cycle = 0

    while not _shutdown:
        cycle += 1
        LOGGER.log("patrol_cycle_started", cycle=cycle,
                   battery_pct=BATTERY.get_percent(),
                   on_charger=on_charger)
        print("\n── Patrol cycle %d ──────────────────────────────────────" % cycle)

        # ── wait for sufficient charge before departing ───────────────────────
        if on_charger:
            pct = BATTERY.get_percent()
            if pct is not None and pct < DEPART_PCT:
                _wait_for_charge()
            else:
                print("Battery %s %%  (>= %.0f %% target) – departing now." % (
                    "%.0f" % pct if pct is not None else "unknown", DEPART_PCT))

            # Drive off the charger before beginning pong movement
            print("Driving off charging station…")
            LOGGER.log("charger_exit_started", exit_secs=CHARGER_EXIT_SECS)
            pybot_scout.set_translate_smooth(0, CHARGER_EXIT_SECS)
            time.sleep(CHARGER_EXIT_SECS + 0.2)
            LOGGER.log("charger_exit_completed")

        # ── explore (pong) ────────────────────────────────────────────────────
        waypoints = _explore(subscribed, sensor_active)

        if _shutdown:
            break

        # ── return home ───────────────────────────────────────────────────────
        _return_home(waypoints)

        on_charger = True   # assume we made it back

        LOGGER.log("patrol_cycle_finished", cycle=cycle,
                   battery_pct=BATTERY.get_percent())
        print("Cycle %d complete." % cycle)


# ── entry point ────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    signal.signal(signal.SIGINT,  _signal_handler)
    signal.signal(signal.SIGHUP,  _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    pybot_scout.start()

    try:
        run_patrol()
    except Exception as exc:
        LOGGER.log("run_exception", error=str(exc),
                   error_type=exc.__class__.__name__)
        pybot_scout.handle_exception(exc.__class__.__name__ + ": " + str(exc))

    stats = ODOM.get_stats()
    LOGGER.log("patrol_stopped", **stats)
    ODOM.stop()
    PILE.stop()
    CHARGER_IO.stop()
    BATTERY.stop()
    LOGGER.log("run_stopped")
    LOGGER.close()
    pybot_scout.stop()
