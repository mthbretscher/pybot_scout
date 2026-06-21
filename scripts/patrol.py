# -*- coding: utf-8 -*-
"""
Long-running autonomous patrol script.

Implements a continuous charge cycle:
  1. If docked: wait until battery >= DEPART_PCT (default 80 %)
  2. Drive off the charger and explore using camera-brightness steering
  3. Monitor battery every BATTERY_CHECK_SECS while exploring
  4. When battery drops below RETURN_PCT (default 50 %): stop exploring,
     navigate back to the charging station, and dock
  5. Repeat from step 1

Return-home strategy (tried in order):
  a. Publish to /navBackup (the same trigger the app "go home" button uses)
     and wait for /CoreNode/going_home_status to confirm completion.
  b. Replay the odometry breadcrumbs recorded during exploration in reverse
     (same approach as return_home.py).
  c. Visual scan: keep driving around using camera steering until the charging pile is detected,
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
import signal
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT   = os.path.dirname(SCRIPT_DIR)
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from pybot_scout.battery        import BatteryMonitor
from pybot_scout.camera         import BrightnessWindow, GreyImageMonitor
from pybot_scout.charging_pile  import ChargingPileDetector, ChargingStatusDetector
from pybot_scout.feedback       import FeedbackLogger
from pybot_scout.odometry       import OdometryTracker
from pybot_scout.proximity      import discover_proximity_topics
from pybot_scout.dashboard      import ScriptDashboard
from pybot_scout.ros_inventory  import log_ros_inventory
from pybot_scout.scout          import pybot_scout

# ── tunable constants ──────────────────────────────────────────────────────────
DEPART_PCT           = float(os.environ.get("PYBOT_SCOUT_DEPART_PCT",          "80"))
RETURN_PCT           = float(os.environ.get("PYBOT_SCOUT_RETURN_PCT",          "50"))
BATTERY_CHECK_SECS   = float(os.environ.get("PYBOT_SCOUT_BATTERY_CHECK_SECS",  "30"))
ALLOW_SENSORLESS     = os.environ.get("PYBOT_SCOUT_ALLOW_SENSORLESS_FALLBACK", "0") == "1"

# Explore (camera steering) parameters
SPEED                = 0.20    # m/s forward speed (low but continuous)
CRAWL_SPEED          = 0.1     # m/s when obstacle is in warning zone
ROTATION_SPEED       = 90
MAX_STEER_DEG        = 45.0
STEER_GAIN           = 28.0
DARK_SLOW_BRIGHTNESS = 60.0
DARK_PIVOT_BRIGHTNESS = 40.0
CHECK_INTERVAL_SECS  = 0.30
PAUSE_SECS           = 0.30
CAMERA_TIMEOUT_SECS  = 5.0
REDISCOVERY_SECS     = 15.0

# Delta-based obstacle approach steering
DELTA_APPROACH_THRESHOLD = 0.025  # delta_fwd/base ratio that triggers approach boost
DELTA_STEER_BOOST        = 3.0    # multiplier applied to lateral_norm when approaching
DELTA_WINDOW_SPAN        = 5      # number of readings over which to compute deltas

# Stuck detection and recovery
STUCK_WINDOW_SIZE       = 20    # brightness readings in the stuck-detection window (~6 s)
STUCK_BRIGHTNESS_SPREAD = 5.0   # max(fwd) - min(fwd) below this → scene not changing
STUCK_DIST_THRESHOLD_M  = 0.20  # odometry distance that confirms NOT stuck
STUCK_COOLDOWN_SECS     = 15.0  # minimum seconds between successive stuck-recoveries
STUCK_ROTATE_DEG        = 90    # degrees to rotate during recovery
STUCK_DRIVE_SECS        = 2.0   # seconds to drive straight after recovery rotation

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
DASHBOARD     = ScriptDashboard("patrol", logger=LOGGER)
CAMERA        = GreyImageMonitor()

_shutdown = False


# ── signal handling ────────────────────────────────────────────────────────────

def _signal_handler(signum, frame):
    global _shutdown
    print("\nInterrupt received – stopping robot.")
    LOGGER.log("signal_received", signum=signum)
    _shutdown = True
    DASHBOARD.close()
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


def _wait_for_camera_data(timeout_secs):
    deadline = time.time() + timeout_secs
    while time.time() < deadline:
        if CAMERA.has_data():
            return True
        time.sleep(0.1)
    return False


def _safe_float(value):
    try:
        return float(value)
    except Exception:
        return None


def _clamp(value, lo, hi):
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def _camera_drive_command(camera_stats, delta_fwd=0.0, delta_left=0.0, delta_right=0.0):
    if not camera_stats:
        return None
    left = _safe_float(camera_stats.get("lower_left_mean_brightness"))
    center = _safe_float(camera_stats.get("lower_center_mean_brightness"))
    right = _safe_float(camera_stats.get("lower_right_mean_brightness"))
    if left is None or center is None or right is None:
        return None

    base = max(left, center, right, 1.0)
    lateral_norm = (right - left) / base

    # Forward-approach boost: when the scene ahead is brightening, steer toward
    # whichever side is currently darker (more open space).
    delta_fwd_norm = max(0.0, delta_fwd) / base
    if delta_fwd_norm > DELTA_APPROACH_THRESHOLD:
        darker_bias = 1.0 if left < right else -1.0
        lateral_norm += darker_bias * delta_fwd_norm * DELTA_STEER_BOOST

    # Side-approach correction: steer away from a side that is rapidly brightening.
    lateral_norm += max(0.0, delta_left) / base * DELTA_STEER_BOOST * 0.5
    lateral_norm -= max(0.0, delta_right) / base * DELTA_STEER_BOOST * 0.5

    steer = STEER_GAIN * lateral_norm
    heading_deg = _clamp(steer, -MAX_STEER_DEG, MAX_STEER_DEG)
    speed = CRAWL_SPEED if center < DARK_SLOW_BRIGHTNESS else SPEED
    pivot = bool(center < DARK_PIVOT_BRIGHTNESS and abs(lateral_norm) > 0.08)
    return {
        "left": left,
        "center": center,
        "right": right,
        "lateral_norm": lateral_norm,
        "center_conf": _clamp(center / base, 0.0, 1.0),
        "heading_deg": heading_deg,
        "speed": speed,
        "pivot": pivot,
        "delta_fwd": round(delta_fwd, 3),
        "delta_left": round(delta_left, 3),
        "delta_right": round(delta_right, 3),
    }


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
    DASHBOARD.update_state(mode="wait_for_charge", target_depart_pct=DEPART_PCT)
    DASHBOARD.tick()
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
            DASHBOARD.update_state(
                mode="wait_for_charge",
                battery_pct=pct,
                charging=chg,
                target_depart_pct=DEPART_PCT,
            )
            DASHBOARD.tick()

        if pct is None or pct >= DEPART_PCT:
            break

        time.sleep(5.0)

    pct = BATTERY.get_percent()
    LOGGER.log("wait_for_charge_done", battery_pct=pct)
    DASHBOARD.update_state(mode="wait_for_charge_done", battery_pct=pct)
    DASHBOARD.tick()
    print("Ready to depart (battery=%.0f %%)." % (pct or 0))


# ── phase 2 : explore until battery low ──────────────────────────────────────

def _explore(subscribed, camera_active):
    """Run the pong-ball exploration loop.

    Returns a list of (x_m, y_m, heading_deg) waypoints recorded during the
    run, for use by the return-home phase.
    """
    LOGGER.log("explore_started",
               depart_pct=DEPART_PCT,
               return_pct=RETURN_PCT,
               battery_check_secs=BATTERY_CHECK_SECS)
    print("Exploring…  will return home when battery < %.0f %%." % RETURN_PCT)
    DASHBOARD.update_state(
        mode="explore_started",
        return_pct=RETURN_PCT,
        camera_active=camera_active,
        subscribed_topics=len(subscribed),
    )
    DASHBOARD.tick()

    heading           = 0.0
    waypoints         = []
    burst_count       = 0
    next_battery_check= time.time() + BATTERY_CHECK_SECS
    next_rediscovery  = time.time() + REDISCOVERY_SECS
    bw                = BrightnessWindow(STUCK_WINDOW_SIZE)
    last_stuck_ts     = 0.0
    odom_dist_at_window_fill = None

    # Log initial pose as first waypoint
    waypoints.append(dict(ODOM.get_pose()))
    LOGGER.log("step_selected",
               direction_deg=heading,
               pose=waypoints[-1],
               camera_active=camera_active)

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
            DASHBOARD.update_state(
                mode="battery_check",
                battery_pct=pct,
                charging=chg,
                heading_deg=heading,
                waypoints_recorded=len(waypoints),
            )
            DASHBOARD.tick()

            if _battery_low():
                print("Battery low – stopping exploration to return home.")
                LOGGER.log("explore_stopped_low_battery", battery_pct=pct)
                pybot_scout.stop_move()
                break

        # ── periodic topic rediscovery ─────────────────────────────────────
        if time.time() >= next_rediscovery:
            subscribed = _subscribe_topics(subscribed)
            next_rediscovery = time.time() + REDISCOVERY_SECS

        if not camera_active:
            camera_active = _wait_for_camera_data(0.2)

        if not camera_active and not ALLOW_SENSORLESS:
            LOGGER.log("waiting_for_camera_data")
            DASHBOARD.update_state(
                mode="waiting_for_camera_data",
                heading_deg=heading,
                camera_active=camera_active,
                subscribed_topics=len(subscribed),
                battery_pct=BATTERY.get_percent(),
                charging=BATTERY.is_charging(),
            )
            DASHBOARD.update_sensors(pybot_scout.get_proximity_readings())
            DASHBOARD.tick()
            time.sleep(0.5)
            continue

        readings = pybot_scout.get_proximity_readings()
        camera_stats = CAMERA.get_snapshot()
        delta_fwd, delta_left, delta_right = bw.get_delta(DELTA_WINDOW_SPAN)
        drive = _camera_drive_command(camera_stats, delta_fwd, delta_left, delta_right)
        DASHBOARD.update_sensors(readings)
        DASHBOARD.update_state(
            mode="explore_loop",
            heading_deg=heading,
            camera_active=camera_active,
            subscribed_topics=len(subscribed),
            camera_left=(drive or {}).get("left"),
            camera_center=(drive or {}).get("center"),
            camera_right=(drive or {}).get("right"),
            battery_pct=BATTERY.get_percent(),
            charging=BATTERY.is_charging(),
        )
        DASHBOARD.tick()

        if drive is None:
            if not ALLOW_SENSORLESS:
                LOGGER.log("camera_drive_missing", camera_stats=camera_stats)
                time.sleep(0.3)
                continue
            drive = {
                "heading_deg": 0.0,
                "speed": CRAWL_SPEED,
                "left": None,
                "center": None,
                "right": None,
                "lateral_norm": 0.0,
                "center_conf": 0.0,
                "pivot": False,
                "delta_fwd": 0.0,
                "delta_left": 0.0,
                "delta_right": 0.0,
            }

        heading = drive.get("heading_deg", 0.0)

        # ── stuck detection ────────────────────────────────────────────────────
        if bw.is_stuck() and time.time() - last_stuck_ts > STUCK_COOLDOWN_SECS:
            odom_stats = ODOM.get_stats()
            odom_pose = ODOM.get_pose()
            do_unstick = True
            dist_since_fill = None
            if odom_pose["has_data"] and odom_dist_at_window_fill is not None:
                dist_since_fill = odom_stats["total_distance_m"] - odom_dist_at_window_fill
                if dist_since_fill > STUCK_DIST_THRESHOLD_M:
                    do_unstick = False
            LOGGER.log(
                "stuck_check",
                do_unstick=do_unstick,
                dist_since_fill=dist_since_fill,
                odom_has_data=odom_pose["has_data"],
            )
            if do_unstick:
                rot_dir = 1 if int(time.time()) % 2 == 0 else 2
                LOGGER.log("stuck_recovery_started",
                           rot_dir=rot_dir, rot_deg=STUCK_ROTATE_DEG,
                           drive_secs=STUCK_DRIVE_SECS)
                print("Stuck detected – rotating %d° and driving forward." % STUCK_ROTATE_DEG)
                pybot_scout.stop_move()
                pybot_scout.set_rotate_3(rot_dir, STUCK_ROTATE_DEG)
                time.sleep(float(STUCK_ROTATE_DEG) / ROTATION_SPEED + 0.3)
                pybot_scout.set_translationSpeed(SPEED)
                pybot_scout.set_translate_2(0, STUCK_DRIVE_SECS)
                time.sleep(STUCK_DRIVE_SECS)
                LOGGER.log("stuck_recovery_done")
                bw.reset()
                odom_dist_at_window_fill = None
                last_stuck_ts = time.time()
                continue

        if drive.get("pivot"):
            pybot_scout.stop_move()
            pybot_scout.set_rotate_3(2 if heading > 0 else 1, 12)
            LOGGER.log("camera_pivot",
                       heading_deg=heading,
                       camera_stats=camera_stats,
                       readings=readings)
            DASHBOARD.update_state(mode="camera_pivot", heading_deg=heading, camera_center=drive.get("center"))
            DASHBOARD.tick()

            pose = dict(ODOM.get_pose())
            waypoints.append(pose)
            LOGGER.log("step_selected", direction_deg=heading, pose=pose, camera_active=camera_active)
            time.sleep(PAUSE_SECS)
            continue

        pybot_scout.set_translationSpeed(drive.get("speed", SPEED))
        pybot_scout.set_translate_2(heading % 360, CHECK_INTERVAL_SECS)
        burst_count += 1
        LOGGER.log("move_burst_camera",
                   direction_deg=heading,
                   burst_secs=CHECK_INTERVAL_SECS,
                   camera_active=camera_active,
                   camera_stats=camera_stats,
                   readings=readings)
        DASHBOARD.update_state(mode="move_burst_camera", heading_deg=heading, camera_center=drive.get("center"))
        DASHBOARD.tick()

        # ── update brightness window after each drive burst ────────────────────
        if camera_stats:
            lc = camera_stats.get("lower_center_mean_brightness") or 0.0
            ll = camera_stats.get("lower_left_mean_brightness") or 0.0
            lr = camera_stats.get("lower_right_mean_brightness") or 0.0
            was_full = bw.full()
            bw.push(lc, ll, lr)
            if not was_full and bw.full():
                odom_dist_at_window_fill = ODOM.get_stats()["total_distance_m"]

        # Record a waypoint every WAYPOINT_STRIDE bursts
        if burst_count % WAYPOINT_STRIDE == 0:
            waypoints.append(dict(ODOM.get_pose()))

    LOGGER.log("explore_finished",
               waypoints_recorded=len(waypoints),
               battery_pct=BATTERY.get_percent())
    DASHBOARD.update_state(mode="explore_finished", waypoints_recorded=len(waypoints), battery_pct=BATTERY.get_percent())
    DASHBOARD.tick()
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

    heading   = 0.0
    deadline  = time.time() + timeout_secs

    pybot_scout.set_translationSpeed(SPEED)
    pybot_scout.set_rotationSpeed(ROTATION_SPEED)

    while time.time() < deadline and not _shutdown:
        if PILE.was_recently_seen():
            LOGGER.log("visual_scan_found_pile")
            return True

        readings = pybot_scout.get_proximity_readings()
        camera_stats = CAMERA.get_snapshot()
        drive = _camera_drive_command(camera_stats)  # no delta history in visual scan
        if drive is None:
            LOGGER.log("visual_scan_camera_missing", camera_stats=camera_stats, readings=readings)
            time.sleep(0.2)
            continue
        heading = drive.get("heading_deg", 0.0)

        if drive.get("pivot"):
            pybot_scout.stop_move()
            pybot_scout.set_rotate_3(2 if heading > 0 else 1, 12)
            LOGGER.log("visual_scan_camera_pivot", heading_deg=heading, camera_stats=camera_stats, readings=readings)
            time.sleep(PAUSE_SECS)
            continue

        pybot_scout.set_translationSpeed(drive.get("speed", SPEED))
        pybot_scout.set_translate_2(heading % 360, CHECK_INTERVAL_SECS)

    LOGGER.log("visual_scan_timeout")
    return False


def _return_home(waypoints):
    """Orchestrate the full return-to-charger sequence."""
    LOGGER.log("return_home_started", battery_pct=BATTERY.get_percent())
    DASHBOARD.update_state(mode="return_home_started", battery_pct=BATTERY.get_percent(), waypoints=len(waypoints))
    DASHBOARD.tick()
    print("Starting return-home sequence.")
    pybot_scout.stop_move()

    # ── step 1: try built-in /navBackup ──────────────────────────────────────
    triggered = _trigger_navbackup()
    if triggered:
        print("Triggered built-in go-home via /navBackup.  Waiting…")
        done = _wait_for_going_home_done(NAVBACKUP_WAIT_SECS)
        if done:
            LOGGER.log("return_home_via_navbackup_succeeded")
            DASHBOARD.update_state(mode="return_home_navbackup_ok")
            DASHBOARD.tick()
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
    DASHBOARD.update_state(
        mode="return_home_finished",
        charging=BATTERY.is_charging(),
        battery_pct=BATTERY.get_percent(),
    )
    DASHBOARD.tick()
    print("Return-home sequence complete.")


# ── main cycle ─────────────────────────────────────────────────────────────────

def run_patrol():
    # ── setup ─────────────────────────────────────────────────────────────────
    ODOM.start()
    PILE.start()
    CHARGER_IO.start()
    BATTERY.start(logger=LOGGER)
    CAMERA.start(logger=LOGGER)
    DASHBOARD.start()
    inventory = log_ros_inventory(LOGGER)
    DASHBOARD.update_ros_topics(inventory.get("topics", []))

    pybot_scout.set_rotationSpeed(ROTATION_SPEED)
    pybot_scout.set_translationSpeed(SPEED)

    subscribed = _subscribe_topics(set())
    LOGGER.log(
        "patrol_started",
        depart_pct=DEPART_PCT,
        return_pct=RETURN_PCT,
        battery_check_secs=BATTERY_CHECK_SECS,
        max_steer_deg=MAX_STEER_DEG,
        steer_gain=STEER_GAIN,
        dark_slow_brightness=DARK_SLOW_BRIGHTNESS,
        dark_pivot_brightness=DARK_PIVOT_BRIGHTNESS,
        crawl_speed=CRAWL_SPEED,
        allow_sensorless=ALLOW_SENSORLESS,
        proximity_topics=sorted(subscribed),
    )

    print("Patrol started.  Depart at %.0f %% / Return at %.0f %%.  "
          "Press Ctrl-C to stop." % (DEPART_PCT, RETURN_PCT))
    DASHBOARD.update_state(
        mode="patrol_started",
        depart_pct=DEPART_PCT,
        return_pct=RETURN_PCT,
        subscribed_topics=len(subscribed),
    )
    DASHBOARD.tick(force=True)

    print("Waiting up to %.0f s for camera brightness data…" % CAMERA_TIMEOUT_SECS)
    camera_active = _wait_for_camera_data(CAMERA_TIMEOUT_SECS)
    LOGGER.log("camera_wait_completed",
               camera_active=camera_active,
               camera_snapshot=CAMERA.get_snapshot(),
               readings=pybot_scout.get_proximity_readings())
    DASHBOARD.update_sensors(pybot_scout.get_proximity_readings())
    DASHBOARD.update_state(mode="camera_wait_completed", camera_active=camera_active)
    DASHBOARD.tick(force=True)

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
        DASHBOARD.update_state(
            mode="patrol_cycle_started",
            cycle=cycle,
            battery_pct=BATTERY.get_percent(),
            charging=BATTERY.is_charging(),
            on_charger=on_charger,
        )
        DASHBOARD.tick()
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
        waypoints = _explore(subscribed, camera_active)

        if _shutdown:
            break

        # ── return home ───────────────────────────────────────────────────────
        _return_home(waypoints)

        on_charger = True   # assume we made it back

        LOGGER.log("patrol_cycle_finished", cycle=cycle,
                   battery_pct=BATTERY.get_percent())
        DASHBOARD.update_state(
            mode="patrol_cycle_finished",
            cycle=cycle,
            battery_pct=BATTERY.get_percent(),
            charging=BATTERY.is_charging(),
        )
        DASHBOARD.tick()
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
    DASHBOARD.close()
    ODOM.stop()
    PILE.stop()
    CHARGER_IO.stop()
    BATTERY.stop()
    CAMERA.stop()
    LOGGER.log("run_stopped")
    LOGGER.close()
    pybot_scout.stop()
