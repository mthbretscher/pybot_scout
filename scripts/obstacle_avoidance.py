# -*- coding: utf-8 -*-
"""
Camera-brightness continuous analog explorer.

Implements an analog-circuit-style control loop: the robot's speed and
heading are internal state variables that are nudged each tick toward
target values derived from camera brightness — like a capacitor being
charged or discharged through a resistor.

  v[n]       = v[n-1]  + alpha * (v_target  - v[n-1])
  heading[n] = h[n-1]  + beta  * (h_target  - h[n-1])

where alpha/beta are per-tick gains (ACCEL_ALPHA, BRAKE_ALPHA,
STEER_ALPHA).  There are no discrete move-bursts; velocity is streamed
continuously via set_translate_4, which re-publishes at ~10 Hz via the
async sender thread.

Sensor inputs:
  - Primary: lower-center camera brightness (dark = caution, bright = clear)
  - Secondary: left/right brightness delta for lateral steering
  - Tertiary: brightness-over-time delta (approaching obstacle)

Proximity topics are subscribed and logged for diagnostics only.

At startup the script detects whether the robot is sitting on its charging
station (via /SensorNode/simple_battery_status).  If so it drives straight
forward for CHARGER_EXIT_SECS to clear the dock before beginning exploration.

Usage:
    python scripts/obstacle_avoidance.py

Environment:
    PYBOT_SCOUT_ALLOW_SENSORLESS_FALLBACK  – set to "1" to allow movement
                                              without camera data
    PYBOT_SCOUT_PROXIMITY_TOPICS           – override discovered topics
"""

import os
import signal
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT   = os.path.dirname(SCRIPT_DIR)
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from pybot_scout.charging_pile import ChargingPileDetector, ChargingStatusDetector
from pybot_scout.camera import BrightnessWindow, GreyImageMonitor
from pybot_scout.feedback import FeedbackLogger
from pybot_scout.odometry import OdometryTracker
from pybot_scout.proximity import discover_proximity_topics
from pybot_scout.dashboard import ScriptDashboard
from pybot_scout.ros_inventory import log_ros_inventory
from pybot_scout.scout import pybot_scout

# ── tunable constants ──────────────────────────────────────────────────────────
TICK_SECS            = 0.30    # control-loop interval (s)
PAUSE_SECS           = 0.25    # pause after in-place pivot before resuming
CAMERA_TIMEOUT_SECS  = 5.0     # wait this long for the first camera brightness sample
CHARGER_EXIT_SECS    = 3.0     # drive straight for this long to clear the dock
REDISCOVERY_SECS     = 15.0    # re-scan for proximity topics this often

MAX_SPEED            = 0.20    # m/s absolute speed ceiling
MIN_SPEED            = 0.04    # m/s – below this snap to zero (prevent creeping)
CRAWL_SPEED          = 0.10    # m/s used for charger-exit ramp; not used in main loop
ROTATION_SPEED       = 90      # deg/s for pivot / stuck recovery
MAX_STEER_DEG        = 45.0    # clamp heading command to ±this range
STEER_GAIN           = 28.0    # heading gain from lateral brightness normalised ratio

# Camera brightness thresholds (centre lower-panel brightness 0–255)
DARK_STOP_BRIGHTNESS  = 35.0   # at or below this → v_target = 0  (hard brake)
DARK_SLOW_BRIGHTNESS  = 60.0   # transition zone boundary (crawl→full speed)
DARK_PIVOT_BRIGHTNESS = 40.0   # centre brightness that enables in-place pivot
PIVOT_HEADING_DEG     = 15.0   # |heading_target| that triggers pivot when stopped

# ── analog controller gains (first-order difference equations per tick) ────────
# Think of these as RC time constants:  alpha = dt / (dt + tau)
# At TICK_SECS=0.30:  ACCEL_ALPHA=0.20 → tau≈1.2 s (slow charge)
#                     BRAKE_ALPHA=0.65 → tau≈0.16 s (fast discharge)
#                     STEER_ALPHA=0.40 → tau≈0.45 s (moderate steering slew)
ACCEL_ALPHA  = 0.20    # speed climb rate toward higher target  (gradual ramp-up)
BRAKE_ALPHA  = 0.65    # speed drop rate toward lower target    (fast braking)
STEER_ALPHA  = 0.40    # heading slew rate per tick

# Delta-based obstacle approach steering
DELTA_APPROACH_THRESHOLD = 0.025  # delta_fwd/base ratio that triggers approach boost
DELTA_STEER_BOOST        = 3.0    # multiplier applied to lateral_norm when approaching
DELTA_WINDOW_SPAN        = 5      # number of readings over which to compute deltas

# Stuck detection and recovery
STUCK_WINDOW_SIZE       = 20    # brightness readings in the stuck-detection window (~6 s)
STUCK_BRIGHTNESS_SPREAD = 5.0   # max(fwd) - min(fwd) below this → scene not changing
STUCK_DIST_THRESHOLD_M  = 0.20  # odometry distance that confirms NOT stuck (moving freely)
STUCK_COOLDOWN_SECS     = 15.0  # minimum seconds between successive stuck-recoveries
STUCK_ROTATE_DEG        = 90    # degrees to rotate during recovery
STUCK_DRIVE_SECS        = 2.0   # seconds to drive straight after recovery rotation

ALLOW_SENSORLESS = os.environ.get("PYBOT_SCOUT_ALLOW_SENSORLESS_FALLBACK", "0") == "1"

FEEDBACK_DIR   = os.path.join(REPO_ROOT, "run_feedback")
LOGGER         = FeedbackLogger("obstacle_avoidance", output_dir=FEEDBACK_DIR)
ODOM_TRACKER   = OdometryTracker()
PILE_DETECTOR  = ChargingPileDetector()
CHARGER_STATUS = ChargingStatusDetector()
DASHBOARD      = ScriptDashboard("obstacle_avoidance", logger=LOGGER)
CAMERA         = GreyImageMonitor()


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


def _wait_for_camera_data(timeout_secs):
    """Return True once the camera monitor has brightness data."""
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
    """Return camera-based drive targets dict or None when insufficient data.

    Returns v_target (desired forward speed, m/s) and heading_target (desired
    heading in degrees, 0=straight, positive=right) computed from camera
    brightness.  These are *targets* for the analog state variables, not
    instantaneous commands — the caller applies difference-equation smoothing.

    delta_* are brightness changes over recent readings (positive = brighter =
    approaching an obstacle on that face).  When the forward view is brightening,
    the gain is boosted toward the darker (more-open) side.
    """
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
    heading_target = _clamp(steer, -MAX_STEER_DEG, MAX_STEER_DEG)

    # Continuous speed target from centre brightness:
    #   centre ≤ DARK_STOP_BRIGHTNESS  → v_target = 0  (hard brake zone)
    #   DARK_STOP < centre ≤ DARK_SLOW → linear ramp from 0 to CRAWL_SPEED
    #   centre > DARK_SLOW             → linear ramp from CRAWL_SPEED to MAX_SPEED
    if center <= DARK_STOP_BRIGHTNESS:
        v_target = 0.0
    elif center <= DARK_SLOW_BRIGHTNESS:
        frac = (center - DARK_STOP_BRIGHTNESS) / (DARK_SLOW_BRIGHTNESS - DARK_STOP_BRIGHTNESS)
        v_target = frac * CRAWL_SPEED
    else:
        frac = _clamp((center - DARK_SLOW_BRIGHTNESS) / (base - DARK_SLOW_BRIGHTNESS + 1.0), 0.0, 1.0)
        v_target = CRAWL_SPEED + frac * (MAX_SPEED - CRAWL_SPEED)

    return {
        "left": left,
        "center": center,
        "right": right,
        "lateral_norm": lateral_norm,
        "center_conf": _clamp(center / base, 0.0, 1.0),
        "heading_target": heading_target,
        "v_target": v_target,
        "delta_fwd": round(delta_fwd, 3),
        "delta_left": round(delta_left, 3),
        "delta_right": round(delta_right, 3),
    }



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
    CAMERA.start(logger=LOGGER)

    pybot_scout.set_rotationSpeed(ROTATION_SPEED)
    pybot_scout.set_translationSpeed(CRAWL_SPEED)
    DASHBOARD.start()
    inventory = log_ros_inventory(LOGGER)
    DASHBOARD.update_ros_topics(inventory.get("topics", []))

    subscribed = _subscribe_topics(set())
    LOGGER.log(
        "run_started",
        max_speed=MAX_SPEED,
        min_speed=MIN_SPEED,
        crawl_speed=CRAWL_SPEED,
        rotation_speed=ROTATION_SPEED,
        max_steer_deg=MAX_STEER_DEG,
        steer_gain=STEER_GAIN,
        dark_stop_brightness=DARK_STOP_BRIGHTNESS,
        dark_slow_brightness=DARK_SLOW_BRIGHTNESS,
        dark_pivot_brightness=DARK_PIVOT_BRIGHTNESS,
        accel_alpha=ACCEL_ALPHA,
        brake_alpha=BRAKE_ALPHA,
        steer_alpha=STEER_ALPHA,
        tick_secs=TICK_SECS,
        camera_timeout_secs=CAMERA_TIMEOUT_SECS,
        charger_exit_secs=CHARGER_EXIT_SECS,
        allow_sensorless=ALLOW_SENSORLESS,
        proximity_topics=sorted(subscribed),
    )

    print("Waiting up to %.0f s for camera brightness data…" % CAMERA_TIMEOUT_SECS)
    camera_active = _wait_for_camera_data(CAMERA_TIMEOUT_SECS)
    LOGGER.log(
        "camera_wait_completed",
        camera_active=camera_active,
        camera_snapshot=CAMERA.get_snapshot(),
        readings=pybot_scout.get_proximity_readings(),
    )

    if camera_active:
        print("Camera brightness data received – camera steering ENABLED.")
    else:
        print("WARNING: No camera brightness data after %.0f s." % CAMERA_TIMEOUT_SECS)
        if not ALLOW_SENSORLESS:
            print("         Set PYBOT_SCOUT_ALLOW_SENSORLESS_FALLBACK=1 to run with fallback steering.")
            LOGGER.log("camera_fallback_disabled")
    DASHBOARD.update_state(
        heading_deg=0,
        mode="camera_wait_done",
        camera_active=camera_active,
        allow_sensorless=ALLOW_SENSORLESS,
    )
    DASHBOARD.update_sensors(pybot_scout.get_proximity_readings())
    DASHBOARD.tick(force=True)

    # ── leave the dock first ───────────────────────────────────────────────────
    _exit_charger_if_needed()

    # ── analog controller state ───────────────────────────────────────────────
    # v_cmd and heading_cmd are the capacitor-like state variables.  Each tick
    # they charge/discharge toward their targets via difference equations.
    v_cmd       = 0.0   # current commanded speed, m/s
    heading_cmd = 0.0   # current commanded heading, degrees (0=straight forward)
    print("Analog explorer started.  Press Ctrl-C to stop.")

    next_rediscovery = time.time() + REDISCOVERY_SECS
    bw = BrightnessWindow(STUCK_WINDOW_SIZE)
    last_stuck_ts = 0.0
    odom_dist_at_window_fill = None

    while True:
        # ── periodic topic rediscovery (helps if sensors come online late) ────
        if time.time() >= next_rediscovery:
            subscribed = _subscribe_topics(subscribed)
            next_rediscovery = time.time() + REDISCOVERY_SECS

        if not camera_active:
            camera_active = _wait_for_camera_data(0.2)

        if not camera_active and not ALLOW_SENSORLESS:
            LOGGER.log("waiting_for_camera_data")
            DASHBOARD.update_state(
                mode="waiting_for_camera_data",
                heading_deg=heading_cmd,
                camera_active=camera_active,
                subscribed_topics=len(subscribed),
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
            mode="drive_loop",
            heading_deg=heading_cmd,
            camera_active=camera_active,
            subscribed_topics=len(subscribed),
            camera_left=(drive or {}).get("left"),
            camera_center=(drive or {}).get("center"),
            camera_right=(drive or {}).get("right"),
        )
        DASHBOARD.tick()

        if drive is None:
            if not ALLOW_SENSORLESS:
                LOGGER.log("camera_drive_missing", camera_stats=camera_stats)
                time.sleep(0.3)
                continue
            drive = {
                "heading_target": 0.0,
                "v_target": MIN_SPEED,
                "left": None,
                "center": None,
                "right": None,
                "lateral_norm": 0.0,
                "center_conf": 0.0,
                "delta_fwd": 0.0,
                "delta_left": 0.0,
                "delta_right": 0.0,
            }

        v_target = drive.get("v_target", 0.0)
        h_target = drive.get("heading_target", 0.0)

        # ── stuck detection ────────────────────────────────────────────────────
        if bw.is_stuck() and time.time() - last_stuck_ts > STUCK_COOLDOWN_SECS:
            odom_stats = ODOM_TRACKER.get_stats()
            odom_pose = ODOM_TRACKER.get_pose()
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
                pybot_scout.set_translationSpeed(CRAWL_SPEED)
                pybot_scout.set_translate_2(0, STUCK_DRIVE_SECS)
                time.sleep(STUCK_DRIVE_SECS)
                LOGGER.log("stuck_recovery_done")
                bw.reset()
                odom_dist_at_window_fill = None
                last_stuck_ts = time.time()
                # Reset analog state so we accelerate smoothly after recovery
                v_cmd = 0.0
                heading_cmd = 0.0
                continue

        # ── analog difference-equation state update ───────────────────────────
        # Speed: use faster discharge (BRAKE_ALPHA) when target is below current
        # so the robot reacts quickly to obstacles, and slower charge (ACCEL_ALPHA)
        # for a gradual ramp-up on clear paths.
        alpha = BRAKE_ALPHA if v_target < v_cmd else ACCEL_ALPHA
        v_cmd = v_cmd + alpha * (v_target - v_cmd)
        if v_cmd < MIN_SPEED:
            v_cmd = 0.0

        # Heading: first-order low-pass, clamped to physical steering range.
        heading_cmd = heading_cmd + STEER_ALPHA * (h_target - heading_cmd)
        heading_cmd = _clamp(heading_cmd, -MAX_STEER_DEG, MAX_STEER_DEG)

        # ── issue continuous velocity command ─────────────────────────────────
        # set_translate_4 feeds the async-sender thread which re-publishes the
        # Twist at ~10 Hz, providing continuous smooth motion between ticks.
        if v_cmd > 0.0:
            pybot_scout.set_translate_4(heading_cmd % 360, v_cmd)
        else:
            pybot_scout.stop_move()
            # In-place pivot toward the clearer side when fully stopped and
            # the lateral error is large enough to be meaningful.
            if (drive.get("center") is not None
                    and drive["center"] < DARK_PIVOT_BRIGHTNESS
                    and abs(h_target) > PIVOT_HEADING_DEG):
                rot_dir = 2 if h_target > 0 else 1
                pybot_scout.set_rotate_3(rot_dir, 12)
                LOGGER.log("camera_pivot",
                           heading_cmd=round(heading_cmd, 2),
                           h_target=round(h_target, 2),
                           camera_stats=camera_stats,
                           readings=readings)
                DASHBOARD.update_state(mode="camera_pivot", heading_deg=heading_cmd)
                DASHBOARD.tick()
                time.sleep(PAUSE_SECS)

        LOGGER.log(
            "drive_tick",
            v_cmd=round(v_cmd, 4),
            v_target=round(v_target, 4),
            heading_cmd=round(heading_cmd, 2),
            heading_target=round(h_target, 2),
            camera_active=camera_active,
            camera_stats=camera_stats,
            readings=readings,
        )
        DASHBOARD.update_state(
            mode="continuous_drive",
            heading_deg=heading_cmd,
            camera_center=drive.get("center"),
            v_cmd=v_cmd,
            v_target=v_target,
        )
        DASHBOARD.tick()

        # ── pace the control loop ─────────────────────────────────────────────
        time.sleep(TICK_SECS)

        # ── update brightness window after each tick ───────────────────────────
        if camera_stats:
            lc = camera_stats.get("lower_center_mean_brightness") or 0.0
            ll = camera_stats.get("lower_left_mean_brightness") or 0.0
            lr = camera_stats.get("lower_right_mean_brightness") or 0.0
            was_full = bw.full()
            bw.push(lc, ll, lr)
            if not was_full and bw.full():
                odom_dist_at_window_fill = ODOM_TRACKER.get_stats()["total_distance_m"]


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
    CAMERA.stop()
    DASHBOARD.close()
    ODOM_TRACKER.stop()
    PILE_DETECTOR.stop()
    CHARGER_STATUS.stop()
    LOGGER.log("run_stopped")
    LOGGER.close()
    pybot_scout.stop()
