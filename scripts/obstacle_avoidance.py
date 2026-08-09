# -*- coding: utf-8 -*-
"""
ToF-only straight-line explorer (v7, 2026-07-26 -- simplified rewrite).

Design: keep driving straight ahead in whatever direction the robot is
currently facing. Only change direction when something is actually in the
way. No camera/brightness-based steering or stuck-detection -- see
"Why the camera-based version was replaced" below.

  - Drive forward continuously at DRIVE_SPEED in the current heading
    (degree=0 relative to the robot's own body -- direction only changes via
    the explicit rotate-and-search below, never by continuous steering).
  - Each tick, read a robustified (median-of-3, debounced) /SensorNode/tof
    forward distance.
  - Periodically (every WAG_INTERVAL_SECS, but only when the forward ToF
    already reads within WAG_TRIGGER_DISTANCE_M -- "if in doubt, keep
    driving" rather than interrupting a clear run for no reason) look
    briefly left then right of straight-ahead (_wag_scan_left_right) to get
    a little situational awareness beyond the single forward point reading.
    This curves continuously (set_translate_rotate, same non-blocking
    cmd_vel-streaming path as the bounce below) rather than stopping to
    pivot -- the robot keeps translating forward the entire time it glances
    each way. If one side reads meaningfully closer than the other (an
    angled approach to a wall), bounce away from it (_maybe_bounce): keep
    driving forward while slowly rotating away from the near side for a
    couple of seconds using set_translate_rotate, like a ball bouncing off
    a wall at an angle, then let go -- since forward is always translate
    degree 0 relative to the body, once the rotation stops the robot is
    already driving in its new heading with the ToF sensor looking the
    same way, no separate re-alignment step needed. This is a
    soft/anticipatory layer that tries to avoid ever reaching the hard stop
    below; it does not replace it. After any maneuver that already commits
    to a fresh heading (a bounce, a critical-stop room search, or a blind-
    mode redirect), the next wag check is pushed out by
    WAG_POST_MANEUVER_GRACE_SECS -- there's usually a lot more open room
    ahead right after picking a new direction, so keep driving it rather
    than immediately re-checking sideways again.
  - If that distance drops below TOF_STOP_DISTANCE_M ("bumped into
    something"), OR the raw reading simply hasn't changed at all for
    STUCK_TOF_TIMEOUT_SECS (see _TofStuckDetector -- catches both a stale
    frozen close reading AND a long run of genuinely-invalid NaN/-inf/
    negative readings, either of which live testing showed can happen while
    genuinely stuck; a steady +inf reading means confirmed-clear and is
    deliberately NOT treated as stuck -- see STUCK_RELEVANT_DISTANCE_M):
    stop,
    back away slightly, then rotate through a full-circle sweep, nudging
    forward briefly at each heading before sampling ToF (_tof_search_for_room
    -- the nudge both unsticks a latched sensor reading and physically tests
    whether that heading is passable), and commit to whichever heading had
    the most room. Resume driving straight ahead from there.

Why the camera-based version was replaced (see scripts/previous_scripts/
obstacle_avoidance_camera_v6.py for the old approach, kept for reference):
live testing on 2026-07-26 showed the camera-brightness stuck-detection
false-triggered constantly (18 stuck-recoveries vs. only 2 real ToF critical
stops in one ~6 minute run -- see run_feedback/20260726_163236_*.jsonl) even
while the user visually confirmed the ToF reading showed real open space
ahead. The robot spent most of its time standing, rotating a bit, creeping
forward, and standing again. Meanwhile a live isolated test (motors off vs.
motors actively spinning, robot immobilized) showed ToF readings stay sane
(no inf/nan, same noise floor ~+/-0.02m) whether or not the motors are
running, and the original run's own tof_critical_stop/_recovered events
already showed plausible distances during real driving and during the
rotate-sweep. So ToF is the reliable signal here; camera brightness was not.

Odometry: NOT used as a reward/cross-check for anything in this script.
Verified live 2026-07-26 that neither wheel nor VIO odometry accumulates
distance during continuous-Twist driving (the only way this script drives)
-- see Agents/instructions.md's Odometry row. ODOM_TRACKER is still started
purely for best-effort diagnostic logging at shutdown, nothing more.

Adaptive tuning: intentionally NOT included in this rewrite (v5/v6 had a
self-tuning tof_slow_distance_m/max_speed hill-climber -- see the archived
v6 script for that design, preserved in case it's worth reviving once this
simpler approach is validated). Keeping this version to as few moving parts
as possible while the core straight-line-until-bumped design gets proven out
live.

At startup the script detects whether the robot is sitting on its charging
station (via /SensorNode/simple_battery_status). If so it drives straight
forward for CHARGER_EXIT_SECS to clear the dock before beginning exploration.

Battery watchdog / auto-dock (unchanged from the previous version, not
camera-dependent): while exploring, if battery % drops to
PYBOT_SCOUT_RETURN_BATTERY_PCT (default 50) *and* the charging pile has been
visually seen recently, the robot stops and calls the vendor's /nav_low_bat
service to try to dock. Once actually charging, exploration pauses until
battery reaches PYBOT_SCOUT_UNDOCK_BATTERY_PCT (default 99), then it drives
forward to unlodge itself and resumes exploring.

Usage:
    python scripts/obstacle_avoidance.py

Environment:
    PYBOT_SCOUT_RETURN_BATTERY_PCT  – battery % that triggers a dock attempt
                                       when the pile is visible (default: 50)
    PYBOT_SCOUT_UNDOCK_BATTERY_PCT  – battery % at which to unlodge and
                                       resume exploring while charging
                                       (default: 99)
"""

import atexit
import os
import random
import select
import signal
import subprocess
import sys
import threading
import termios
import time
import tty

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT   = os.path.dirname(SCRIPT_DIR)
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import rospy

from pybot_scout.battery import BatteryMonitor
from pybot_scout.charging_pile import ChargingPileDetector, ChargingStatusDetector
from pybot_scout.feedback import FeedbackLogger
from pybot_scout.odometry import OdometryTracker
from pybot_scout.proximity import discover_proximity_topics
from pybot_scout.dashboard import ScriptDashboard
from pybot_scout.ros_inventory import log_ros_inventory
from pybot_scout.ros_bridge import nav_low_bat as _nav_low_bat_srv
from pybot_scout.ros_bridge import status as _RollerEyeStatus
from pybot_scout.scout import pybot_scout

# ── tunable constants ──────────────────────────────────────────────────────────
TICK_SECS            = 0.20    # control-loop interval (s)
PAUSE_SECS           = 0.25    # pause after a room-search before resuming
CHARGER_EXIT_SECS    = 3.0     # drive straight for this long to clear the dock
REDISCOVERY_SECS     = 15.0    # re-scan for proximity topics this often

DRIVE_SPEED          = 0.16    # m/s while driving straight ahead
CRAWL_SPEED          = 0.10    # m/s used for the brief backup after a critical stop
ROTATION_SPEED       = 90      # deg/s for the room-search rotation

# ToF safety layer -- the only sensor this script reacts to. TOF_TOPIC_HINT
# matches against proximity-reading topic names so unrelated range topics
# (e.g. the ibeacon charging-pile range) are never mistaken for a
# forward-obstacle distance.
TOF_TOPIC_HINT           = "tof"
TOF_STOP_DISTANCE_M      = 0.07    # "bumped into something" -- stop + search below this
TOF_BACKUP_SECS          = 0.8     # seconds to reverse before the room search
TOF_SEARCH_STEPS         = 6       # headings sampled in the room search
TOF_SEARCH_STEP_DEG      = 60      # degrees rotated per search step (steps*step_deg = 360)
TOF_SEARCH_SETTLE_SECS   = 0.35    # settle time after each rotation step before sampling
TOF_SEARCH_SAMPLES       = 3       # ToF samples taken (median) at each heading
TOF_SEARCH_NUDGE_SECS    = 0.4     # forward nudge at each search heading -- see _tof_search_for_room
# NOTE: TOF_STOP_DISTANCE_M was 0.05m in the first live test -- raised to
# 0.10m (2026-07-26) after that test showed the robot getting uncomfortably
# close to obstacles before the safety stop fired (0.05m left only ~2cm of
# margin above the sensor's own ~0.03m min_range, too little stopping
# distance at DRIVE_SPEED). Lowered back to 0.07m (2026-07-26, explicit user
# request for more aggressive driving -- "nothing terrible happens if it
# bumps into a wall") now that DRIVE_SPEED is also higher; splits the
# difference between the two prior values rather than returning to the
# too-close 0.05m extreme.

# ToF "stuck" detection: live testing 2026-07-26 showed two related failure
# modes that a simple distance-threshold check misses entirely:
#   1. ToF sometimes reads NaN/-inf/negative (genuinely invalid, see
#      _tof_distance) for a long stretch while the robot is pressed against
#      something. _tof_distance() correctly filters these out as "not
#      currently measurable", but the debounced TOF_HISTORY then just keeps
#      returning its last-known-good value forever, so the distance-
#      threshold check never fires. IMPORTANT correction (2026-07-26,
#      confirmed empirically by the user): a +inf reading is NOT one of
#      these invalid cases -- it means "confirmed clear, no obstacle in
#      range" and is treated as a real, valid, far-away distance (see
#      _tof_distance) -- a steady +inf is normal/expected in open space, not
#      a failure mode, and is explicitly excluded from "stuck" via
#      STUCK_RELEVANT_DISTANCE_M below.
#   2. ToF sometimes *latches* on a stale low reading (e.g. 0.036m) even once
#      genuinely open space is back in front of the sensor -- readings only
#      become sensible again once the robot physically moves forward again.
# Both look the same from the control loop's point of view: the raw ToF
# reading (valid or not) simply isn't changing for a long time while we're
# supposedly driving forward -- that IS the stuck signal, independent of
# whatever value (if any) it's frozen at. See _TofStuckDetector below.
STUCK_TOF_EPSILON_M      = 0.03    # readings within this much of each other count as "unchanged"
STUCK_TOF_TIMEOUT_SECS   = 4.0     # how long an unchanging reading must persist to count as stuck
# Loosened from 0.02m/3.0s (2026-07-26 v9): user reported the robot getting
# flagged "stuck" -> blind-mode sometimes while it was visibly still moving
# in small increments -- a little more tolerance for ordinary sensor noise
# and brief plateaus avoids false-triggering blind mode during genuine (if
# slow) progress.
# A frozen reading is only actually treated as "stuck" (see is_stuck gating
# in the main loop) when it's this close or the reading is fully invalid --
# a steady FAR reading (e.g. driving down a long corridor, or mid-bounce/wag
# curve where the forward beam's target barely moves) is normal, not stuck;
# "if in doubt, keep driving" rather than derailing into blind-mode fallback
# over a harmless steady reading with nothing nearby.
STUCK_RELEVANT_DISTANCE_M = 0.5

# "Blind" fallback driving (2026-07-26): when the ToF sensor itself is stuck
# (frozen/unmeasurable per _TofStuckDetector above), the normal critical-stop
# + full-circle room-search maneuver can't be trusted either -- it depends on
# reading the very sensor that's currently broken. Rather than stopping and
# waiting (or searching blind), just keep exploring: drive straight ahead,
# do a brief bump-and-turn every BLIND_MODE_REDIRECT_INTERVAL_SECS so it
# doesn't keep pushing into the same obstacle, and fall back to normal
# ToF-based navigation the moment the sensor reports a change again. Bumping
# into things occasionally is an acceptable tradeoff -- the goal is to keep
# covering ground rather than sit still waiting for a sensor that (per the
# systems-level ToF freeze investigation) might not come back for minutes.
BLIND_MODE_REDIRECT_INTERVAL_SECS = 6.0    # how often to change direction while blind
BLIND_MODE_BACKUP_SECS            = 0.6    # brief backup before turning (in case pressed against something)
BLIND_MODE_MIN_TURN_DEG           = 60      # randomized turn range so it doesn't retrace the same path
BLIND_MODE_MAX_TURN_DEG           = 150

# Angled left-right "wag" scan (2026-07-26): the forward ToF only ever tells
# us what's directly ahead, so an approach to a wall at an angle isn't seen
# until it's already close enough to trigger a full stop-and-search. This
# periodically looks left/right of center to catch that earlier, and reacts
# by curving away from the closer side (_maybe_bounce) rather than driving
# straight into it.
# NOTE: each wag scan (rotate left -> settle -> sample -> swing right ->
# settle -> sample -> rotate back to center) takes roughly 1.5-1.7s at
# WAG_ANGLE_DEG=20/ROTATION_SPEED=90. At the original WAG_INTERVAL_SECS=3.0
# that meant over half of every cycle was spent scanning/rotating instead of
# driving straight -- the actual cause of the "short bursts of driving"
# symptom reported live (2026-07-26). Raised to 10.0s so the scan overhead
# (~1.6s) is a small fraction of each cycle and straight-line driving bursts
# are much longer, while still checking for an angled approach often enough
# to matter at DRIVE_SPEED.
# v8 update (2026-07-26): the interval fix above wasn't enough on its own --
# _wag_scan_left_right() still used _timed_rotate()/set_rotate_3(), a
# BLOCKING vendor algo_roll ROS service call that internally calls
# _stop_async_translate_rotate() first, so every wag scan was still a full
# stop-pivot-stop-pivot-stop-straighten sequence with zero forward motion
# for its ~1.5-2s duration -- this, not a CPU/threading problem, is what
# produced the "moves so abruptly" symptom (get_proximity_readings() is
# already a cheap non-blocking cache read fed by a ROS subscriber callback,
# so there was never anything to multithread). Fixed by switching the wag
# scan to the same continuous/non-blocking cmd_vel-streaming path
# _maybe_bounce() already used (set_translationSpeed/set_rotationSpeed +
# set_translate_rotate, backed by a background publisher thread), at a
# gentler WAG_ROTATION_SPEED so the curve is a glance, not a sharp turn --
# the robot keeps driving forward the entire time it looks each way. Also
# added WAG_TRIGGER_DISTANCE_M ("if in doubt, keep driving": skip the wag
# scan entirely when the forward ToF already reads clear/unknown, since
# there's usually a lot more open room ahead than this check needs to
# interrupt for) and WAG_POST_MANEUVER_GRACE_SECS (push the next wag check
# further out right after a bounce/critical-stop-search/blind-mode redirect
# already committed to a fresh heading with room).
WAG_ANGLE_DEG            = 20      # degrees left/right of center sampled by the wag scan
WAG_ROTATION_SPEED       = 30      # deg/s for the gentle continuous wag curve (vs. ROTATION_SPEED=90 used for the full-stop search sweep)
WAG_INTERVAL_SECS        = 10.0    # how often to consider a wag scan while driving straight
WAG_TRIGGER_DISTANCE_M   = 0.6     # only actually run the wag scan when the forward ToF is already this close or nearer -- otherwise just keep driving
WAG_POST_MANEUVER_GRACE_SECS = 20.0  # after a bounce/critical-stop-search/blind-mode redirect, wait this long before the next wag check
WAG_SETTLE_SECS          = 0.25    # settle time after each wag rotation before sampling
WAG_SAMPLES              = 2       # ToF samples taken (median) at each wag heading
BOUNCE_TRIGGER_DIFF_M    = 0.15    # min left-vs-right difference to call the wall "angled"
BOUNCE_TRIGGER_MAX_M     = 0.4     # only bounce if the closer side is at least this near --
                                    # lowered from 0.6m (2026-07-26, more aggressive driving
                                    # request) so moderate distances count as "open" and don't
                                    # trigger a cautious bounce
BOUNCE_ANGLE_DEG         = 35      # degrees to curve away from the near side during a bounce
BOUNCE_ROTATE_DEG_PER_SEC = 20     # slow turn rate used while driving through the bounce arc

# Battery watchdog: stop exploring and let the vendor firmware auto-dock (via
# the nav_low_bat service) once battery drops to or below this percentage.
RETURN_BATTERY_PCT          = float(os.environ.get("PYBOT_SCOUT_RETURN_BATTERY_PCT", "50"))
BATTERY_CHECK_INTERVAL_SECS = 5.0
AUTO_DOCK_TIMEOUT_SECS      = 180.0

# Once docked/charging, wait until battery reaches this level, then drive
# straight off the dock and resume exploring. Only attempted when the
# charging pile has actually been seen recently (see PILE_VISIBLE_WINDOW_SECS).
UNDOCK_BATTERY_PCT       = float(os.environ.get("PYBOT_SCOUT_UNDOCK_BATTERY_PCT", "99"))
PILE_VISIBLE_WINDOW_SECS = 2.0

FEEDBACK_DIR   = os.path.join(REPO_ROOT, "run_feedback")
LOGGER         = FeedbackLogger("obstacle_avoidance", output_dir=FEEDBACK_DIR)
ODOM_TRACKER   = OdometryTracker()
PILE_DETECTOR  = ChargingPileDetector()
CHARGER_STATUS = ChargingStatusDetector()
BATTERY        = BatteryMonitor()
DASHBOARD      = ScriptDashboard("obstacle_avoidance", logger=LOGGER)

_battery_triggered_return = {"done": False}
# When True, the main loop stops exploring and just waits for the battery to
# reach UNDOCK_BATTERY_PCT before unlodging and resuming.
_charge_wait = {"active": False}

# When True, the main loop is in "blind" fallback driving because the ToF
# sensor is stuck/frozen -- see BLIND_MODE_* constants above.
_blind_mode = {"active": False, "next_redirect_ts": None}

# ── vendor background-service pause/resume ────────────────────────────────────
# The vendor roller_eye stack (see /etc/ros/melodic/roller_eye.d/start.launch
# on the robot) runs several ROS nodes this script never touches: the camera
# pipeline and everything downstream of it (cloud upload, RTMP streaming,
# motion-triggered recording, mobile-app comms). This script is pure-ToF (see
# the module docstring) so most of these aren't needed while it drives.
# Pausing them with SIGSTOP (never SIGTERM/kill) frees CPU without ever
# actually taking the processes down -- roslaunch's respawn="true" only
# reacts to a process actually exiting, so a stopped-but-not-exited process
# is never respawned, and SIGCONT resumes it exactly where it left off.
#
# NOT paused automatically at startup (2026-07-26 v9, explicit user request)
# -- only toggled manually via the 'p' key (_toggle_paused_services), so the
# camera/cloud/app services stay up by default (e.g. so the mobile app's
# live view keeps working) unless/until the user chooses to free up the CPU.
#
# MotorNode/SensorNode are required (driving/ToF) and deliberately excluded.
# SupervisorNode is also excluded -- its exact behavior around missing/paused
# nodes isn't confirmed, so it's left alone rather than risk it.
#
# media_core_node (ROS node name "CoreNode") is ALSO deliberately excluded
# (2026-07-26 v9 bugfix) despite using 150%+ CPU on its own: it's the node
# that publishes /CoreNode/chargingPile, which is exactly what
# charging_pile.ChargingPileDetector/PILE_DETECTOR.was_recently_seen() reads
# to decide `pile_visible` in the battery watchdog. Pausing it (whether via
# the old startup auto-pause or the 'p' toggle) silently starves that
# detector forever, so `pile_visible` stays False and
# `_trigger_return_to_dock()` never gets called even when the battery really
# is low and the pile really is in view -- this was the actual root cause of
# "homing never happens until I kill the script" (killing the script lets
# atexit's _resume_paused_services() resume CoreNode, sightings start
# flowing again, and the vendor's own onboard logic reacts once our
# continuous cmd_vel stream also stops). Never add media_core_node back to
# this list without also finding another way to keep chargingPile detection
# alive.
PAUSED_SERVICE_PROCESS_NAMES = [
    "cloud_node",
    "s3_node",
    "detect_record_node",
    "recorder_agent_node",
    "rtmp_node",
    "app_node",
]

_PAUSED_PIDS = []       # [(process_name, pid), ...] currently-paused vendor processes
_SERVICES_PAUSED = {"value": False}  # current toggle state, flipped by the 'p' key
_QUIT_REQUESTED = threading.Event()  # set by _keyboard_listener on 'q'


def _safe_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _clamp(value, lo, hi):
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


class _TofHistory(object):
    """Rolling window that smooths/debounces raw ToF readings before they are
    trusted for a safety action ("robustified" -- median of the last `window`
    readings once at least `min_samples` have been collected)."""

    def __init__(self, window=3, min_samples=2):
        self._window = window
        self._min_samples = min_samples
        self._history = []

    def update(self, dist):
        if dist is not None:
            self._history.append(dist)
            if len(self._history) > self._window:
                self._history.pop(0)
        if len(self._history) < self._min_samples:
            return None
        ordered = sorted(self._history)
        mid = len(ordered) // 2
        if len(ordered) % 2 == 1:
            return ordered[mid]
        return (ordered[mid - 1] + ordered[mid]) / 2.0

    def reset(self):
        self._history = []


TOF_HISTORY = _TofHistory()


class _TofStuckDetector(object):
    """Flags "stuck" once the raw ToF reading (valid or not) hasn't changed
    by more than STUCK_TOF_EPSILON_M for STUCK_TOF_TIMEOUT_SECS. Treats a
    persistent genuinely-invalid reading (NaN/-inf/negative, see
    _tof_distance) the same as a frozen valid one -- live testing showed
    both happen while genuinely stuck (a jam can read as invalid; the
    sensor can also latch on a stale close value), and a plain distance-
    threshold check misses both. IMPORTANT (2026-07-26, confirmed
    empirically): +inf is NOT one of the invalid cases -- it means
    confirmed-clear/no-obstacle-in-range and is a real value here, not
    None. A steady +inf can still be flagged "frozen" by this class (it's
    just as unchanging as any other constant value), but the caller (see
    STUCK_RELEVANT_DISTANCE_M gating in the main loop) deliberately does
    NOT treat a frozen +inf as actually "stuck" -- an unchanging confirmed-
    clear reading is the best possible situation, not a problem.
    """

    def __init__(self, epsilon=STUCK_TOF_EPSILON_M, timeout_secs=STUCK_TOF_TIMEOUT_SECS):
        self._epsilon = epsilon
        self._timeout_secs = timeout_secs
        self._last_value = None       # None means "currently invalid/unmeasurable"
        self._last_change_ts = None

    def reset(self):
        self._last_value = None
        self._last_change_ts = None

    def update(self, raw_dist):
        """raw_dist: a float distance in metres (float('inf') means
        confirmed-clear/no-obstacle-in-range, a real usable value -- see
        _tof_distance), or None if the current reading is genuinely
        invalid/unmeasurable (NaN/-inf/negative). Returns True once the
        reading has been unchanging for longer than timeout_secs.
        """
        now = time.time()
        if self._last_change_ts is None:
            self._last_value = raw_dist
            self._last_change_ts = now
            return False
        changed = (
            (raw_dist is None) != (self._last_value is None) or
            (raw_dist is not None and self._last_value is not None and
             abs(raw_dist - self._last_value) > self._epsilon)
        )
        if changed:
            self._last_value = raw_dist
            self._last_change_ts = now
            return False
        return (now - self._last_change_ts) > self._timeout_secs


TOF_STUCK_DETECTOR = _TofStuckDetector()


def _tof_distance(readings):
    """Return the nearest ToF reading in metres, or None if unmeasurable.

    IMPORTANT (confirmed empirically by the user, 2026-07-26): a raw reading
    of +inf means "no obstacle within the sensor's range" -- the path is
    completely clear -- it is NOT an invalid/unmeasurable reading (an
    earlier version of this script, and its comments, wrongly assumed inf
    always meant invalid/stuck -- see corrected comments on
    _TofStuckDetector and the STUCK_* constants above). It's returned here
    as the real float('inf') value: every "< threshold"-style safety check
    downstream (critical-stop, wag-trigger, stuck-relevance gating) already
    treats an arbitrarily-far distance as "nothing to worry about", which is
    exactly correct, so no other code needed to change. Only NaN, -inf, and
    negative readings are genuinely invalid/unmeasurable and are filtered
    out here.
    """
    best = None
    for topic, dist in readings.items():
        if TOF_TOPIC_HINT not in topic.lower():
            continue
        d = _safe_float(dist)
        if d is None or d != d:  # None or NaN: genuinely invalid
            continue
        if d == float("-inf") or d < 0.0:  # -inf/negative: genuinely invalid
            continue
        # +inf ("confirmed clear") and any finite non-negative reading are
        # both real, usable distances -- "nearest" (smallest) naturally still
        # prefers a genuinely close finite reading over a clear one.
        if best is None or d < best:
            best = d
    return best


def _timed_rotate(direction, degree, rotation_speed=None):
    """Non-blocking in-place rotate.

    Replaces the old set_rotate_3()-based implementation (2026-07-26 v9,
    user request: audit every ROS call in this script and make sure none of
    them are blocking). set_rotate_3() calls the vendor's /UtilNode/algo_roll
    ROS service via a plain synchronous rospy.ServiceProxy() -- that call
    genuinely BLOCKS the calling thread until the vendor's own rotate-
    completion response comes back, handing control to its blocking
    algo_move/algo_roll state machine instead of our own continuous cmd_vel
    stream (this was the same root cause fixed for the wag scan earlier the
    same day -- see the WAG_* comments above). This version instead starts
    continuous rotation via the async cmd_vel-streaming path
    (set_rotationSpeed + set_rotate(), backed by MotionCmdAsyncSender --
    returns immediately, no ROS service call at all) and sleeps in Python
    for the time needed to cover `degree` at the given speed. Open-loop
    (no completion feedback from the vendor), same as the wag scan/bounce
    already are, but never blocks on a vendor service.
    """
    if rotation_speed is None:
        rotation_speed = ROTATION_SPEED
    start_ts = time.time()
    pybot_scout.set_rotationSpeed(rotation_speed)
    pybot_scout.set_rotate(direction)
    time.sleep(float(degree) / rotation_speed)
    elapsed = time.time() - start_ts
    LOGGER.log("rotate_call_timing", direction=direction, degree=degree, elapsed_secs=elapsed, blocking=False)
    return elapsed


def _timed_translate(degree, seconds):
    """Timed translate via pybot_scout.set_translate_2().

    Audited alongside _timed_rotate (2026-07-26 v9): set_translate_2() does
    NOT call a ROS service at all -- it calls PyBotScout._move()/_move_once(),
    which directly publishes cmd_vel in a foreground loop
    (self._ros_bridge.publish_raw_cmd_vel(), sleeping between publishes)
    until `seconds` elapses, the same underlying mechanism the async
    background sender uses, just run synchronously in the caller's thread
    instead of a background thread. So this already never blocks on a
    vendor ROS service -- it blocks the calling Python code for exactly
    `seconds` by design (a deliberate, short, fully-intentional timed move
    like a backup/nudge), which is fine for the sequential recovery
    maneuvers that use it.
    """
    start_ts = time.time()
    pybot_scout.set_translate_2(degree, seconds)
    elapsed = time.time() - start_ts
    LOGGER.log("translate_call_timing", degree=degree, seconds=seconds, elapsed_secs=elapsed)
    return elapsed


def _timed_translate_smooth(degree, seconds):
    """Wraps pybot_scout.set_translate_smooth() -- same reasoning as _timed_rotate;
    set_translate_smooth() also blocks internally for the full ramp+sustain+ramp
    duration before returning.
    """
    start_ts = time.time()
    pybot_scout.set_translate_smooth(degree, seconds)
    elapsed = time.time() - start_ts
    LOGGER.log("translate_smooth_call_timing", degree=degree, seconds=seconds, elapsed_secs=elapsed)
    return elapsed


def _tof_search_for_room(steps=TOF_SEARCH_STEPS, step_deg=TOF_SEARCH_STEP_DEG,
                          settle_secs=TOF_SEARCH_SETTLE_SECS, samples=TOF_SEARCH_SAMPLES,
                          nudge_secs=TOF_SEARCH_NUDGE_SECS):
    """Rotate through a full sweep (steps*step_deg = 360 deg by default). At
    each heading, nudge forward briefly before sampling a debounced
    (median-of-`samples`) ToF reading -- live testing showed the ToF sensor
    can latch on a stale reading while just rotating in place, only
    reporting sensible values again once the robot physically moves forward;
    the nudge also directly tests whether that heading is actually passable,
    not just what the sensor reports from a standstill. Then rotates to
    face whichever heading had the most room. Returns the best distance
    found in metres, or None if no ToF reading was ever obtained during the
    sweep.

    Always rotates the same direction (right) so the steps sum to a full
    circle and the robot ends up back at its start heading once done -- the
    final rotate then only needs to cover the offset to the best heading
    found, not backtrack through the whole sweep.
    """
    total_deg = steps * step_deg
    best_offset = 0
    best_dist = _tof_distance(pybot_scout.get_proximity_readings())
    for step_idx in range(1, steps + 1):
        _timed_rotate(2, step_deg)
        time.sleep(settle_secs)

        # Small forward nudge: unsticks a latched ToF reading and physically
        # tests this heading, rather than trusting a standstill reading.
        _timed_translate(0, nudge_secs)
        time.sleep(0.15)

        reads = []
        for _ in range(samples):
            d = _tof_distance(pybot_scout.get_proximity_readings())
            if d is not None:
                reads.append(d)
            time.sleep(0.08)

        median = None
        if reads:
            reads.sort()
            median = reads[len(reads) // 2]
            if best_dist is None or median > best_dist:
                best_dist = median
                best_offset = step_idx * step_deg

        # If the nudge revealed this heading is actually blocked, back off
        # immediately rather than staying pressed against it for the rest
        # of the sweep.
        if median is not None and median < TOF_STOP_DISTANCE_M:
            _timed_translate(180, nudge_secs)
            time.sleep(0.15)
    # The sweep ends back at the starting heading (mod 360); rotate the
    # remaining bit (if any) to face whichever heading had the most room.
    final_rotate = best_offset % total_deg
    if final_rotate > 0:
        _timed_rotate(2, final_rotate)
        time.sleep(0.2)
    return best_dist


def _wag_scan_left_right(angle_deg=WAG_ANGLE_DEG, settle_secs=WAG_SETTLE_SECS, samples=WAG_SAMPLES):
    """Briefly look left then right of straight-ahead and sample ToF at each,
    then curve back to center -- all while continuing to drive forward the
    whole time (continuous set_translate_rotate at WAG_ROTATION_SPEED, the
    same non-blocking cmd_vel-streaming path _maybe_bounce() uses), not a
    full stop-and-pivot via the blocking set_rotate_3()/algo_roll service
    call used elsewhere (e.g. _tof_search_for_room()). Returns (left_dist,
    right_dist), each a metres distance or None if no valid ToF reading was
    seen at that heading.
    """
    def _sample():
        reads = []
        for _ in range(samples):
            d = _tof_distance(pybot_scout.get_proximity_readings())
            if d is not None:
                reads.append(d)
            time.sleep(0.08)
        if not reads:
            return None
        reads.sort()
        return reads[len(reads) // 2]

    pybot_scout.set_rotationSpeed(WAG_ROTATION_SPEED)

    pybot_scout.set_translate_rotate(1, 0)          # curve left while still driving forward
    time.sleep(float(angle_deg) / WAG_ROTATION_SPEED + settle_secs)
    left_dist = _sample()

    pybot_scout.set_translate_rotate(2, 0)          # curve across to look right
    time.sleep(float(angle_deg * 2) / WAG_ROTATION_SPEED + settle_secs)
    right_dist = _sample()

    pybot_scout.set_translate_rotate(1, 0)          # curve back to center
    time.sleep(float(angle_deg) / WAG_ROTATION_SPEED + settle_secs)

    # Restore normal driving -- mirrors the cleanup _maybe_bounce() does
    # after its own continuous curve (set_translate_4 also zeroes angular.z).
    pybot_scout.set_rotationSpeed(ROTATION_SPEED)
    pybot_scout.set_translate_4(0, DRIVE_SPEED)

    return left_dist, right_dist


def _maybe_bounce(left_dist, right_dist):
    """Given a _wag_scan_left_right() result, decide whether a wall is being
    approached at an angle and, if so, curve away from it.

    Rather than an instantaneous direction change, this keeps the robot
    driving forward while slowly rotating away from whichever side read
    closer (set_translate_rotate -- continuous, non-blocking, unlike the
    discrete set_rotate_3 used elsewhere) for long enough to turn
    BOUNCE_ANGLE_DEG, like a ball bouncing off an angled wall but spread
    over a couple of seconds so the robot never fully stops. Because
    forward is always translate degree 0 relative to the robot's own body,
    once the rotation is stopped again the robot is immediately driving in
    its new (post-bounce) heading with the ToF sensor already looking the
    same way -- no separate re-alignment step is needed.

    Returns True if a bounce was triggered.
    """
    if left_dist is None or right_dist is None:
        return False
    closer = min(left_dist, right_dist)
    diff = abs(left_dist - right_dist)
    if closer > BOUNCE_TRIGGER_MAX_M or diff < BOUNCE_TRIGGER_DIFF_M:
        return False

    away_direction = 2 if left_dist < right_dist else 1   # 1=left, 2=right
    LOGGER.log("tof_bounce", left_distance=left_dist, right_distance=right_dist,
               away_direction=away_direction)
    print("Wall at an angle (left=%.2fm right=%.2fm) - bouncing %s." %
          (left_dist, right_dist, "right" if away_direction == 2 else "left"))

    pybot_scout.set_rotationSpeed(BOUNCE_ROTATE_DEG_PER_SEC)
    pybot_scout.set_translate_rotate(away_direction, 0)
    time.sleep(float(BOUNCE_ANGLE_DEG) / BOUNCE_ROTATE_DEG_PER_SEC)

    # Restore normal driving -- set_translate_4 sets angular.z back to 0, so
    # this also cleanly ends the bounce's rotation component.
    pybot_scout.set_rotationSpeed(ROTATION_SPEED)
    pybot_scout.set_translate_4(0, DRIVE_SPEED)
    return True


def _trigger_return_to_dock():
    """Ask the vendor firmware to auto-dock via the nav_low_bat service.

    Blocks (polling, non-busy) until the battery monitor reports charging=True
    (dock success), a BACK_UP_FAIL/CANCEL status is seen, or AUTO_DOCK_TIMEOUT_SECS
    elapses. Returns True on success, False otherwise.
    """
    print("Battery at/below %.0f%% - triggering auto-return-to-dock (nav_low_bat)." % RETURN_BATTERY_PCT)
    LOGGER.log("auto_dock_triggered", battery_pct=BATTERY.get_percent())
    pybot_scout.stop_move()

    last_status = [None]

    def _status_cb(msg):
        try:
            code = int(msg.status[0])
        except Exception:
            return
        if code != last_status[0]:
            last_status[0] = code
            LOGGER.log("auto_dock_backup_status", status=code)

    sub = rospy.Subscriber("/CoreNode/backing_up_status", _RollerEyeStatus, _status_cb)
    try:
        rospy.wait_for_service("/nav_low_bat", timeout=5.0)
        rospy.ServiceProxy("/nav_low_bat", _nav_low_bat_srv)()
    except Exception as exc:
        LOGGER.log("auto_dock_call_failed", error=str(exc))
        sub.unregister()
        return False

    deadline = time.time() + AUTO_DOCK_TIMEOUT_SECS
    while time.time() < deadline:
        if BATTERY.is_charging():
            LOGGER.log("auto_dock_success", final_status=last_status[0])
            sub.unregister()
            return True
        if last_status[0] in (5, 7):  # BACK_UP_FAIL, BACK_UP_CANCEL
            LOGGER.log("auto_dock_failed", status=last_status[0])
            sub.unregister()
            return False
        time.sleep(1.0)
    LOGGER.log("auto_dock_timeout", final_status=last_status[0])
    sub.unregister()
    return False


# ── vendor service pause/resume ────────────────────────────────────────────────

def _find_pids(process_name):
    """Return PIDs of processes whose command line contains process_name."""
    try:
        output = subprocess.check_output(["pgrep", "-f", process_name])
    except subprocess.CalledProcessError:
        return []   # pgrep exits non-zero when nothing matches -- not an error here
    except Exception as exc:
        LOGGER.log("pgrep_failed", process_name=process_name, error=str(exc))
        return []
    return [int(pid) for pid in output.decode().split() if pid.strip()]


def _pause_unneeded_services():
    """Pause (SIGSTOP) the vendor camera/cloud ROS nodes this script never
    uses, to free their CPU for the duration of the run (or for as long as
    the 'p' toggle leaves them paused -- see _toggle_paused_services). Safe
    to call repeatedly: skips any pid already tracked as paused.
    """
    already_paused = set(pid for _, pid in _PAUSED_PIDS)
    for name in PAUSED_SERVICE_PROCESS_NAMES:
        for pid in _find_pids(name):
            if pid in already_paused:
                continue
            try:
                subprocess.check_call(["sudo", "kill", "-STOP", str(pid)])
                _PAUSED_PIDS.append((name, pid))
                print("Paused %s (pid %d) to free CPU." % (name, pid))
            except Exception as exc:
                LOGGER.log("service_pause_failed", process_name=name, pid=pid, error=str(exc))
    LOGGER.log("services_paused", paused=[{"name": n, "pid": p} for n, p in _PAUSED_PIDS])


def _resume_paused_services():
    """Resume (SIGCONT) any processes paused by _pause_unneeded_services.
    Safe to call more than once (e.g. atexit firing after an explicit 'p'
    toggle already resumed them) -- clears the list after resuming so a
    second call is a no-op.
    """
    if not _PAUSED_PIDS:
        return
    resumed = []
    for name, pid in _PAUSED_PIDS:
        if not os.path.exists("/proc/%d" % pid):
            continue    # process is gone entirely -- nothing to resume
        try:
            subprocess.check_call(["sudo", "kill", "-CONT", str(pid)])
            resumed.append({"name": name, "pid": pid})
        except Exception as exc:
            LOGGER.log("service_resume_failed", process_name=name, pid=pid, error=str(exc))
    print("Resumed %d paused service(s)." % len(resumed))
    LOGGER.log("services_resumed", resumed=resumed)
    del _PAUSED_PIDS[:]


def _toggle_paused_services():
    """Flip the vendor camera/cloud services between paused/running -- bound
    to the 'p' key via _keyboard_listener. _SERVICES_PAUSED tracks the
    current state so the dashboard/log can show it as a status indicator.
    """
    if _SERVICES_PAUSED["value"]:
        _resume_paused_services()
        _SERVICES_PAUSED["value"] = False
        print("Vendor camera/cloud services: RUNNING (resumed via 'p').")
    else:
        _pause_unneeded_services()
        _SERVICES_PAUSED["value"] = True
        print("Vendor camera/cloud services: PAUSED (suspended via 'p').")
    LOGGER.log("services_toggled", paused=_SERVICES_PAUSED["value"])


# ── keyboard quit listener ──────────────────────────────────────────────────────

def _keyboard_listener():
    """Background thread: watches stdin for single keypresses (no Enter
    needed) -- 'q' requests a clean shutdown via _QUIT_REQUESTED (polled by
    the main loop), 'p' toggles the vendor camera/cloud services
    paused/running (_toggle_paused_services). Does nothing if stdin isn't an
    interactive terminal (e.g. run under nohup/systemd with no TTY) --
    Ctrl-C/SIGTERM still work either way via _signal_handler.
    """
    if not sys.stdin.isatty():
        return
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while not _QUIT_REQUESTED.is_set():
            ready, _, _ = select.select([sys.stdin], [], [], 0.5)
            if not ready:
                continue
            ch = sys.stdin.read(1).lower()
            if ch == 'q':
                _QUIT_REQUESTED.set()
            elif ch == 'p':
                _toggle_paused_services()
    except Exception as exc:
        LOGGER.log("keyboard_listener_error", error=str(exc))
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


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


# ── charger exit ───────────────────────────────────────────────────────────────

def _exit_charger_if_needed():
    """Drive straight off the charging station if we appear to be docked.

    Returns True if a charger-exit move was performed.
    """
    charging = CHARGER_STATUS.wait_for_status(timeout_secs=2.5)

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
    _timed_translate_smooth(0, CHARGER_EXIT_SECS)
    time.sleep(0.2)
    LOGGER.log("charger_exit_completed")
    return True


# ── main loop ──────────────────────────────────────────────────────────────────

def start():
    ODOM_TRACKER.start()
    PILE_DETECTOR.start()
    CHARGER_STATUS.start()
    BATTERY.start(logger=LOGGER)

    pybot_scout.set_rotationSpeed(ROTATION_SPEED)
    pybot_scout.set_translationSpeed(DRIVE_SPEED)
    DASHBOARD.start()
    inventory = log_ros_inventory(LOGGER)
    DASHBOARD.update_ros_topics(inventory.get("topics", []))

    subscribed = _subscribe_topics(set())
    LOGGER.log(
        "run_started",
        drive_speed=DRIVE_SPEED,
        crawl_speed=CRAWL_SPEED,
        rotation_speed=ROTATION_SPEED,
        tick_secs=TICK_SECS,
        charger_exit_secs=CHARGER_EXIT_SECS,
        proximity_topics=sorted(subscribed),
        return_battery_pct=RETURN_BATTERY_PCT,
        tof_stop_distance_m=TOF_STOP_DISTANCE_M,
        tof_search_steps=TOF_SEARCH_STEPS,
        tof_search_step_deg=TOF_SEARCH_STEP_DEG,
        tof_search_nudge_secs=TOF_SEARCH_NUDGE_SECS,
        stuck_tof_epsilon_m=STUCK_TOF_EPSILON_M,
        stuck_tof_timeout_secs=STUCK_TOF_TIMEOUT_SECS,
        wag_angle_deg=WAG_ANGLE_DEG,
        wag_rotation_speed=WAG_ROTATION_SPEED,
        wag_interval_secs=WAG_INTERVAL_SECS,
        wag_trigger_distance_m=WAG_TRIGGER_DISTANCE_M,
        wag_post_maneuver_grace_secs=WAG_POST_MANEUVER_GRACE_SECS,
        bounce_trigger_diff_m=BOUNCE_TRIGGER_DIFF_M,
        bounce_trigger_max_m=BOUNCE_TRIGGER_MAX_M,
        bounce_angle_deg=BOUNCE_ANGLE_DEG,
        bounce_rotate_deg_per_sec=BOUNCE_ROTATE_DEG_PER_SEC,
    )

    print("Waiting up to 5s for battery status…")
    batt_deadline = time.time() + 5.0
    while BATTERY.get_percent() is None and time.time() < batt_deadline:
        time.sleep(0.2)
    LOGGER.log("battery_initial", percent=BATTERY.get_percent(), charging=BATTERY.is_charging())

    DASHBOARD.update_state(heading_deg=0, mode="startup", services_paused=_SERVICES_PAUSED["value"])
    DASHBOARD.update_sensors(pybot_scout.get_proximity_readings())
    DASHBOARD.tick(force=True)

    # ── leave the dock first ───────────────────────────────────────────────────
    _exit_charger_if_needed()
    TOF_STUCK_DETECTOR.reset()

    print("ToF-only straight-line explorer started.  Press Ctrl-C to stop.")

    next_rediscovery = time.time() + REDISCOVERY_SECS
    next_battery_check = time.time() + BATTERY_CHECK_INTERVAL_SECS
    next_wag_ts = time.time() + WAG_INTERVAL_SECS

    while True:
        # ── keyboard 'q'-to-quit: checked first so it takes effect promptly ────
        if _QUIT_REQUESTED.is_set():
            print("\n'q' pressed – stopping robot.")
            LOGGER.log("quit_key_pressed")
            DASHBOARD.close()
            pybot_scout.stop()

        # ── charging-wait mode: docked/charging, waiting to reach UNDOCK_BATTERY_PCT ──
        if _charge_wait["active"]:
            if time.time() >= next_battery_check:
                next_battery_check = time.time() + BATTERY_CHECK_INTERVAL_SECS
                batt_pct = BATTERY.get_percent()
                charging = BATTERY.is_charging()
                LOGGER.log("charge_wait_check", percent=batt_pct, charging=charging)
                if not charging:
                    LOGGER.log("charge_wait_ended_not_charging")
                    _charge_wait["active"] = False
                    TOF_STUCK_DETECTOR.reset()
                elif batt_pct is not None and batt_pct >= UNDOCK_BATTERY_PCT:
                    print("Battery at %.0f%% - undocking and resuming exploration." % batt_pct)
                    LOGGER.log("undock_started", percent=batt_pct, threshold=UNDOCK_BATTERY_PCT)
                    _exit_charger_if_needed()
                    TOF_STUCK_DETECTOR.reset()
                    LOGGER.log("undock_completed")
                    _charge_wait["active"] = False
            DASHBOARD.update_state(mode="charge_wait", heading_deg=0, services_paused=_SERVICES_PAUSED["value"])
            DASHBOARD.tick()
            time.sleep(TICK_SECS)
            continue

        # ── battery watchdog: try auto-dock once low AND pile is visible ─────────
        if time.time() >= next_battery_check:
            next_battery_check = time.time() + BATTERY_CHECK_INTERVAL_SECS
            batt_pct = BATTERY.get_percent()
            pile_visible = PILE_DETECTOR.was_recently_seen(within_secs=PILE_VISIBLE_WINDOW_SECS)
            LOGGER.log("battery_check", percent=batt_pct, pile_visible=pile_visible)
            if batt_pct is not None and batt_pct <= RETURN_BATTERY_PCT and pile_visible:
                pybot_scout.stop_move()
                LOGGER.log("battery_low_stop", percent=batt_pct, threshold=RETURN_BATTERY_PCT)
                DASHBOARD.update_state(mode="battery_low_returning", heading_deg=0)
                DASHBOARD.tick(force=True)
                docked = _trigger_return_to_dock()
                _battery_triggered_return["done"] = docked or _battery_triggered_return["done"]
                if docked:
                    _charge_wait["active"] = True
                    continue

        # ── periodic topic rediscovery (helps if sensors come online late) ────
        if time.time() >= next_rediscovery:
            subscribed = _subscribe_topics(subscribed)
            next_rediscovery = time.time() + REDISCOVERY_SECS

        readings = pybot_scout.get_proximity_readings()
        tof_dist_raw = _tof_distance(readings)
        tof_dist = TOF_HISTORY.update(tof_dist_raw)
        # A frozen raw reading only counts as "stuck" when it's close enough
        # (or entirely invalid) to actually matter -- see STUCK_RELEVANT_DISTANCE_M
        # above. A steady FAR reading is normal (long corridor, mid-curve),
        # not stuck; avoids false-triggering blind mode while genuinely
        # still driving (2026-07-26 v9).
        is_frozen = TOF_STUCK_DETECTOR.update(tof_dist_raw)
        is_stuck = is_frozen and (tof_dist_raw is None or tof_dist_raw <= STUCK_RELEVANT_DISTANCE_M)
        DASHBOARD.update_sensors(readings)
        DASHBOARD.update_state(mode="drive_loop", heading_deg=0, tof_distance=tof_dist, tof_stuck=is_stuck,
                                services_paused=_SERVICES_PAUSED["value"])
        DASHBOARD.tick()

        # ── ToF stuck -> "blind" fallback driving ───────────────────────────
        # A frozen/unmeasurable ToF reading means the sensor itself can't be
        # trusted right now, so the normal search maneuver (which reads ToF
        # throughout) isn't reliable either. Keep exploring blind instead of
        # stopping and waiting -- see BLIND_MODE_* constants above.
        if is_stuck:
            if not _blind_mode["active"]:
                _blind_mode["active"] = True
                _blind_mode["next_redirect_ts"] = time.time() + BLIND_MODE_REDIRECT_INTERVAL_SECS
                LOGGER.log("blind_mode_entered", raw_distance=tof_dist_raw)
                print("ToF stuck/frozen - falling back to blind driving until it recovers.")
            DASHBOARD.update_state(mode="blind_drive", heading_deg=0, tof_distance=tof_dist, tof_stuck=True,
                                    services_paused=_SERVICES_PAUSED["value"])
            DASHBOARD.tick()
            if time.time() >= _blind_mode["next_redirect_ts"]:
                _blind_mode["next_redirect_ts"] = time.time() + BLIND_MODE_REDIRECT_INTERVAL_SECS
                turn_deg = random.uniform(BLIND_MODE_MIN_TURN_DEG, BLIND_MODE_MAX_TURN_DEG)
                direction = random.choice((1, 2))   # 1=left, 2=right
                LOGGER.log("blind_mode_redirect", turn_deg=turn_deg, direction=direction)
                print("Blind mode: changing direction (%.0f deg %s)." %
                      (turn_deg, "right" if direction == 2 else "left"))
                pybot_scout.stop_move()
                pybot_scout.set_translationSpeed(CRAWL_SPEED)
                _timed_translate(180, BLIND_MODE_BACKUP_SECS)
                time.sleep(0.2)
                pybot_scout.set_translationSpeed(DRIVE_SPEED)
                _timed_rotate(direction, turn_deg)
                time.sleep(0.2)
                # Just committed to a fresh heading -- keep driving it for a
                # while rather than immediately wag-checking again.
                next_wag_ts = time.time() + WAG_POST_MANEUVER_GRACE_SECS
            pybot_scout.set_translate_4(0, DRIVE_SPEED)
            time.sleep(TICK_SECS)
            continue

        if _blind_mode["active"]:
            _blind_mode["active"] = False
            TOF_HISTORY.reset()
            LOGGER.log("blind_mode_exited")
            print("ToF back online - resuming normal ToF-based navigation.")

        # ── ToF critical stop: a genuinely close, trustworthy reading ───────
        is_critical = tof_dist is not None and tof_dist < TOF_STOP_DISTANCE_M
        if is_critical:
            LOGGER.log("tof_critical_stop", distance=tof_dist, raw_distance=tof_dist_raw)
            print("ToF distance %.2f m - stopping and searching for room." % tof_dist)
            pybot_scout.stop_move()
            pybot_scout.set_translationSpeed(CRAWL_SPEED)
            _timed_translate(180, TOF_BACKUP_SECS)
            time.sleep(0.2)
            best_dist = _tof_search_for_room()
            LOGGER.log("tof_critical_stop_recovered", best_distance_found=best_dist)
            pybot_scout.set_translationSpeed(DRIVE_SPEED)
            TOF_HISTORY.reset()
            TOF_STUCK_DETECTOR.reset()
            # The search sweep just found/committed to the best heading
            # available -- keep driving it for a while before the next wag
            # check instead of immediately second-guessing it.
            next_wag_ts = time.time() + WAG_POST_MANEUVER_GRACE_SECS
            time.sleep(PAUSE_SECS)
            continue

        # ── periodic left-right wag scan: catch an angled approach early ────
        # "If in doubt, keep driving": only actually interrupt for a look
        # around when the forward ToF already suggests something's within
        # WAG_TRIGGER_DISTANCE_M -- there's usually a lot more open room
        # ahead than that, so skip the scan (and the curve it costs) entirely
        # rather than needlessly checking side-to-side while the way ahead
        # already reads clear or isn't measured yet.
        if time.time() >= next_wag_ts:
            next_wag_ts = time.time() + WAG_INTERVAL_SECS
            if tof_dist is not None and tof_dist <= WAG_TRIGGER_DISTANCE_M:
                left_dist, right_dist = _wag_scan_left_right()
                DASHBOARD.update_state(mode="drive_loop", heading_deg=0, tof_distance=tof_dist,
                                        tof_stuck=is_stuck, wag_left=left_dist, wag_right=right_dist)
                if _maybe_bounce(left_dist, right_dist):
                    TOF_HISTORY.reset()
                    TOF_STUCK_DETECTOR.reset()
                    # Bounce already turned us away from the near wall --
                    # give that new heading a longer clear run too.
                    next_wag_ts = time.time() + WAG_POST_MANEUVER_GRACE_SECS
                pybot_scout.set_translate_4(0, DRIVE_SPEED)
                time.sleep(PAUSE_SECS)
                continue

        # ── otherwise just keep driving straight ahead ─────────────────────────
        pybot_scout.set_translate_4(0, DRIVE_SPEED)
        time.sleep(TICK_SECS)


# ── entry point ────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    signal.signal(signal.SIGINT,  _signal_handler)
    signal.signal(signal.SIGHUP,  _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    pybot_scout.start()
    # Vendor camera/cloud services are left running by default now (2026-07-26
    # v9, explicit user request) -- only paused on demand via the 'p' key
    # (_toggle_paused_services). atexit still registers the resume so a run
    # that WAS toggled paused always cleans up on exit; it's a no-op if
    # nothing was ever paused (_resume_paused_services returns early on an
    # empty _PAUSED_PIDS list).
    atexit.register(_resume_paused_services)

    _keyboard_thread = threading.Thread(target=_keyboard_listener)
    _keyboard_thread.daemon = True
    _keyboard_thread.start()
    print("ToF-only straight-line explorer ready.  Press 'q' to stop, 'p' to toggle paused vendor services (or Ctrl-C to stop).")

    try:
        start()
    except Exception as exc:
        LOGGER.log("run_exception", error=str(exc), error_type=exc.__class__.__name__)
        pybot_scout.handle_exception(exc.__class__.__name__ + ': ' + str(exc))

    stats = ODOM_TRACKER.get_stats()
    LOGGER.log("run_summary", auto_docked=_battery_triggered_return["done"],
               final_battery_pct=BATTERY.get_percent(), **stats)
    DASHBOARD.close()
    ODOM_TRACKER.stop()
    PILE_DETECTOR.stop()
