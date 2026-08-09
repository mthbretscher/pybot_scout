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
  - Primary: lower-center camera brightness. IMPORTANT polarity: bright =
    caution/near obstacle, dark = clear/open space. This is backwards from
    naive "well-lit room" intuition, but matches how the robot's own IR
    illumination behaves indoors -- nearby reflective surfaces bounce much
    more light back to the sensor than distant/open areas (like a camera
    flash), so brightness roughly tracks proximity, not room lighting.
    Confirmed empirically 2026-07-26: center brightness 66-143 while 0.03-0.33m
    from a wall/dock (i.e. "close" reads bright, not dark).
  - Secondary: left/right brightness delta for lateral steering
  - Tertiary: brightness-over-time delta (approaching obstacle)
  - Safety layer: /SensorNode/tof forward distance is combined as a
    potential-field repulsion vector on top of the camera targets (caps
    speed and adds a steering offset as distance shrinks below
    TOF_SLOW_DISTANCE_M, with a hard stop-and-back-away below
    TOF_STOP_DISTANCE_M) -- this corroborates the camera heuristic, which
    can be fooled by light-coloured close obstacles or dark doorways.

Any other discovered proximity topics are subscribed and logged for
diagnostics only.

Adaptive tuning: a couple of "how cautious" knobs (tof_slow_distance_m,
max_speed) are not fixed constants -- ADAPTIVE_TUNER nudges them up/down
over time using real wheel-odometry distance-covered-per-second as a reward
signal (bounded hill-climbing, one knob at a time; see its docstring). The
hard ToF stop-distance safety floor is never touched by this. Disable with
PYBOT_SCOUT_ADAPTIVE_TUNING=0.

At startup the script detects whether the robot is sitting on its charging
station (via /SensorNode/simple_battery_status).  If so it drives straight
forward for CHARGER_EXIT_SECS to clear the dock before beginning exploration.

Battery watchdog / auto-dock (best-effort, not guaranteed reliable):
  While exploring, if battery % drops to PYBOT_SCOUT_RETURN_BATTERY_PCT (default
  50) *and* the charging pile has been visually seen recently (via
  ChargingPileDetector / /CoreNode/chargingPile), the robot stops and calls the
  vendor's /nav_low_bat service to try to dock.  If the pile hasn't been seen,
  it just keeps exploring and re-checks on the next tick -- nav_low_bat fails
  fast (not a real search) when the pile isn't in view, so we don't force it
  blindly.  Once actually charging, exploration pauses until battery reaches
  PYBOT_SCOUT_UNDOCK_BATTERY_PCT (default 99), at which point the robot drives
  straight forward to unlodge itself and resumes exploring.

Usage:
    python scripts/obstacle_avoidance.py

Environment:
    PYBOT_SCOUT_ALLOW_SENSORLESS_FALLBACK  – set to "1" to allow movement
                                              without camera data
    PYBOT_SCOUT_PROXIMITY_TOPICS           – override discovered topics
    PYBOT_SCOUT_RETURN_BATTERY_PCT         – battery % that triggers a dock
                                              attempt when pile is visible
                                              (default: 50)
    PYBOT_SCOUT_UNDOCK_BATTERY_PCT         – battery % at which to unlodge and
                                              resume exploring while charging
                                              (default: 99)
"""

import os
import signal
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT   = os.path.dirname(SCRIPT_DIR)
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import rospy

from pybot_scout.battery import BatteryMonitor
from pybot_scout.charging_pile import ChargingPileDetector, ChargingStatusDetector
from pybot_scout.camera import BrightnessWindow, GreyImageMonitor
from pybot_scout.feedback import FeedbackLogger
from pybot_scout.odometry import OdometryTracker
from pybot_scout.proximity import discover_proximity_topics
from pybot_scout.dashboard import ScriptDashboard
from pybot_scout.ros_inventory import log_ros_inventory
from pybot_scout.ros_bridge import nav_low_bat as _nav_low_bat_srv
from pybot_scout.ros_bridge import status as _RollerEyeStatus
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

# Camera brightness thresholds (centre lower-panel brightness 0–255).
# Polarity: BRIGHT = near/obstacle (IR reflection off a close surface), DARK =
# open/clear (see module docstring). These are first-pass estimates derived
# from a small live sample (center 66-143 while 0.03-0.33m from a wall) -- no
# clean "far, open room" baseline has been measured yet, so treat as tunable
# pending a proper calibration run.
OBSTACLE_STOP_BRIGHTNESS  = 130.0  # at or above this → v_target = 0  (hard brake)
OBSTACLE_SLOW_BRIGHTNESS  = 90.0   # transition zone boundary (full→crawl speed)
OBSTACLE_PIVOT_BRIGHTNESS = 120.0  # centre brightness that enables in-place pivot
PIVOT_HEADING_DEG        = 15.0   # |heading_target| that triggers pivot when stopped

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
STUCK_COOLDOWN_SECS     = 15.0  # minimum seconds between successive stuck-recoveries
STUCK_ROTATE_DEG        = 90    # degrees to rotate during recovery
STUCK_DRIVE_SECS        = 2.0   # seconds to drive straight after recovery rotation

# ToF vector-repulsion safety layer (added 2026-07-26 v2, retuned v4) ────────
# Camera brightness is the primary steering signal, but it can be fooled (a
# light-coloured close obstacle looks the same as open space; a dark doorway
# looks the same as a wall).  /SensorNode/tof gives a real forward distance in
# metres and is used here as a corroborating potential-field repulsion vector
# on top of the camera steering, not a replacement for it.
#
# v4 retune: live run 20260726_153757 showed tof_repulsion firing on 92% of
# ticks (224/243) with v_cmd pinned at 0.000 the *entire* run -- the original
# TOF_SLOW_DISTANCE_M=0.6 was well above this apartment's normal ambient ToF
# reading (median 0.44 m, p10 0.33 m across that run), so the repulsion layer
# was almost always braking even with nothing actually blocking the camera's
# view, and the analog speed state never got a chance to ramp up before being
# chopped back down again. The same run also recorded one genuine close call
# at 0.099 m (correctly triggered the hard stop). Retuned so the layer mostly
# stays out of the way at normal indoor distances and only intervenes
# meaningfully closer than the observed ambient floor ("if in doubt, keep
# driving" -- only react to a real, comparatively close reading).
TOF_TOPIC_HINT           = "tof"   # matched against proximity-reading topic names
TOF_SLOW_DISTANCE_M      = 0.25    # start capping speed / adding repulsion below this
TOF_STOP_DISTANCE_M      = 0.10    # hard stop + search-for-room below this (verified robustified reading)
TOF_REPULSE_GAIN_DEG     = 40.0    # max steer (deg) added by repulsion at full magnitude
TOF_BACKUP_SECS          = 1.0     # seconds to reverse during a tof critical-stop recovery
TOF_BACKUP_ROTATE_DEG    = 30      # degrees to pivot after backing away
TOF_BACKUP_COOLDOWN_SECS = 4.0     # minimum seconds between tof-triggered backups
TOF_SEARCH_STEPS         = 6       # number of headings sampled in the post-critical-stop room search
TOF_SEARCH_STEP_DEG      = 60      # degrees rotated per search step (steps*step_deg = full 360 sweep)
TOF_SEARCH_SETTLE_SECS   = 0.35    # settle time after each rotation step before sampling
TOF_SEARCH_SAMPLES       = 3       # ToF samples taken (median) at each heading

# Online brightness<->distance calibration (added 2026-07-26 v3) ─────────────
# OBSTACLE_STOP/SLOW_BRIGHTNESS above are rough first-pass guesses (no clean
# far/open baseline was available when they were picked). Since ToF gives a
# real distance, use it as ground truth to learn what a given brightness
# level actually means and re-derive the thresholds from that fit instead of
# trusting hand-picked constants forever. A sample is only trusted when the
# robot is judged near-stationary/crawling AND the ToF reading has held
# steady over the last few ticks -- guarding against the possibility (raised
# during testing) that ToF readings aren't meaningful while the robot is
# actively moving/rotating (motion/vibration could sweep the beam across
# unrelated surfaces).
CALIBRATION_MAX_SPEED           = MIN_SPEED  # only learn from near-stationary/crawl ticks
CALIBRATION_STABILITY_WINDOW    = 4          # consecutive ticks of steady ToF required to trust a sample
CALIBRATION_STABILITY_EPS_M     = 0.03       # max spread within the window to call it "steady"
CALIBRATION_MIN_SAMPLES         = 25         # samples needed before trusting a fitted slope
CALIBRATION_MIN_BRIGHTNESS_SPAN = 15.0       # brightness variety needed to fit a meaningful slope
CALIBRATION_REFIT_INTERVAL_SECS = 10.0       # how often to recompute thresholds from the fit

ALLOW_SENSORLESS = os.environ.get("PYBOT_SCOUT_ALLOW_SENSORLESS_FALLBACK", "0") == "1"

# Battery watchdog: stop exploring and let the vendor firmware auto-dock
# (via the nav_low_bat service, which drives the BACK_UP_* state machine using
# the ibeacon signal + charging-pile camera detection) once battery drops to
# or below this percentage.  Verified live on 2026-07-26: nav_low_bat reliably
# docks the robot from a few metres away in under a minute — far more robust
# than any home-grown dead-reckoning return, because /MotorNode/baselink_odom_relative
# does NOT accumulate distance the way OdometryTracker assumes (see instructions.md).
RETURN_BATTERY_PCT      = float(os.environ.get("PYBOT_SCOUT_RETURN_BATTERY_PCT", "50"))
BATTERY_CHECK_INTERVAL_SECS = 5.0
AUTO_DOCK_TIMEOUT_SECS  = 180.0

# Once docked/charging, wait until battery reaches this level, then drive
# straight off the dock and resume exploring.  Per live testing on 2026-07-26,
# the vendor's nav_low_bat charging-pile vision detection is unreliable unless
# the pile is currently in the camera's field of view (ChargingPileDetector /
# /CoreNode/chargingPile), so we only attempt a dock when the pile has
# actually been seen recently -- otherwise we just keep exploring and retry
# on the next battery-check tick.
UNDOCK_BATTERY_PCT      = float(os.environ.get("PYBOT_SCOUT_UNDOCK_BATTERY_PCT", "99"))
PILE_VISIBLE_WINDOW_SECS = 2.0

FEEDBACK_DIR   = os.path.join(REPO_ROOT, "run_feedback")
LOGGER         = FeedbackLogger("obstacle_avoidance", output_dir=FEEDBACK_DIR)
ODOM_TRACKER   = OdometryTracker()
PILE_DETECTOR  = ChargingPileDetector()
CHARGER_STATUS = ChargingStatusDetector()
BATTERY        = BatteryMonitor()
DASHBOARD      = ScriptDashboard("obstacle_avoidance", logger=LOGGER)
CAMERA         = GreyImageMonitor()

_battery_triggered_return = {"done": False}
# When True, the main loop stops camera-driven exploration and just waits for
# the battery to reach UNDOCK_BATTERY_PCT before unlodging and resuming.
_charge_wait = {"active": False}

# Brightness thresholds actually used by _camera_drive_command(). Start out at
# the static first-guess constants above; get replaced with data-derived
# values once BRIGHTNESS_CALIBRATOR has enough confirmed (brightness, ToF
# distance) samples (see _BrightnessDistanceCalibrator below).
_brightness_thresholds = {
    "stop": OBSTACLE_STOP_BRIGHTNESS,
    "slow": OBSTACLE_SLOW_BRIGHTNESS,
    "pivot": OBSTACLE_PIVOT_BRIGHTNESS,
    "calibrated": False,
}


class _TofHistory(object):
    """Rolling window that smooths/debounces raw ToF readings before they are
    trusted for a safety action (repulsion / critical-stop).

    "If in doubt, keep driving": a single close-looking reading is not enough
    to slow or stop the robot -- we don't yet know whether ToF readings stay
    meaningful while the robot is moving (suspected they may only be reliable
    while it's nearly still), so until CALIBRATION-style corroboration builds
    up across a few consecutive ticks (~1 s at TICK_SECS=0.30), the effective
    distance is reported as None (unknown) rather than reacting to noise.
    Returns the median of the last `window` readings once at least
    `min_samples` have been collected.
    """

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


class _BrightnessDistanceCalibrator(object):
    """Online least-squares fit of camera centre-brightness -> ToF distance.

    Learns what a given brightness level actually corresponds to in metres so
    the OBSTACLE_STOP/SLOW_BRIGHTNESS thresholds can be re-derived from real
    data instead of hand-picked guesses. A sample is only accepted from the
    caller (see main loop) when the robot is judged near-stationary/crawling
    AND the raw ToF reading has held steady over the last few ticks -- both
    guard against the possibility that ToF readings aren't meaningful while
    the robot is actively moving/rotating.
    """

    def __init__(self):
        self._window = []
        self._n = 0
        self._sum_x = 0.0
        self._sum_y = 0.0
        self._sum_xx = 0.0
        self._sum_xy = 0.0
        self._min_x = None
        self._max_x = None

    def note_tof(self, tof_dist):
        """Record a raw ToF sample for stability tracking; return True once
        the last CALIBRATION_STABILITY_WINDOW readings have stayed steady."""
        self._window.append(tof_dist)
        if len(self._window) > CALIBRATION_STABILITY_WINDOW:
            self._window.pop(0)
        if len(self._window) < CALIBRATION_STABILITY_WINDOW:
            return False
        return (max(self._window) - min(self._window)) <= CALIBRATION_STABILITY_EPS_M

    def observe(self, brightness, tof_dist):
        self._n += 1
        self._sum_x += brightness
        self._sum_y += tof_dist
        self._sum_xx += brightness * brightness
        self._sum_xy += brightness * tof_dist
        self._min_x = brightness if self._min_x is None else min(self._min_x, brightness)
        self._max_x = brightness if self._max_x is None else max(self._max_x, brightness)

    def sample_count(self):
        return self._n

    def _fit(self):
        """Return (slope, intercept) mapping brightness -> distance, or None
        if there isn't enough data yet, or the fit doesn't make physical
        sense (distance should fall as brightness rises)."""
        if self._n < CALIBRATION_MIN_SAMPLES:
            return None
        if self._max_x is None or (self._max_x - self._min_x) < CALIBRATION_MIN_BRIGHTNESS_SPAN:
            return None
        denom = self._n * self._sum_xx - self._sum_x * self._sum_x
        if abs(denom) < 1e-6:
            return None
        slope = (self._n * self._sum_xy - self._sum_x * self._sum_y) / denom
        intercept = (self._sum_y - slope * self._sum_x) / self._n
        if slope >= 0.0:
            return None
        return slope, intercept

    def thresholds_from_fit(self):
        """Return (stop_brightness, slow_brightness) derived from the current
        fit, or (None, None) if there isn't a confident fit yet."""
        fit = self._fit()
        if fit is None:
            return None, None
        slope, intercept = fit
        stop_b = (TOF_STOP_DISTANCE_M - intercept) / slope
        slow_b = (_adaptive_params["tof_slow_distance_m"] - intercept) / slope
        if stop_b >= slow_b:
            return None, None
        stop_b = _clamp(stop_b, 10.0, 250.0)
        slow_b = _clamp(slow_b, 5.0, stop_b - 5.0)
        return stop_b, slow_b


TOF_HISTORY  = _TofHistory()
BRIGHTNESS_CALIBRATOR = _BrightnessDistanceCalibrator()


# Adaptive "operant learning" tuner (added 2026-07-26 v5) ────────────────────
# The 0.6 m ToF-slow-distance guess (see v4 note above) was picked from one
# short live sample -- reasonable-looking in isolation, but there's no reason
# to believe *any* hand-picked constant generalises across a whole apartment.
# Rather than guess again, let a couple of specifically-chosen "how cautious"
# knobs tune themselves using a reward signal.
#
# Reliable distance measurement: NO, verified live 2026-07-26 -- both
# /MotorNode/baselink_odom_relative (wheel) and /MotorNode/vio_odom_relative
# (VIO) stayed at *exactly* (0.0, 0.0) through a real ~4s commanded forward
# move that visibly displaced the robot. Best-evidence hypothesis: the
# vendor's pose integration for both only runs during blocking algo_move/
# algo_roll/algo_action service calls (matches an earlier observation that a
# blocking algo_move made pose hover near a small nonzero value then reset to
# (0,0,0) on completion) -- it is simply never exercised by the continuous
# /cmd_vel-republishing style (set_translate_4 etc.) this whole codebase uses
# to drive, so it just sits at all-zero forever in that mode. This is very
# likely a genuine architecture mismatch rather than a bug: the vendor's own
# NavPathNode/algo_* stack presumably relies on it, our continuous-Twist
# control loop just doesn't route through the same path. It could in
# principle be fixed at the firmware level (the roller_eye ROS package is
# open source -- Pilot-Labs-Dev/Scout-open-source -- so MotorNode's C++ could
# be patched to integrate pose continuously regardless of algo_* state,
# rebuilt, and redeployed) but that's a heavier, higher-risk embedded
# cross-compile/redeploy undertaking with no easy rollback -- treat as a
# possible future task, not something to attempt casually.
#
# So instead, the reward below uses only data this script already fully
# controls and knows to be real:
#   - "commanded distance": v_cmd * TICK_SECS, accumulated every tick via
#     note_commanded_distance() -- exactly what our own control loop told
#     the wheels to do. This is a self-referential proxy (it can't detect
#     wheel slip/stall on its own), NOT independent ground truth.
#   - incident count: how many stuck-recoveries / ToF critical-stops (both
#     grounded in real sensor readings -- camera brightness stagnation and
#     ToF proximity -- independent of wheel rotation) happened in the
#     window. This is what actually keeps the learner honest: a setting that
#     only "covers more commanded distance" by repeatedly stalling/crashing
#     gets compounding penalties, not just a flat one-time discount.
#
# Method: simple bounded coordinate-ascent hill-climbing (a minimal, safe form
# of operant learning -- "try a nudge, keep it if it helped") rather than a
# full RL agent, on purpose:
#   - Only a couple of *soft* caution knobs are adaptive (how far out the ToF
#     repulsion starts, how fast MAX_SPEED is allowed to go). The hard ToF
#     stop-distance safety floor (TOF_STOP_DISTANCE_M) is NEVER touched by
#     the learner -- it stays a fixed constant regardless of reward.
#   - One parameter is perturbed at a time (round-robin) so a change in
#     reward can be attributed to that one knob instead of being confounded
#     by several moving at once.
#   - Every cycle re-measures a fresh baseline (rather than trusting an old
#     one) since different parts of the apartment legitimately call for
#     different caution -- the goal is a decent generalist setting, not
#     overfitting to one corner.
ADAPTIVE_TUNING_ENABLED    = os.environ.get("PYBOT_SCOUT_ADAPTIVE_TUNING", "1") == "1"
ADAPTIVE_EVAL_WINDOW_SECS  = 45.0   # how long to measure before scoring a trial
ADAPTIVE_INCIDENT_PENALTY  = 0.5    # reward multiplier PER incident (compounds: 0.5**count) in the window
ADAPTIVE_INITIAL_STEP_FRAC = 0.25   # first nudge size, as a fraction of (hi - lo)
ADAPTIVE_MIN_STEP_FRAC     = 0.1    # smallest nudge size a reverted trial can shrink to

# Live values actually read by _tof_repulsion()/_camera_drive_command() below
# -- start at the (now data-corrected) static guesses above, then get nudged
# by ADAPTIVE_TUNER over time. Same mutable-dict pattern as _brightness_thresholds.
_adaptive_params = {
    "tof_slow_distance_m": TOF_SLOW_DISTANCE_M,
    "max_speed": MAX_SPEED,
}


class _AdaptiveParameter(object):
    """One bounded knob in `_adaptive_params`, tuned via hill-climbing.

    lo/hi are hard bounds the learner can never exceed, chosen so every
    reachable value stays plausibly safe (e.g. tof_slow_distance_m's lower
    bound is kept comfortably above the fixed TOF_STOP_DISTANCE_M safety
    floor). step shrinks whenever a trial is reverted, so the search settles
    down instead of oscillating forever.
    """

    def __init__(self, key, lo, hi, step_frac=ADAPTIVE_INITIAL_STEP_FRAC):
        self.key = key
        self.lo = lo
        self.hi = hi
        span = max(hi - lo, 1e-6)
        self.step = span * step_frac
        self.min_step = span * ADAPTIVE_MIN_STEP_FRAC
        self.direction = 1
        self.settled_value = _adaptive_params[key]

    def begin_trial(self):
        trial_value = _clamp(self.settled_value + self.direction * self.step, self.lo, self.hi)
        _adaptive_params[self.key] = trial_value
        return trial_value

    def accept_trial(self):
        self.settled_value = _adaptive_params[self.key]

    def revert_trial(self):
        _adaptive_params[self.key] = self.settled_value
        self.direction *= -1
        self.step = max(self.step * 0.6, self.min_step)


class _AdaptiveTuner(object):
    """Round-robin hill-climbing over a list of _AdaptiveParameter knobs.

    Each cycle is baseline-window -> trial-window: measure reward with the
    settled value, nudge the parameter, measure reward again, keep the nudge
    only if reward did not get worse (otherwise revert and flip direction).
    """

    def __init__(self, params, window_secs):
        self._params = params
        self._window_secs = window_secs
        self._idx = 0
        self._phase = "baseline"
        self._window_start_ts = None
        self._window_dist_accum = 0.0
        self._incident_count = 0
        self._baseline_reward = None

    def note_incident(self):
        """Record that a stuck-recovery or ToF critical-stop happened during
        the window currently being measured (real-sensor-grounded events --
        see module comment above)."""
        self._incident_count += 1

    def note_commanded_distance(self, delta_m):
        """Accumulate v_cmd*TICK_SECS for the window currently being measured.
        Self-referential (see module comment) but the best available proxy
        given verified-broken ROS odometry/VIO in this drive mode."""
        self._window_dist_accum += delta_m

    def tick(self, now):
        if not self._params:
            return
        if self._window_start_ts is None:
            self._start_window(now)
            return
        if now - self._window_start_ts < self._window_secs:
            return
        reward = self._score_window(now)
        param = self._params[self._idx]
        if self._phase == "baseline":
            self._baseline_reward = reward
            trial_value = param.begin_trial()
            LOGGER.log("adaptive_tuning_eval", phase="baseline", param=param.key,
                       value=round(param.settled_value, 4), trial_value=round(trial_value, 4),
                       reward=round(reward, 4), incident_count=self._incident_count)
            self._phase = "trial"
        else:
            trial_value = _adaptive_params[param.key]
            improved = self._baseline_reward is not None and reward >= self._baseline_reward
            if improved:
                param.accept_trial()
            else:
                param.revert_trial()
            LOGGER.log("adaptive_tuning_eval",
                       phase="trial_accepted" if improved else "trial_reverted",
                       param=param.key, value=round(_adaptive_params[param.key], 4),
                       reward=round(reward, 4), baseline_reward=round(self._baseline_reward, 4),
                       incident_count=self._incident_count)
            self._phase = "baseline"
            self._idx = (self._idx + 1) % len(self._params)
        self._start_window(now)

    def _score_window(self, now):
        elapsed = max(now - self._window_start_ts, 1e-3)
        reward = self._window_dist_accum / elapsed
        reward *= ADAPTIVE_INCIDENT_PENALTY ** self._incident_count
        return reward

    def _start_window(self, now):
        self._window_start_ts = now
        self._window_dist_accum = 0.0
        self._incident_count = 0


# tof_slow_distance_m: lower bound stays a healthy 0.05 m above the fixed hard
# stop floor (never allowed to collapse onto it); upper bound is well below
# the old catastrophic 0.6 m guess. max_speed: lower bound stays above
# CRAWL_SPEED (required by _camera_drive_command's ramp math); upper bound
# gives modest headroom above the original 0.20 m/s guess.
ADAPTIVE_TUNER = _AdaptiveTuner(
    [
        _AdaptiveParameter("tof_slow_distance_m", lo=TOF_STOP_DISTANCE_M + 0.05, hi=0.45),
        _AdaptiveParameter("max_speed", lo=0.12, hi=0.24),
    ],
    ADAPTIVE_EVAL_WINDOW_SECS,
)


def _trigger_return_to_dock():
    """Ask the vendor firmware to auto-dock via the nav_low_bat service.

    Blocks (polling, non-busy) until the battery monitor reports charging=True
    (dock success), a BACK_UP_FAIL/CANCEL status is seen, or AUTO_DOCK_TIMEOUT_SECS
    elapses.  Returns True on success, False otherwise.
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


def _tof_distance(readings):
    """Return the nearest finite, non-negative ToF reading in metres, or None.

    Only considers topics whose name contains TOF_TOPIC_HINT so that unrelated
    range topics (e.g. the ibeacon charging-pile range) are never mistaken for
    a forward-obstacle distance.
    """
    best = None
    for topic, dist in readings.items():
        if TOF_TOPIC_HINT not in topic.lower():
            continue
        d = _safe_float(dist)
        if d is None or d != d:  # None or NaN
            continue
        if d in (float("inf"), float("-inf")) or d < 0.0:
            continue
        if best is None or d < best:
            best = d
    return best


def _tof_repulsion(distance, lateral_bias):
    """Return (speed_scale, heading_offset_deg) from a potential-field
    repulsion vector pointing away from a close forward obstacle.

    speed_scale in [0, 1] multiplies the camera-derived speed target.
    heading_offset_deg is added to the camera-derived heading target, steering
    toward whichever side lateral_bias indicates (>=0 steers right/positive,
    <0 steers left/negative) -- typically fed from the camera's own current
    left/right brightness comparison so the two signals agree.
    """
    slow_distance_m = _adaptive_params["tof_slow_distance_m"]
    if distance is None or distance >= slow_distance_m:
        return 1.0, 0.0
    span = max(slow_distance_m - TOF_STOP_DISTANCE_M, 0.01)
    magnitude = _clamp((slow_distance_m - distance) / span, 0.0, 1.0)
    speed_scale = _clamp(1.0 - magnitude, 0.0, 1.0)
    heading_offset = TOF_REPULSE_GAIN_DEG * magnitude * (1.0 if lateral_bias >= 0 else -1.0)
    return speed_scale, heading_offset


def _tof_search_for_room(steps=TOF_SEARCH_STEPS, step_deg=TOF_SEARCH_STEP_DEG,
                          settle_secs=TOF_SEARCH_SETTLE_SECS, samples=TOF_SEARCH_SAMPLES):
    """Rotate through a full sweep (steps*step_deg = 360 deg by default),
    sampling a debounced (median-of-`samples`) ToF reading at each heading,
    then rotate to face whichever heading had the most room. Used after a
    ToF critical-stop instead of a single fixed-angle guess -- the ToF sensor
    only looks forward, so the only way to find "a direction with more room"
    is to actually turn and look. Returns the best distance found in metres,
    or None if no ToF reading was ever obtained during the sweep.

    Always rotates the same direction (right) so the steps sum to a full
    circle and the robot ends up back at its start heading once done -- the
    final rotate then only needs to cover the offset to the best heading
    found, not backtrack through the whole sweep.
    """
    total_deg = steps * step_deg
    best_offset = 0
    best_dist = _tof_distance(pybot_scout.get_proximity_readings())
    for step_idx in range(1, steps + 1):
        pybot_scout.set_rotate_3(2, step_deg)
        time.sleep(float(step_deg) / ROTATION_SPEED + settle_secs)
        reads = []
        for _ in range(samples):
            d = _tof_distance(pybot_scout.get_proximity_readings())
            if d is not None:
                reads.append(d)
            time.sleep(0.08)
        if reads:
            reads.sort()
            median = reads[len(reads) // 2]
            if best_dist is None or median > best_dist:
                best_dist = median
                best_offset = step_idx * step_deg
    # The sweep ends back at the starting heading (mod 360); rotate the
    # remaining bit (if any) to face whichever heading had the most room.
    final_rotate = best_offset % total_deg
    if final_rotate > 0:
        pybot_scout.set_rotate_3(2, final_rotate)
        time.sleep(float(final_rotate) / ROTATION_SPEED + 0.2)
    return best_dist


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

    # Continuous speed target from centre brightness (bright = near/obstacle).
    # Thresholds come from _brightness_thresholds, which starts at the static
    # OBSTACLE_STOP/SLOW_BRIGHTNESS guesses and gets replaced by
    # BRIGHTNESS_CALIBRATOR's data-derived values once confident (see main
    # loop / _BrightnessDistanceCalibrator).
    #   centre ≥ stop_b        → v_target = 0  (hard brake zone)
    #   slow_b ≤ centre < stop_b → linear ramp from CRAWL_SPEED down to 0
    #   centre < slow_b          → linear ramp from MAX_SPEED down to CRAWL_SPEED
    # (max_speed itself comes from _adaptive_params -- see ADAPTIVE_TUNER.)
    stop_b = _brightness_thresholds["stop"]
    slow_b = _brightness_thresholds["slow"]
    max_speed = _adaptive_params["max_speed"]
    if center >= stop_b:
        v_target = 0.0
    elif center >= slow_b:
        frac = (stop_b - center) / (stop_b - slow_b)
        v_target = frac * CRAWL_SPEED
    else:
        frac = _clamp((slow_b - center) / slow_b, 0.0, 1.0)
        v_target = CRAWL_SPEED + frac * (max_speed - CRAWL_SPEED)

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
    BATTERY.start(logger=LOGGER)
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
        obstacle_stop_brightness=OBSTACLE_STOP_BRIGHTNESS,
        obstacle_slow_brightness=OBSTACLE_SLOW_BRIGHTNESS,
        obstacle_pivot_brightness=OBSTACLE_PIVOT_BRIGHTNESS,
        accel_alpha=ACCEL_ALPHA,
        brake_alpha=BRAKE_ALPHA,
        steer_alpha=STEER_ALPHA,
        tick_secs=TICK_SECS,
        camera_timeout_secs=CAMERA_TIMEOUT_SECS,
        charger_exit_secs=CHARGER_EXIT_SECS,
        allow_sensorless=ALLOW_SENSORLESS,
        proximity_topics=sorted(subscribed),
        return_battery_pct=RETURN_BATTERY_PCT,
        tof_slow_distance_m=TOF_SLOW_DISTANCE_M,
        tof_stop_distance_m=TOF_STOP_DISTANCE_M,
        tof_repulse_gain_deg=TOF_REPULSE_GAIN_DEG,
        adaptive_tuning_enabled=ADAPTIVE_TUNING_ENABLED,
        adaptive_eval_window_secs=ADAPTIVE_EVAL_WINDOW_SECS,
    )

    print("Waiting up to 5s for battery status…")
    batt_deadline = time.time() + 5.0
    while BATTERY.get_percent() is None and time.time() < batt_deadline:
        time.sleep(0.2)
    LOGGER.log("battery_initial", percent=BATTERY.get_percent(), charging=BATTERY.is_charging())

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
    next_battery_check = time.time() + BATTERY_CHECK_INTERVAL_SECS
    next_calibration_refit = time.time() + CALIBRATION_REFIT_INTERVAL_SECS
    bw = BrightnessWindow(STUCK_WINDOW_SIZE)
    last_stuck_ts = 0.0
    last_tof_backup_ts = 0.0

    while True:
        # ── charging-wait mode: docked/charging, waiting to reach UNDOCK_BATTERY_PCT ──
        if _charge_wait["active"]:
            if time.time() >= next_battery_check:
                next_battery_check = time.time() + BATTERY_CHECK_INTERVAL_SECS
                batt_pct = BATTERY.get_percent()
                charging = BATTERY.is_charging()
                LOGGER.log("charge_wait_check", percent=batt_pct, charging=charging)
                if not charging:
                    # Fell off the dock, or status glitch -- resume exploring.
                    LOGGER.log("charge_wait_ended_not_charging")
                    _charge_wait["active"] = False
                elif batt_pct is not None and batt_pct >= UNDOCK_BATTERY_PCT:
                    print("Battery at %.0f%% - undocking and resuming exploration." % batt_pct)
                    LOGGER.log("undock_started", percent=batt_pct, threshold=UNDOCK_BATTERY_PCT)
                    _exit_charger_if_needed()
                    LOGGER.log("undock_completed")
                    _charge_wait["active"] = False
            DASHBOARD.update_state(mode="charge_wait", heading_deg=heading_cmd)
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
                DASHBOARD.update_state(mode="battery_low_returning", heading_deg=heading_cmd)
                DASHBOARD.tick(force=True)
                docked = _trigger_return_to_dock()
                _battery_triggered_return["done"] = docked or _battery_triggered_return["done"]
                if docked:
                    _charge_wait["active"] = True
                    continue
                # Failed to dock (pile lost, timeout, etc.) -- keep exploring and
                # retry on a later tick when the pile is next seen.

        # ── periodic topic rediscovery (helps if sensors come online late) ────
        if time.time() >= next_rediscovery:
            subscribed = _subscribe_topics(subscribed)
            next_rediscovery = time.time() + REDISCOVERY_SECS

        # ── adaptive parameter tuning: evaluate/nudge one knob at a time ──────
        # (see ADAPTIVE_TUNER definition for the reward/safety rationale)
        if ADAPTIVE_TUNING_ENABLED:
            ADAPTIVE_TUNER.tick(time.time())

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

        # ── ToF vector-repulsion safety layer ──────────────────────────────────
        # Corroborates/overrides the camera-brightness targets with a real
        # distance reading so a close obstacle can't be missed just because it
        # happens to look "bright"/open to the camera heuristic.
        #
        # "If in doubt, keep driving": we don't yet know for certain whether
        # ToF readings stay meaningful while the robot is moving/rotating, so
        # the raw reading is smoothed/debounced through TOF_HISTORY first --
        # a lone close-looking sample has no effect; only once a few
        # consecutive ticks agree does it get treated as real evidence of an
        # obstacle. No ToF data (or not-yet-confirmed data) => no repulsion,
        # not a stop.
        tof_dist_raw = _tof_distance(readings)
        tof_dist = TOF_HISTORY.update(tof_dist_raw)
        lateral_bias = drive.get("lateral_norm", 0.0)
        if lateral_bias == 0.0:
            lateral_bias = 1.0 if int(time.time()) % 2 == 0 else -1.0
        tof_speed_scale, tof_heading_offset = _tof_repulsion(tof_dist, lateral_bias)
        if tof_speed_scale < 1.0 or tof_heading_offset != 0.0:
            v_target = v_target * tof_speed_scale
            h_target = _clamp(h_target + tof_heading_offset, -MAX_STEER_DEG, MAX_STEER_DEG)
            LOGGER.log("tof_repulsion", distance=tof_dist, raw_distance=tof_dist_raw,
                       speed_scale=round(tof_speed_scale, 3), heading_offset=round(tof_heading_offset, 2))

        # Online brightness<->distance calibration: only learn from samples
        # where the robot is nearly stationary/crawling AND the raw ToF
        # reading has held steady for a few ticks -- see
        # _BrightnessDistanceCalibrator docstring for the reasoning.
        if (tof_dist_raw is not None and drive.get("center") is not None
                and v_cmd <= CALIBRATION_MAX_SPEED
                and BRIGHTNESS_CALIBRATOR.note_tof(tof_dist_raw)):
            BRIGHTNESS_CALIBRATOR.observe(drive["center"], tof_dist_raw)

        if time.time() >= next_calibration_refit:
            next_calibration_refit = time.time() + CALIBRATION_REFIT_INTERVAL_SECS
            new_stop, new_slow = BRIGHTNESS_CALIBRATOR.thresholds_from_fit()
            if new_stop is not None and new_slow is not None:
                new_pivot = new_slow + 0.75 * (new_stop - new_slow)
                _brightness_thresholds["stop"] = new_stop
                _brightness_thresholds["slow"] = new_slow
                _brightness_thresholds["pivot"] = new_pivot
                _brightness_thresholds["calibrated"] = True
                LOGGER.log("brightness_calibration_updated",
                           stop=round(new_stop, 1), slow=round(new_slow, 1), pivot=round(new_pivot, 1),
                           samples=BRIGHTNESS_CALIBRATOR.sample_count())

        if (tof_dist is not None and tof_dist < TOF_STOP_DISTANCE_M
                and time.time() - last_tof_backup_ts > TOF_BACKUP_COOLDOWN_SECS):
            if ADAPTIVE_TUNING_ENABLED:
                ADAPTIVE_TUNER.note_incident()
            LOGGER.log("tof_critical_stop", distance=tof_dist, raw_distance=tof_dist_raw)
            print("ToF critical distance (%.2f m) - backing away and searching for room." % tof_dist)
            pybot_scout.stop_move()
            pybot_scout.set_translationSpeed(CRAWL_SPEED)
            pybot_scout.set_translate_2(180, TOF_BACKUP_SECS)
            time.sleep(TOF_BACKUP_SECS + 0.2)
            best_dist = _tof_search_for_room()
            LOGGER.log("tof_critical_stop_recovered", best_distance_found=best_dist)
            last_tof_backup_ts = time.time()
            bw.reset()
            v_cmd = 0.0
            heading_cmd = 0.0
            continue

        # ── stuck detection ────────────────────────────────────────────────────
        # Used to cross-check against ODOM_TRACKER's distance-since-window-fill
        # before triggering recovery, to avoid interrupting genuine slow-but-
        # real progress. Removed 2026-07-26: verified live that ODOM_TRACKER's
        # distance never advances during continuous-Twist driving (the only
        # mode this script uses -- see ADAPTIVE_TUNER module comment above),
        # so that cross-check was always silently inert (dist_since_fill was
        # always ~0, never exceeding the threshold, so it never once actually
        # suppressed a recovery). Stuck-detection now relies solely on the
        # camera-brightness stagnation window (bw.is_stuck()) -- a real signal
        # independent of wheel rotation.
        if bw.is_stuck() and time.time() - last_stuck_ts > STUCK_COOLDOWN_SECS:
            if ADAPTIVE_TUNING_ENABLED:
                ADAPTIVE_TUNER.note_incident()
            LOGGER.log("stuck_check", do_unstick=True)
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

        if ADAPTIVE_TUNING_ENABLED:
            ADAPTIVE_TUNER.note_commanded_distance(v_cmd * TICK_SECS)

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
                    and drive["center"] > _brightness_thresholds["pivot"]
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
            tof_distance=tof_dist,
            tof_distance_raw=tof_dist_raw,
            brightness_calibrated=_brightness_thresholds["calibrated"],
            tof_slow_distance_m=round(_adaptive_params["tof_slow_distance_m"], 3),
            max_speed=round(_adaptive_params["max_speed"], 3),
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
            bw.push(lc, ll, lr)


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
    LOGGER.log("run_summary", auto_docked=_battery_triggered_return["done"],
               final_battery_pct=BATTERY.get_percent(), **stats)
    CAMERA.stop()
    DASHBOARD.close()
    ODOM_TRACKER.stop()
    PILE_DETECTOR.stop()
    CHARGER_STATUS.stop()
    BATTERY.stop()
    LOGGER.log("run_stopped")
    LOGGER.close()
    pybot_scout.stop()
