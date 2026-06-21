# -*- coding: utf-8 -*-
"""
Return the robot to its starting position by replaying a logged exploration
path in reverse.

The script loads a run_feedback JSONL file produced by
obstacle_avoidance.py (auto-detects the most recent file if no path is given),
extracts the pose snapshots embedded in step_selected events, and
drives the robot backward along those waypoints using live odometry to measure
progress.

Termination conditions (whichever comes first):
  1. All waypoints replayed.
  2. The robot arrives within GOAL_RADIUS_M of the first logged pose.
  3. The charging pile is visually detected (visual confirmation of "home").
  4. Ctrl-C / SIGTERM.

Usage:
    python scripts/return_home.py [path/to/run_feedback.jsonl]

Environment:
    PYBOT_SCOUT_GOAL_RADIUS_M    – proximity threshold to consider "arrived"
                                    (default: 0.25)
    PYBOT_SCOUT_ROTATION_SPEED   – degrees / second for heading corrections
                                    (default: 60)
    PYBOT_SCOUT_RETURN_SPEED     – translation speed in m/s (default: 0.2)
"""

import glob
import json
import math
import os
import signal
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from pybot_scout.charging_pile import ChargingPileDetector
from pybot_scout.feedback import FeedbackLogger
from pybot_scout.odometry import OdometryTracker
from pybot_scout.scout import pybot_scout

# Tunable constants (also exposed via env for quick field adjustment)
GOAL_RADIUS_M = float(os.environ.get("PYBOT_SCOUT_GOAL_RADIUS_M", "0.25"))
ROTATION_SPEED = float(os.environ.get("PYBOT_SCOUT_ROTATION_SPEED", "60"))
RETURN_SPEED = float(os.environ.get("PYBOT_SCOUT_RETURN_SPEED", "0.2"))

# Segments shorter than this are skipped to avoid micro-corrections
MIN_SEGMENT_M = 0.05

# Pause after each segment
PAUSE_SECS = 0.3

LOGGER = FeedbackLogger("return_home", output_dir=os.path.join(REPO_ROOT, "run_feedback"))
ODOM_TRACKER = OdometryTracker()
PILE_DETECTOR = ChargingPileDetector()

_shutdown_requested = False


def _signal_handler(signum, frame):
    global _shutdown_requested
    print("\nInterrupt received – stopping robot.")
    LOGGER.log("signal_received", signum=signum)
    _shutdown_requested = True
    pybot_scout.stop()


# ---------------------------------------------------------------------------
# JSONL loader
# ---------------------------------------------------------------------------

def _find_latest_feedback_file():
    """Return the most-recently-modified *.jsonl in run_feedback/."""
    pattern = os.path.join(REPO_ROOT, "run_feedback", "*.jsonl")
    candidates = glob.glob(pattern)
    if not candidates:
        return None
    return max(candidates, key=os.path.getmtime)


def _load_pose_waypoints(jsonl_path):
    """Parse a JSONL log file and return an ordered list of pose dicts.

    Extracts every step_selected (obstacle_avoidance)
    event that carries a 'pose' field.  Returns poses in chronological order
    so that reversing the list gives the return path.
    """
    waypoints = []
    with open(jsonl_path) as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except ValueError:
                continue
            event = record.get("event", "")
            if event not in ("move_step", "step_selected"):
                continue
            pose = record.get("pose")
            if pose is None:
                continue
            # Must have at least x_m and y_m
            if "x_m" not in pose or "y_m" not in pose:
                continue
            waypoints.append({
                "x_m": pose["x_m"],
                "y_m": pose["y_m"],
                "heading_deg": pose.get("heading_deg", 0.0),
                "ts": pose.get("ts"),
                "source_event": event,
                "source_ts": record.get("ts_utc"),
            })
    return waypoints


# ---------------------------------------------------------------------------
# Navigation helpers
# ---------------------------------------------------------------------------

def _normalise_angle_deg(angle_deg):
    """Wrap angle to [-180, 180]."""
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


def _distance(a, b):
    """Euclidean distance between two pose dicts."""
    return math.sqrt((a["x_m"] - b["x_m"]) ** 2 + (a["y_m"] - b["y_m"]) ** 2)


def _rotate_to_heading(target_heading_deg, current_heading_deg):
    """Rotate the robot so its heading matches target_heading_deg.

    Uses set_rotate_3 which blocks until the rotation completes.
    """
    delta = _normalise_angle_deg(target_heading_deg - current_heading_deg)
    if abs(delta) < 3.0:  # within 3 degrees – skip tiny corrections
        return

    direction = 1 if delta > 0 else 2  # 1=left(CCW), 2=right(CW) in scout convention
    degree = abs(delta)
    degree = min(degree, 360.0)
    print("  Rotating %s %.1f deg" % ("left" if direction == 1 else "right", degree))
    LOGGER.log("rotate_to_heading", delta_deg=round(delta, 1), direction=direction)
    pybot_scout.set_rotate_3(direction, int(degree))


def _drive_segment(segment_m):
    """Drive forward segment_m metres at RETURN_SPEED."""
    if segment_m < MIN_SEGMENT_M:
        return
    print("  Driving %.3f m forward" % segment_m)
    LOGGER.log("drive_segment", distance_m=round(segment_m, 3))
    pybot_scout.set_translationSpeed(RETURN_SPEED)
    pybot_scout.set_translate_3(0, segment_m)  # direction=0 = forward


# ---------------------------------------------------------------------------
# Main return-home logic
# ---------------------------------------------------------------------------

def return_home(waypoints):
    """Drive the robot backward along the recorded waypoints.

    waypoints: list of pose dicts in *chronological* order (oldest first).
    Reversal happens here so the robot heads from the last pose back to the first.
    """
    if len(waypoints) < 2:
        print("Not enough waypoints to navigate (need at least 2).")
        LOGGER.log("return_home_aborted", reason="insufficient_waypoints",
                   count=len(waypoints))
        return

    start_pose = waypoints[0]
    LOGGER.log("return_home_started",
               waypoint_count=len(waypoints),
               goal_x_m=start_pose["x_m"],
               goal_y_m=start_pose["y_m"],
               goal_radius_m=GOAL_RADIUS_M)
    print("Returning home.  Goal: (%.3f, %.3f)  radius: %.2f m" % (
        start_pose["x_m"], start_pose["y_m"], GOAL_RADIUS_M))

    pybot_scout.set_rotationSpeed(ROTATION_SPEED)
    pybot_scout.set_translationSpeed(RETURN_SPEED)

    # Walk from last waypoint back toward first
    reverse_path = list(reversed(waypoints))

    for idx in range(len(reverse_path) - 1):
        if _shutdown_requested:
            break

        current_wp = reverse_path[idx]    # where we think we are
        target_wp  = reverse_path[idx + 1]  # where we want to go next

        # Live pose for progress check
        live = ODOM_TRACKER.get_pose()
        dist_to_goal = _distance(live, start_pose)
        LOGGER.log("waypoint_progress",
                   segment=idx,
                   live_pose=live,
                   dist_to_goal_m=round(dist_to_goal, 3))

        # Early-exit: already close enough to the goal
        if dist_to_goal <= GOAL_RADIUS_M:
            print("Arrived within %.2f m of start. Stopping." % dist_to_goal)
            LOGGER.log("return_home_arrived", dist_to_goal_m=round(dist_to_goal, 3))
            break

        # Early-exit: charging pile visible
        if PILE_DETECTOR.was_recently_seen():
            sighting = PILE_DETECTOR.get_last_sighting()
            print("Charging pile detected – stopping for docking.")
            LOGGER.log("charging_pile_sighted_during_return",
                       live_pose=live, sighting=sighting)
            break

        # Compute direction to the next (earlier) waypoint in world frame
        dx = target_wp["x_m"] - current_wp["x_m"]
        dy = target_wp["y_m"] - current_wp["y_m"]
        seg_dist = math.sqrt(dx * dx + dy * dy)

        if seg_dist < MIN_SEGMENT_M:
            continue

        # Standard math convention: atan2(dy, dx) → 0=+x east, 90=+y north
        required_world_heading_deg = math.degrees(math.atan2(dy, dx))

        # Rotate robot to face the required heading using live odometry
        live_heading = live.get("heading_deg", 0.0)
        _rotate_to_heading(required_world_heading_deg, live_heading)

        # Translate forward
        _drive_segment(seg_dist)

        time.sleep(PAUSE_SECS)

    # Final check
    live = ODOM_TRACKER.get_pose()
    stats = ODOM_TRACKER.get_stats()
    dist_to_goal = _distance(live, start_pose)
    pile_seen = PILE_DETECTOR.was_recently_seen()
    LOGGER.log("return_home_finished",
               dist_to_goal_m=round(dist_to_goal, 3),
               charging_pile_detected=pile_seen,
               **stats)
    print("Return home finished.  Distance from start: %.3f m" % dist_to_goal)
    if pile_seen:
        print("Charging pile was detected.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGHUP, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    # Resolve JSONL path
    if len(sys.argv) > 1:
        jsonl_path = sys.argv[1]
    else:
        jsonl_path = _find_latest_feedback_file()
        if jsonl_path is None:
            print("No run_feedback/*.jsonl file found. "
                  "Run obstacle_avoidance.py first.")
            sys.exit(1)
        print("Using most recent feedback file: %s" % jsonl_path)

    if not os.path.isfile(jsonl_path):
        print("File not found: %s" % jsonl_path)
        sys.exit(1)

    waypoints = _load_pose_waypoints(jsonl_path)
    if not waypoints:
        print("No pose waypoints found in %s.\n"
              "The log was probably created before odometry tracking was added.\n"
              "Run obstacle_avoidance.py once more to generate "
              "a log with embedded poses." % jsonl_path)
        sys.exit(1)

    print("Loaded %d waypoints from %s" % (len(waypoints), jsonl_path))
    LOGGER.log("waypoints_loaded",
               count=len(waypoints),
               source_file=jsonl_path,
               first_pose=waypoints[0],
               last_pose=waypoints[-1])

    pybot_scout.start()
    ODOM_TRACKER.start()
    PILE_DETECTOR.start()

    try:
        return_home(waypoints)
    except Exception as exc:
        LOGGER.log("run_exception", error=str(exc), error_type=exc.__class__.__name__)
        pybot_scout.handle_exception(exc.__class__.__name__ + ": " + str(exc))

    ODOM_TRACKER.stop()
    PILE_DETECTOR.stop()
    LOGGER.log("run_stopped")
    LOGGER.close()
    pybot_scout.stop()
