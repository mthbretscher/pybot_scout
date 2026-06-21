# -*- coding: utf-8 -*-
"""
Watch for a human (person) using the robot's AI camera and play a sound each
time one is detected.

Detection uses the /CoreNode/obj topic via pybot_scout's built-in enable_reg /
recResult API.  A brief sound is played and the event is logged whenever the
camera recognises a person.

Usage:
    python scripts/human_detect.py

Environment:
    PYBOT_SCOUT_HUMAN_SOUND_ID   – effect_id passed to play_sound (1, 2 or 3;
                                    default: 1)
    PYBOT_SCOUT_HUMAN_COOLDOWN   – seconds to wait after a detection before
                                    triggering again (default: 3.0)
    PYBOT_SCOUT_HUMAN_SOUND_BLOCKING – "1" waits for aplay exit code (default: 1)
    PYBOT_SCOUT_HUMAN_SOUND_VOLUME   – optional 0..100 startup volume override
"""

import os
import signal
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from pybot_scout.feedback import FeedbackLogger
from pybot_scout.dashboard import ScriptDashboard
from pybot_scout.ros_inventory import log_ros_inventory
from pybot_scout.scout import pybot_scout, reg

POLL_INTERVAL_SECS = 0.1
SOUND_ID = int(os.environ.get("PYBOT_SCOUT_HUMAN_SOUND_ID", "1"))
COOLDOWN_SECS = float(os.environ.get("PYBOT_SCOUT_HUMAN_COOLDOWN", "3.0"))
SOUND_BLOCKING = os.environ.get("PYBOT_SCOUT_HUMAN_SOUND_BLOCKING", "1").strip().lower() not in ("0", "false", "no")
_SOUND_VOLUME_RAW = os.environ.get("PYBOT_SCOUT_HUMAN_SOUND_VOLUME", "").strip()
SOUND_VOLUME = None
if _SOUND_VOLUME_RAW:
    try:
        SOUND_VOLUME = max(0, min(100, int(_SOUND_VOLUME_RAW)))
    except ValueError:
        SOUND_VOLUME = None

LOGGER = FeedbackLogger("human_detect", output_dir=os.path.join(REPO_ROOT, "run_feedback"))
DASHBOARD = ScriptDashboard("human_detect", logger=LOGGER)


def _signal_handler(signum, frame):
    print("\nInterrupt received – stopping.")
    LOGGER.log("signal_received", signum=signum)
    DASHBOARD.close()
    pybot_scout.stop()
    LOGGER.close()
    sys.exit(0)


def start():
    DASHBOARD.start()
    log_ros_inventory(LOGGER)

    LOGGER.log(
        "human_detect_started",
        sound_id=SOUND_ID,
        cooldown_secs=COOLDOWN_SECS,
        poll_interval_secs=POLL_INTERVAL_SECS,
        sound_blocking=SOUND_BLOCKING,
        sound_volume=SOUND_VOLUME,
    )
    print("Human detection started (sound_id=%d, cooldown=%.1fs, blocking=%s). Press Ctrl-C to stop." % (
        SOUND_ID, COOLDOWN_SECS, SOUND_BLOCKING))
    DASHBOARD.update_state(
        mode="watching",
        sound_id=SOUND_ID,
        cooldown_secs=COOLDOWN_SECS,
        sound_blocking=SOUND_BLOCKING,
        sound_volume=SOUND_VOLUME,
    )
    DASHBOARD.tick(force=True)

    if SOUND_VOLUME is not None:
        try:
            pybot_scout.set_soundVolume(SOUND_VOLUME)
            LOGGER.log("sound_volume_set", volume=SOUND_VOLUME)
        except Exception as exc:
            LOGGER.log("sound_volume_failed", volume=SOUND_VOLUME, error=str(exc))

    pybot_scout.enable_reg(reg.person)

    last_triggered = -COOLDOWN_SECS  # allow immediate first trigger
    detection_count = 0

    while True:
        if pybot_scout.recResult(reg.person):
            now = time.time()
            if now - last_triggered >= COOLDOWN_SECS:
                last_triggered = now
                detection_count += 1
                print("Human detected! Triggering sound %d." % SOUND_ID)
                LOGGER.log("human_detected", sound_id=SOUND_ID)
                DASHBOARD.update_state(
                    mode="human_detected",
                    detections=detection_count,
                    since_last_secs=0.0,
                )
                DASHBOARD.tick()
                try:
                    played = pybot_scout.play_sound(SOUND_ID, SOUND_BLOCKING)
                    if not played:
                        LOGGER.log("sound_failed", sound_id=SOUND_ID, reason="play_sound_returned_false")
                        print("Sound playback command failed for sound %d." % SOUND_ID)
                        DASHBOARD.update_state(mode="sound_failed", detections=detection_count)
                    else:
                        LOGGER.log("sound_played", sound_id=SOUND_ID, blocking=SOUND_BLOCKING)
                        DASHBOARD.update_state(mode="sound_played", detections=detection_count)
                except Exception as exc:
                    LOGGER.log("sound_failed", error=str(exc))
                    print("Sound playback failed: %s" % exc)
                    DASHBOARD.update_state(mode="sound_failed", detections=detection_count)
        else:
            DASHBOARD.update_state(
                mode="watching",
                detections=detection_count,
                since_last_secs=round(max(0.0, time.time() - last_triggered), 2),
            )
        DASHBOARD.tick()
        time.sleep(POLL_INTERVAL_SECS)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGHUP, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    pybot_scout.start()
    try:
        start()
    except Exception as exc:
        LOGGER.log("run_exception", error_type=exc.__class__.__name__, error=str(exc))
        pybot_scout.handle_exception(exc.__class__.__name__ + ': ' + str(exc))

    DASHBOARD.close()
    LOGGER.close()
    pybot_scout.stop()
