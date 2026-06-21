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
from pybot_scout.ros_inventory import log_ros_inventory
from pybot_scout.scout import pybot_scout, reg

POLL_INTERVAL_SECS = 0.1
SOUND_ID = int(os.environ.get("PYBOT_SCOUT_HUMAN_SOUND_ID", "1"))
COOLDOWN_SECS = float(os.environ.get("PYBOT_SCOUT_HUMAN_COOLDOWN", "3.0"))

LOGGER = FeedbackLogger("human_detect", output_dir=os.path.join(REPO_ROOT, "run_feedback"))


def _signal_handler(signum, frame):
    print("\nInterrupt received – stopping.")
    LOGGER.log("signal_received", signum=signum)
    pybot_scout.stop()
    LOGGER.close()
    sys.exit(0)


def start():
    log_ros_inventory(LOGGER)

    LOGGER.log(
        "human_detect_started",
        sound_id=SOUND_ID,
        cooldown_secs=COOLDOWN_SECS,
        poll_interval_secs=POLL_INTERVAL_SECS,
    )
    print("Human detection started (sound_id=%d, cooldown=%.1fs). Press Ctrl-C to stop." % (
        SOUND_ID, COOLDOWN_SECS))

    pybot_scout.enable_reg(reg.person)

    last_triggered = -COOLDOWN_SECS  # allow immediate first trigger

    while True:
        if pybot_scout.recResult(reg.person):
            now = time.time()
            if now - last_triggered >= COOLDOWN_SECS:
                last_triggered = now
                print("Human detected! Playing sound %d." % SOUND_ID)
                LOGGER.log("human_detected", sound_id=SOUND_ID)
                try:
                    played = pybot_scout.play_sound(SOUND_ID, False)
                    if not played:
                        LOGGER.log("sound_failed", sound_id=SOUND_ID, reason="play_sound_returned_false")
                except Exception as exc:
                    LOGGER.log("sound_failed", error=str(exc))
                    print("Sound playback failed: %s" % exc)
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

    LOGGER.close()
    pybot_scout.stop()
