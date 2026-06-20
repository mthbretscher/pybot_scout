# -*- coding: utf-8 -*-

import os
import random
import signal
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from pybot_scout.feedback import FeedbackLogger
from pybot_scout.scout import pybot_scout

SPEED = 0.3
ROTATION_SPEED = 90
MIN_STEP_SECS = 2
MAX_STEP_SECS = 5
PAUSE_SECS = 0.5

LOGGER = FeedbackLogger("random_walk")


def _signal_handler(signum, frame):
    print("\nInterrupt received – stopping robot.")
    LOGGER.log("signal_received", signum=signum)
    pybot_scout.stop()


def start():
    pybot_scout.set_rotationSpeed(ROTATION_SPEED)
    pybot_scout.set_translationSpeed(SPEED)
    LOGGER.log(
        "run_started",
        speed=SPEED,
        rotation_speed=ROTATION_SPEED,
        min_step_secs=MIN_STEP_SECS,
        max_step_secs=MAX_STEP_SECS,
        pause_secs=PAUSE_SECS,
    )

    print("Random walk started. Press Ctrl-C to stop.")

    step_index = 0
    while True:
        step_index += 1
        direction = random.randint(0, 360)
        duration = random.uniform(MIN_STEP_SECS, MAX_STEP_SECS)

        print("Moving direction=%d deg for %.1f s" % (direction, duration))
        pybot_scout.set_translate_2(direction, duration)
        LOGGER.log(
            "move_step",
            step_index=step_index,
            direction_deg=direction,
            requested_duration_s=duration,
            actual_duration_s=duration,
        )

        time.sleep(PAUSE_SECS)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGHUP, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    pybot_scout.start()

    try:
        start()
    except Exception as exc:
        LOGGER.log("run_exception", error=str(exc), error_type=exc.__class__.__name__)
        pybot_scout.handle_exception(exc.__class__.__name__ + ': ' + str(exc))

    LOGGER.log("run_stopped")
    LOGGER.close()
    pybot_scout.stop()
