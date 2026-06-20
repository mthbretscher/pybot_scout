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
MIN_TURN_DEG = 45
MAX_TURN_DEG = 180

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
        turn_direction = random.choice([1, 2])
        turn_degree = random.randint(MIN_TURN_DEG, MAX_TURN_DEG)
        duration = random.uniform(MIN_STEP_SECS, MAX_STEP_SECS)

        print("Turning dir=%d by %d deg" % (turn_direction, turn_degree))
        pybot_scout.set_rotate_3(turn_direction, turn_degree)
        LOGGER.log(
            "turn_step",
            step_index=step_index,
            turn_direction=turn_direction,
            turn_degree=turn_degree,
        )

        print("Moving forward for %.1f s" % duration)
        pybot_scout.set_translate_2(0, duration)
        LOGGER.log(
            "move_step",
            step_index=step_index,
            direction_deg=0,
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
