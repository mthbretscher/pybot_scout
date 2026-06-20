# -*- coding: utf-8 -*-
# random_walk.py
#
# Makes the Moorebot Scout drive randomly through an apartment.
# The robot picks a random direction, drives for a random duration,
# pauses briefly, then repeats indefinitely.
#
# Usage:
#   python random_walk.py
#
# Press Ctrl-C (or send SIGTERM) to stop the robot cleanly.

import signal
import random
import time

from rollereye import rollereye

# ---------------------------------------------------------------------------
# Tuneable constants
# ---------------------------------------------------------------------------
SPEED = 0.3          # translation speed in m/s (0 – 1)
ROTATION_SPEED = 90  # rotation speed in degree/s used by set_rotationSpeed
MIN_STEP_SECS = 2    # minimum time to drive in one direction (seconds)
MAX_STEP_SECS = 5    # maximum time to drive in one direction (seconds)
PAUSE_SECS = 0.5     # pause between steps (seconds)
# ---------------------------------------------------------------------------


def _signal_handler(signum, frame):
    print("\nInterrupt received – stopping robot.")
    rollereye.stop()


def start():
    rollereye.set_rotationSpeed(ROTATION_SPEED)
    rollereye.set_translationSpeed(SPEED)

    print("Random walk started. Press Ctrl-C to stop.")

    while True:
        direction = random.randint(0, 360)
        duration = random.uniform(MIN_STEP_SECS, MAX_STEP_SECS)

        print("Moving direction=%d deg for %.1f s" % (direction, duration))
        rollereye.set_translate_2(direction, int(duration))

        time.sleep(PAUSE_SECS)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGHUP, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    rollereye.start()

    try:
        start()
    except Exception as e:
        rollereye.handle_exception(e.__class__.__name__ + ': ' + str(e))

    rollereye.stop()
