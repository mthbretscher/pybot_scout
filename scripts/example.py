# -*- coding: utf-8 -*-

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from pybot_scout.scout import pybot_scout

pybot_scout.start()

def start():
    pybot_scout.timerStart()
    pybot_scout.set_rotationSpeed(100)
    pybot_scout.set_translationSpeed(0.3)
    while pybot_scout.getTimerTime() <= 12000:
        pybot_scout.set_translate_rotate(2, 270)

if __name__ == '__main__':
    try:
        start()
    except Exception as exc:
        pybot_scout.handle_exception(exc.__class__.__name__ + ': ' + str(exc))
    pybot_scout.stop()
