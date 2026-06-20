# -*- coding: utf-8 -*-
# Example from https://github.com/Pilot-Labs-Dev/Scout-open-sourcewq

import sys
#sys.path.append("/usr/local/lib")
from rollereye import *
rollereye.start()

def start():
	rollereye.timerStart()
	rollereye.set_rotationSpeed(100)
	rollereye.set_translationSpeed(0.3)
	while rollereye.getTimerTime() <= 12000:
		rollereye.set_translate_rotate(2,270)

if __name__ == '__main__':
		try:
			start()
		except Exception as e:
			rollereye.handle_exception(e.__class__.__name__ + ': ' + e.message)
		rollereye.stop()
