# -*- coding: utf-8 -*-
"""
Charging pile (home anchor) detector.

Subscribes to /CoreNode/chargingPile (roller_eye/detect) in a background
thread and caches the most recent sighting, so that scripts can ask
"was the charger seen recently?".

Usage::

    detector = ChargingPileDetector()
    detector.start()

    if detector.was_recently_seen():
        print("charger visible!")

    sighting = detector.get_last_sighting()  # dict or None
    detector.stop()
"""

import os
import threading
import time

CHARGING_PILE_TOPIC = "/CoreNode/chargingPile"


class ChargingPileDetector(object):
    """Detect the charging pile via the robot's built-in AI camera pipeline.

    Thread-safe.  All public methods may be called from any thread.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._last_seen_ts = None
        self._last_seen_data = None
        self._sub = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self):
        """Subscribe to the /CoreNode/chargingPile detection topic."""
        try:
            import rospy
            ros_pkg = os.environ.get("PYBOT_SCOUT_ROS_PACKAGE", "roller_eye")
            _ros_msg = __import__(ros_pkg + ".msg", fromlist=["*"])
            detect_cls = getattr(_ros_msg, "detect")

            def _cb(msg):
                ts = time.time()
                name = getattr(msg, "name", "")
                with self._lock:
                    self._last_seen_ts = ts
                    self._last_seen_data = {"name": name, "ts": ts}

            self._sub = rospy.Subscriber(CHARGING_PILE_TOPIC, detect_cls, _cb)
        except Exception:
            # ROS or roller_eye message type unavailable; detector still safe to call
            pass

    def stop(self):
        """Unsubscribe from the charging pile topic."""
        if self._sub is not None:
            try:
                self._sub.unregister()
            except Exception:
                pass
            self._sub = None

    def was_recently_seen(self, within_secs=2.0):
        """Return True if the charging pile was detected within the last *within_secs* seconds."""
        with self._lock:
            if self._last_seen_ts is None:
                return False
            return (time.time() - self._last_seen_ts) <= within_secs

    def get_last_sighting(self):
        """Return the last sighting as a dict, or None if never detected."""
        with self._lock:
            return dict(self._last_seen_data) if self._last_seen_data else None
