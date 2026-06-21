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


BATTERY_STATUS_TOPIC = "/SensorNode/simple_battery_status"


class ChargingStatusDetector(object):
    """Detect whether the robot is currently sitting on its charging station.

    Subscribes to /SensorNode/simple_battery_status (roller_eye/status) and
    inspects the message to determine charging state.  Thread-safe.

    Usage::

        detector = ChargingStatusDetector()
        detector.start()
        if detector.wait_for_status(timeout_secs=3.0):
            print("on charger!")
        detector.stop()
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._charging = None   # None = no data, True/False once a message arrives
        self._msg_ts = None
        self._sub = None

    def start(self):
        """Subscribe to the battery status topic."""
        try:
            import rospy
            ros_pkg = os.environ.get("PYBOT_SCOUT_ROS_PACKAGE", "roller_eye")
            _ros_msg = __import__(ros_pkg + ".msg", fromlist=["*"])
            status_cls = getattr(_ros_msg, "status")

            def _cb(msg):
                # roller_eye/status carries state in status=[state_code, pct, extra].
                # state_code: 0=CHARGING, 1=UNCHARGE, 2=FULL, 3=UNKNOWN
                charging = None
                status_list = getattr(msg, "status", None)
                if isinstance(status_list, (list, tuple)) and len(status_list) >= 1:
                    state_code = status_list[0]
                    if state_code in (0, 2):   # 0=CHARGING, 2=FULL (still docked)
                        charging = True
                    elif state_code == 1:       # 1=UNCHARGE (discharging)
                        charging = False
                if charging is None:
                    # Fallback: look for "charg" text in any string field
                    for attr in ("name", "state", "data"):
                        val = getattr(msg, attr, None)
                        if val is not None:
                            charging = "charg" in str(val).lower()
                            break
                if charging is None:
                    charging = False
                with self._lock:
                    self._charging = charging
                    self._msg_ts = time.time()

            self._sub = rospy.Subscriber(BATTERY_STATUS_TOPIC, status_cls, _cb)
        except Exception:
            pass

    def stop(self):
        """Unsubscribe from the battery status topic."""
        if self._sub is not None:
            try:
                self._sub.unregister()
            except Exception:
                pass
            self._sub = None

    def wait_for_status(self, timeout_secs=3.0):
        """Block up to *timeout_secs* for a battery message; return True if charging.

        Returns None if no message was received within the timeout.
        """
        deadline = time.time() + timeout_secs
        while time.time() < deadline:
            with self._lock:
                if self._msg_ts is not None:
                    return self._charging
            time.sleep(0.1)
        return None
