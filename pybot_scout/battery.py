# -*- coding: utf-8 -*-
"""
Battery monitor for the roller_eye robot.

Subscribes to /SensorNode/simple_battery_status (roller_eye/status) and
exposes the current battery percentage and charging state.

Because the exact field layout of roller_eye/status is not fully documented,
the first received message is logged in full (all attribute names and values)
so that subsequent sessions can confirm which fields to read.

Usage::

    monitor = BatteryMonitor()
    monitor.start(logger=LOGGER)          # logger is optional
    pct, charging = monitor.wait_for_status(timeout_secs=5.0)
    # pct   – float 0-100, or None if not determinable
    # charging – bool, or None if not determinable

    monitor.get_percent()    # latest known percentage or None
    monitor.is_charging()    # latest known charging flag or None
    monitor.stop()
"""

import re
import threading
import time

BATTERY_STATUS_TOPIC = "/SensorNode/simple_battery_status"

# Field names to probe (in priority order) when looking for a numeric percentage.
_PERCENT_FIELDS = ("percentage", "percent", "battery_level", "level", "capacity")


def _parse_battery_msg(msg):
    """Return (percent_or_None, charging_or_None) from a roller_eye/status message.

    Tries a list of field names for percentage, then falls back to parsing the
    'name' string.  Charging flag is derived from the 'name' or 'state' field.
    """
    percent = None
    charging = None

    # ── percentage ──────────────────────────────────────────────────────────
    for attr in _PERCENT_FIELDS:
        val = getattr(msg, attr, None)
        if val is None:
            continue
        if isinstance(val, (int, float)) and 0.0 <= float(val) <= 100.0:
            percent = float(val)
            break
        if isinstance(val, str):
            m = re.search(r"(\d+(?:\.\d+)?)", val)
            if m:
                v = float(m.group(1))
                if 0.0 <= v <= 100.0:
                    percent = v
                    break

    # ── charging flag ────────────────────────────────────────────────────────
    for attr in ("name", "state", "status_str"):
        val = getattr(msg, attr, None)
        if val is not None:
            text = str(val).lower()
            if "charg" in text:
                charging = True
                break
            if "discharg" in text or "full" in text or "idle" in text:
                charging = False
                break

    # ── fall back: roller_eye/status carries status=[state, pct, extra] ──────
    # Observed layout: status[0] = state (0=CHARGING, 1=UNCHARGE, 2=FULL, 3=UNKNOWN)
    #                  status[1] = battery percentage 0-100
    status_list = getattr(msg, "status", None)
    if isinstance(status_list, (list, tuple)) and len(status_list) >= 2:
        if percent is None:
            pct_val = status_list[1]
            if isinstance(pct_val, (int, float)) and 0.0 <= float(pct_val) <= 100.0:
                percent = float(pct_val)
        if charging is None:
            state_code = status_list[0]
            if state_code in (0, 2):   # 0=CHARGING, 2=FULL (still docked)
                charging = True
            elif state_code == 1:       # 1=UNCHARGE (discharging)
                charging = False
    elif charging is None:
        # Scalar status/data fallback (other message types)
        for attr in ("data",):
            val = getattr(msg, attr, None)
            if isinstance(val, int):
                charging = val in (1, 2)
                break

    return percent, charging


def _msg_attrs(msg):
    """Return a dict of all public (non-dunder) scalar attrs on a ROS message."""
    result = {}
    for attr in dir(msg):
        if attr.startswith("_"):
            continue
        try:
            val = getattr(msg, attr)
            if callable(val):
                continue
            result[attr] = val
        except Exception:
            pass
    return result


class BatteryMonitor(object):
    """Thread-safe battery level and charging-state monitor.

    Reads /SensorNode/simple_battery_status.  All public methods may be called
    from any thread after start() has been called.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._percent = None   # float 0-100, or None
        self._charging = None  # bool, or None
        self._msg_ts = None    # time.time() of last message
        self._first_logged = False
        self._logger = None
        self._sub = None

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def start(self, logger=None):
        """Subscribe to the battery status topic.

        *logger* is an optional :class:`pybot_scout.feedback.FeedbackLogger`
        instance used to record the first raw message structure.
        """
        self._logger = logger
        try:
            import os
            import rospy
            ros_pkg = os.environ.get("PYBOT_SCOUT_ROS_PACKAGE", "roller_eye")
            _ros_msg = __import__(ros_pkg + ".msg", fromlist=["*"])
            status_cls = getattr(_ros_msg, "status")

            def _cb(msg):
                pct, chg = _parse_battery_msg(msg)
                ts = time.time()
                with self._lock:
                    self._percent = pct
                    self._charging = chg
                    self._msg_ts = ts
                    first = not self._first_logged
                    self._first_logged = True

                if first and self._logger is not None:
                    # Log raw attributes so we can confirm the field layout.
                    self._logger.log(
                        "battery_msg_schema",
                        topic=BATTERY_STATUS_TOPIC,
                        attrs=_msg_attrs(msg),
                        parsed_percent=pct,
                        parsed_charging=chg,
                    )

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

    # ── queries ───────────────────────────────────────────────────────────────

    def get_percent(self):
        """Return the most recently parsed battery percentage (0–100), or None."""
        with self._lock:
            return self._percent

    def is_charging(self):
        """Return True if the robot is currently charging, False if not, None if unknown."""
        with self._lock:
            return self._charging

    def has_data(self):
        """Return True once at least one battery message has been received."""
        with self._lock:
            return self._msg_ts is not None

    def wait_for_status(self, timeout_secs=5.0):
        """Block until a battery message arrives or *timeout_secs* elapses.

        Returns (percent_or_None, charging_or_None).
        """
        deadline = time.time() + timeout_secs
        while time.time() < deadline:
            with self._lock:
                if self._msg_ts is not None:
                    return self._percent, self._charging
            time.sleep(0.1)
        return None, None
