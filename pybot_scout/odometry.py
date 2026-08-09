# -*- coding: utf-8 -*-
"""
Lightweight breadcrumb tracker using wheel odometry.

Subscribes to /MotorNode/baselink_odom_relative (nav_msgs/Odometry) in a
background thread, caches the latest pose, and samples breadcrumbs at a
configurable interval into an in-memory ring buffer.

Usage::

    tracker = OdometryTracker()
    tracker.start()

    pose  = tracker.get_pose()        # latest {x_m, y_m, heading_deg, ts}
    crumbs = tracker.get_breadcrumbs() # list of sampled poses, oldest first
    stats  = tracker.get_stats()       # {total_distance_m, avg_speed_mps, ...}

    tracker.stop()
"""

import math
import threading
import time
from collections import deque

ODOM_TOPIC = "/MotorNode/baselink_odom_relative"
DEFAULT_SAMPLE_INTERVAL_SECS = 0.5
DEFAULT_MAX_CRUMBS = 500


def _quat_to_heading_deg(qx, qy, qz, qw):
    """Convert a quaternion to a yaw/heading in degrees.

    For planar (2-D) robot motion the rotation is purely about the z-axis, so
    the yaw can be extracted with the standard formula:
        yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy² + qz²))
    """
    yaw_rad = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    return math.degrees(yaw_rad)


class OdometryTracker(object):
    """Subscribe to wheel odometry and maintain a breadcrumb trail.

    Thread-safe.  All public methods may be called from any thread.
    """

    def __init__(
        self,
        sample_interval_secs=DEFAULT_SAMPLE_INTERVAL_SECS,
        max_crumbs=DEFAULT_MAX_CRUMBS,
    ):
        self._sample_interval = sample_interval_secs
        self._max_crumbs = max_crumbs

        self._lock = threading.Lock()
        self._crumbs = deque(maxlen=max_crumbs)

        # Latest pose from the ROS callback
        self._latest_x = 0.0
        self._latest_y = 0.0
        self._latest_heading_deg = 0.0
        self._latest_ts = None
        self._has_data = False

        # Accumulated stats
        self._total_distance_m = 0.0
        self._start_time = None

        self._sub = None
        self._sampler_thread = None
        self._running = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self):
        """Subscribe to the odometry topic and begin background sampling."""
        try:
            import rospy
            from nav_msgs.msg import Odometry

            def _odom_cb(msg):
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                with self._lock:
                    self._latest_x = p.x
                    self._latest_y = p.y
                    self._latest_heading_deg = _quat_to_heading_deg(
                        q.x, q.y, q.z, q.w
                    )
                    self._latest_ts = time.time()
                    self._has_data = True

            self._sub = rospy.Subscriber(ODOM_TOPIC, Odometry, _odom_cb)
        except Exception:
            # rospy not available (unit-testing, offline runs, etc.)
            pass

        self._running = True
        self._start_time = time.time()
        self._sampler_thread = threading.Thread(
            target=self._sample_loop, name="OdometrySampler"
        )
        self._sampler_thread.daemon = True
        self._sampler_thread.start()

    def stop(self):
        """Stop sampling and unsubscribe from the odometry topic."""
        self._running = False
        if self._sub is not None:
            try:
                self._sub.unregister()
            except Exception:
                pass
            self._sub = None

    def get_pose(self):
        """Return the latest odometry pose.

        Returns:
            dict with keys: x_m, y_m, heading_deg, ts (epoch float or None),
            has_data (bool – False until the first message arrives)
        """
        with self._lock:
            return {
                "x_m": self._latest_x,
                "y_m": self._latest_y,
                "heading_deg": self._latest_heading_deg,
                "ts": self._latest_ts,
                "has_data": self._has_data,
            }

    def get_breadcrumbs(self):
        """Return a list of sampled pose dicts, oldest first.

        Each entry: {x_m, y_m, heading_deg, ts}
        """
        with self._lock:
            return list(self._crumbs)

    def get_stats(self):
        """Return movement statistics accumulated since start().

        Returns:
            dict with keys: total_distance_m, avg_speed_mps, elapsed_secs,
            breadcrumb_count
        """
        with self._lock:
            elapsed = (time.time() - self._start_time) if self._start_time else 0.0
            avg_speed = (self._total_distance_m / elapsed) if elapsed > 0 else 0.0
            return {
                "total_distance_m": round(self._total_distance_m, 3),
                "avg_speed_mps": round(avg_speed, 4),
                "elapsed_secs": round(elapsed, 1),
                "breadcrumb_count": len(self._crumbs),
            }

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _sample_loop(self):
        """Background thread: sample pose at self._sample_interval seconds."""
        prev_x = None
        prev_y = None

        while self._running:
            time.sleep(self._sample_interval)

            with self._lock:
                if not self._has_data:
                    continue
                x = self._latest_x
                y = self._latest_y
                heading = self._latest_heading_deg
                ts = self._latest_ts

            if prev_x is not None:
                dist = math.sqrt((x - prev_x) ** 2 + (y - prev_y) ** 2)
                with self._lock:
                    self._total_distance_m += dist

            prev_x = x
            prev_y = y

            crumb = {
                "x_m": round(x, 4),
                "y_m": round(y, 4),
                "heading_deg": round(heading, 2),
                "ts": ts,
            }
            with self._lock:
                self._crumbs.append(crumb)
