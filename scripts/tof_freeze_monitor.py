# -*- coding: utf-8 -*-
"""
Standalone diagnostic monitor -- does NOT drive the robot. Purely observes and
correlates, over a long unattended window, the three things needed to find the
*systems-level* root cause of the ToF staleness (the Python/ROS layer can't be
trusted to explain it -- see Agents/instructions.md 2026-07-26 findings: not a
CPU-load issue, not generic Linux runtime-autosuspend, and a short strace
window wasn't long enough to catch the actual trigger):

  1. tof_gap        -- every time consecutive /SensorNode/tof messages are
                        more than STALL_THRESHOLD_SECS apart (wall-clock,
                        message-arrival time, NOT header stamp -- we want to
                        know when *our* subscriber actually got a new value),
                        log the gap length and the values either side of it.
  2. dmesg_event     -- polls `sudo dmesg` for growth and logs any new lines
                        matching sensor/i2c/proximity/error/timeout keywords,
                        with dmesg's own [seconds] timestamp preserved so it
                        can be lined up against tof_gap wall-clock times.
  3. network_snapshot -- every HEARTBEAT_SECS, logs established TCP
                        connections (via `sudo ss -tnp`) so a tof_gap can be
                        checked against "was the mobile app/cloud link active
                        right then" (the user's suspicion).

Also logs a periodic `heartbeat` with load average and the top-CPU process,
purely for completeness (CPU load was already ruled out live, but keep
recording it in case that changes under different conditions).

Run this on its own (no obstacle_avoidance.py needed, though it can run
alongside it) for an extended period, then inspect the resulting
run_feedback/<ts>_tof_freeze_monitor.jsonl for tof_gap events and check what
dmesg_event / network_snapshot entries fall in the same window.

Usage:
    source /opt/ros/melodic/setup.bash
    python2 scripts/tof_freeze_monitor.py
    (Ctrl-C to stop)
"""

import os
import re
import subprocess
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import rospy
from sensor_msgs.msg import Range

from pybot_scout.feedback import FeedbackLogger

STALL_THRESHOLD_SECS = 1.0
HEARTBEAT_SECS = 15.0
DMESG_POLL_SECS = 2.0
DMESG_KEYWORDS = re.compile(
    r"i2c|vl53|proximity|sensor_active|stmvl53l0x|timeout|-110|-121|remote i/o|nak",
    re.IGNORECASE,
)

FEEDBACK_DIR = os.path.join(REPO_ROOT, "run_feedback")
LOGGER = FeedbackLogger("tof_freeze_monitor", output_dir=FEEDBACK_DIR)

_state = {
    "last_tof_wall": None,
    "last_tof_range": None,
    "last_tof_seq": None,
    "msgs_since_heartbeat": 0,
    "dmesg_line_count": 0,
}


def _tof_cb(msg):
    now = time.time()
    _state["msgs_since_heartbeat"] += 1
    prev_wall = _state["last_tof_wall"]
    if prev_wall is not None:
        gap = now - prev_wall
        if gap > STALL_THRESHOLD_SECS:
            LOGGER.log(
                "tof_gap",
                gap_secs=round(gap, 3),
                prev_range=_state["last_tof_range"],
                prev_seq=_state["last_tof_seq"],
                new_range=msg.range,
                new_seq=msg.header.seq,
            )
            print("tof_gap: %.2fs (prev=%.3f seq=%s -> new=%.3f seq=%s)" % (
                gap, _state["last_tof_range"] or -1, _state["last_tof_seq"],
                msg.range, msg.header.seq))
    _state["last_tof_wall"] = now
    _state["last_tof_range"] = msg.range
    _state["last_tof_seq"] = msg.header.seq


def _dmesg_lines():
    try:
        out = subprocess.check_output(["sudo", "-n", "dmesg"], stderr=subprocess.STDOUT)
    except Exception as exc:
        return None, str(exc)
    return out.decode(errors="replace").splitlines(), None


def _poll_dmesg():
    lines, err = _dmesg_lines()
    if lines is None:
        LOGGER.log("dmesg_poll_failed", error=err)
        return
    if _state["dmesg_line_count"] == 0:
        # First poll: just record the baseline, don't flood the log with
        # everything that happened before this monitor started.
        _state["dmesg_line_count"] = len(lines)
        return
    if len(lines) < _state["dmesg_line_count"]:
        # Buffer wrapped/rotated; reset baseline.
        _state["dmesg_line_count"] = len(lines)
        return
    new_lines = lines[_state["dmesg_line_count"]:]
    _state["dmesg_line_count"] = len(lines)
    for line in new_lines:
        if DMESG_KEYWORDS.search(line):
            LOGGER.log("dmesg_event", line=line.strip())
            print("dmesg_event: %s" % line.strip())


def _is_external_peer(line):
    """True if the remote (peer) address in an `ss -tnp` ESTAB line is not a
    loopback address. This system uses *both* 127.0.0.1 and 127.0.1.1 for
    internal inter-node ROS traffic, so both must be excluded -- otherwise
    every snapshot is flooded with ~40+ irrelevant ROS peer connections and a
    real external (mobile app / cloud) connection is impossible to spot.
    """
    fields = line.split()
    if len(fields) < 5:
        return False
    peer = fields[4]  # "State Recv-Q Send-Q Local:Port Peer:Port ..."
    return not peer.startswith("127.")


def _network_snapshot():
    try:
        out = subprocess.check_output(["sudo", "-n", "ss", "-tnp"], stderr=subprocess.STDOUT)
        out = out.decode(errors="replace")
    except Exception as exc:
        LOGGER.log("network_snapshot_failed", error=str(exc))
        return
    all_estab = [l for l in out.splitlines() if l.startswith("ESTAB")]
    external = [l for l in all_estab if _is_external_peer(l)]
    LOGGER.log("network_snapshot", total_established=len(all_estab),
               external_count=len(external), external_lines=external)


def _tof_subscriber_count():
    """Number of subscribers currently attached to /SensorNode/tof, via
    `rostopic info` -- if the mobile app attaches its own consumer while
    driving (rather than only going through our own script), this count
    should visibly change.
    """
    try:
        out = subprocess.check_output(["rostopic", "info", "/SensorNode/tof"],
                                       stderr=subprocess.STDOUT).decode(errors="replace")
    except Exception:
        return None
    in_subs = False
    count = 0
    for line in out.splitlines():
        if line.startswith("Subscribers:"):
            in_subs = True
            continue
        if line.startswith("Publishers:"):
            in_subs = False
            continue
        if in_subs and line.strip().startswith("*"):
            count += 1
    return count


def _heartbeat():
    try:
        load1, load5, load15 = os.getloadavg()
    except Exception:
        load1 = load5 = load15 = None
    top_proc = None
    try:
        ps_out = subprocess.check_output(
            ["ps", "-eo", "pid,%cpu,comm", "--sort=-%cpu"]).decode(errors="replace")
        top_proc = ps_out.splitlines()[1].strip() if len(ps_out.splitlines()) > 1 else None
    except Exception:
        pass
    tof_subs = _tof_subscriber_count()
    LOGGER.log(
        "heartbeat",
        load1=load1, load5=load5, load15=load15,
        tof_msgs_since_last=_state["msgs_since_heartbeat"],
        top_proc=top_proc,
        tof_subscriber_count=tof_subs,
    )
    print("heartbeat: load=%.2f tof_msgs=%d top=%s tof_subs=%s" % (
        load1 or -1, _state["msgs_since_heartbeat"], top_proc, tof_subs))
    _state["msgs_since_heartbeat"] = 0


def main():
    rospy.init_node("tof_freeze_monitor", anonymous=True)
    rospy.Subscriber("/SensorNode/tof", Range, _tof_cb)
    print("tof_freeze_monitor started -- logging to %s" % LOGGER.path)
    LOGGER.log("monitor_started", stall_threshold_secs=STALL_THRESHOLD_SECS,
               heartbeat_secs=HEARTBEAT_SECS, dmesg_poll_secs=DMESG_POLL_SECS)

    _poll_dmesg()  # establish baseline without flooding

    next_dmesg = time.time() + DMESG_POLL_SECS
    next_heartbeat = time.time() + HEARTBEAT_SECS
    next_network = time.time() + HEARTBEAT_SECS

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        now = time.time()
        if now >= next_dmesg:
            next_dmesg = now + DMESG_POLL_SECS
            _poll_dmesg()
        if now >= next_heartbeat:
            next_heartbeat = now + HEARTBEAT_SECS
            _heartbeat()
        if now >= next_network:
            next_network = now + HEARTBEAT_SECS
            _network_snapshot()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
