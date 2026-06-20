# Agent Instructions for pybot_scout

This file is written for GitHub Copilot agents working in this repository. Read it at the start of every session.

---

## What this project is

`pybot_scout` contains Python scripts and a reusable library (`pybot_scout/`) for controlling a **roller_eye differential-drive robot** over ROS 1. The robot has:

- **Motion**: `/cmd_vel` (geometry_msgs/Twist)
- **Proximity sensors**: `/SensorNode/tof` and `/SensorNode/ibeacon` (sensor_msgs/Range)
- **AI camera / object detection**: `/CoreNode/obj` (roller_eye/detect) — used for human detection
- **Speaker**: `/SpeakerNode` with sound playback via service calls
- **IMU / odometry**: `/SensorNode/imu`, `/MotorNode/baselink_odom_relative`
- **Full ROS node list** (19 nodes): AppNode, BistNode, CloudNode, CoreNode, DetectRecordNode, MotorNode, NavPathNode, PyBotScoutBridgeNode, RTMPNode, RecorderAgentNode, S3Node, SchedNode, SensorNode, SpeakerNode, UINode, UpgraderNode, UtilNode, WiFiNode, rosout

Reusable library code lives in `pybot_scout/`; user-facing run scripts live in `scripts/`.

---

## What we've been working on

1. **`scripts/random_walk.py`** – Makes the robot explore autonomously: turn, then drive forward in short bursts.
2. **`scripts/obstacle_avoidance.py`** – Auto-discovers `sensor_msgs/Range` topics, waits for real sensor data before moving (no blind movement by default), rotates away from obstacles detected below a threshold distance.
3. **`scripts/human_detect.py`** – Subscribes to `/CoreNode/obj`, plays a configurable sound via `/SpeakerNode` each time a person is detected, with a cooldown to avoid rapid re-triggering.
4. **`scripts/runtime_probe.py`** – Samples all proximity topics for a configurable duration and logs readings. Useful for checking what sensor data is actually available on a given run.
5. **`pybot_scout/ros_inventory.py`** – Logs the full ROS graph (nodes, topics with types, services) at script startup for traceability.

---

## How iterations work

**The agent does not have direct access to the robot at runtime.** Instead:

1. **Robot runs a script** → the script writes a JSONL feedback file to `run_feedback/` (e.g. `run_feedback/20260620_204730_human_detect.jsonl`).
2. **Robot commits and pushes** the log file to this branch.
3. **Agent reads the logs** in a new session, diagnoses issues, and improves the code.
4. **Agent commits the improved code** and pushes it.
5. **Repeat.**

Every log file starts with a `logger_started` event that includes the git commit SHA and branch so logs can always be matched to the exact code version that produced them.

---

## Standard tasks for agents

When starting a session in this repo, check `run_feedback/` first:

- **If there are new log files**: read them, look for errors, anomalies, or opportunities to improve the scripts, then make targeted code changes. Summarise your findings in the commit message.
- **After reviewing logs (or if there are no new ones)**: delete all `.jsonl` files in `run_feedback/` to keep the directory clean for the next run.
- **If the user asks for cleanup only**: just delete the `.jsonl` files.

---

## Key environment variables

| Variable | Default | Purpose |
|---|---|---|
| `PYBOT_SCOUT_ALLOW_SENSORLESS_FALLBACK` | unset (disabled) | Set to `1` to allow movement without sensor data |
| `PYBOT_SCOUT_PROXIMITY_TOPICS` | auto-discover | Comma-separated list of range topic names |
| `PYBOT_SCOUT_ROS_PACKAGE` | `roller_eye` | ROS package name used by the robot vendor |
| `PYBOT_SCOUT_HUMAN_SOUND_ID` | `1` | Sound effect ID for human detection (1–3) |
| `PYBOT_SCOUT_HUMAN_COOLDOWN` | `3.0` | Seconds between human-detect sound triggers |
