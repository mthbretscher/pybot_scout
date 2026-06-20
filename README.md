# pybot_scout

Python control scripts and reusable library code for the PyBot Scout robot.

## Layout

- `pybot_scout/` – reusable robot bridge, feedback logging, proximity discovery, and ROS inventory helpers
- `scripts/` – user-facing entrypoints you can run on the robot
- `run_feedback/` – JSONL feedback files collected from robot runs

## Scripts

Run these from the repository root:

- `python scripts/random_walk.py`
- `python scripts/obstacle_avoidance.py`
- `python scripts/runtime_probe.py`
- `python scripts/human_detect.py`
- `python scripts/example.py`

`random_walk.py` now turns first, then drives forward each step, which gives a
more exploratory path on differential-drive robots.

## Feedback logs

Every JSONL log file starts with a `logger_started` event that includes the
**git commit SHA**, **branch**, and whether the working tree was **dirty** at
run time.  This makes it easy to match a log file to the exact code version
that produced it, even if old logs are kept around for comparison.

```json
{"event": "logger_started", "git_branch": "main", "git_commit": "a1b2c3d...", "git_dirty": false, ...}
```

## Human detection

`scripts/human_detect.py` subscribes to the robot's AI camera (`/CoreNode/obj`)
and plays a sound each time a person is recognised.  Adjust with environment
variables:

- `PYBOT_SCOUT_HUMAN_SOUND_ID` – sound effect to play (1, 2 or 3; default: 1)
- `PYBOT_SCOUT_HUMAN_COOLDOWN` – seconds between triggers (default: 3.0)

If audio playback fails (missing wave file or `aplay` execution error), the
script logs `sound_failed` events in `run_feedback/*.jsonl`.

## ROS inventory

`runtime_probe.py` (and `human_detect.py`) call `log_ros_inventory()` at
startup.  This logs the complete ROS graph — every **node**, **topic** (with
message type), and **service** — as structured events in the feedback file:

```
ros_inventory_nodes    – list of all known node names
ros_inventory_topics   – list of {topic, topic_type} objects
ros_inventory_services – list of all service names
```

## Obstacle avoidance

The obstacle avoidance script:

- auto-discovers published `sensor_msgs/Range` topics
- includes `/SensorNode/tof` and `/SensorNode/ibeacon` in default candidates
- keeps the old default topic list as a fallback
- re-checks for sensors while running if no data was available at startup
- moves in short bursts so it can stop and rotate away sooner

By default, if no valid proximity data is available, movement is paused until
sensor data appears. To force the old sensor-less fallback behavior, set:

- `PYBOT_SCOUT_ALLOW_SENSORLESS_FALLBACK=1`

If your robot publishes range data on custom topic names, set:

- `PYBOT_SCOUT_PROXIMITY_TOPICS=/topic_a,/topic_b`

If the ROS package on the robot still uses the vendor name, you can leave it alone.
If it changes later, set:

- `PYBOT_SCOUT_ROS_PACKAGE=your_ros_package_name`
