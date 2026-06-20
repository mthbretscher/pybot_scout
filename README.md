# pybot_scout

Python control scripts and reusable library code for the PyBot Scout robot.

## Layout

- `pybot_scout/` – reusable robot bridge, feedback logging, and proximity discovery helpers
- `scripts/` – user-facing entrypoints you can run on the robot
- `run_feedback/` – JSONL feedback files collected from robot runs

## Scripts

Run these from the repository root:

- `python scripts/random_walk.py`
- `python scripts/obstacle_avoidance.py`
- `python scripts/runtime_probe.py`
- `python scripts/example.py`

## Obstacle avoidance

The obstacle avoidance script now:

- auto-discovers published `sensor_msgs/Range` topics
- keeps the old default topic list as a fallback
- re-checks for sensors while running if no data was available at startup
- moves in short bursts so it can stop and rotate away sooner

If your robot publishes range data on custom topic names, set:

- `PYBOT_SCOUT_PROXIMITY_TOPICS=/topic_a,/topic_b`

If the ROS package on the robot still uses the vendor name, you can leave it alone.
If it changes later, set:

- `PYBOT_SCOUT_ROS_PACKAGE=your_ros_package_name`
