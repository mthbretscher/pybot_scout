# Agent Instructions for pybot_scout

This file is written for GitHub Copilot agents working in this repository. Read it at the start of every session.

---

## What this project is

`pybot_scout` contains Python scripts and a reusable library (`pybot_scout/`) for controlling a **Moorebot Scout** — commercial codename **"roller_eye"** — over ROS 1 Melodic. The vendor firmware is closed-source at the app/cloud layer but the ROS-side `roller_eye` catkin package (nodes, messages, services, and a Python scripting SDK) is officially open-sourced. `pybot_scout/scout.py` and `pybot_scout/ros_bridge.py` are a refactored port of that vendor SDK (see "On-device vendor SDK" below) — same algorithms, cleaner packaging.

**Drivetrain — important correction:** this is *not* a simple two-wheel differential-drive robot. `/cmd_vel` (`geometry_msgs/Twist`) is driven with both `linear.x` **and** `linear.y` (see `direction_2_x_y()` in the SDK), i.e. the base accepts arbitrary-direction (0–360°) translation independent of heading — holonomic/omni-style strafing on a **4-wheel drive** chassis (matches vendor marketing "four-wheel drive"). The Python SDK also exposes a `set_wheel(frontLeft, frontRight, rearLeft, rearRight)` per-wheel RPM hook, though in the shipped SDK build it is an unimplemented stub (see gotchas below) — treat `/cmd_vel` Twist as the reliable control surface, not per-wheel RPM.

Reusable library code lives in `pybot_scout/`; user-facing run scripts live in `scripts/`.

---

## Keep this file up to date (continuous, every session — MANDATORY, not optional polish)

This file is the first thing an agent reads each session, so a stale fact here costs the *next* session real debugging time. Whenever a change you make alters behavior, a workaround, a gotcha, or the status of a script described in this file:
1. Before considering a behavior-changing edit "done", check whether this file still describes the *old* behavior — if so, update it as part of the same edit, not as an afterthought.
2. Update **both** `/memories/repo/*.md` (detailed chronological changelog) **and** this file (current-state summary) — they serve different purposes; don't update one and skip the other.
3. Do a final pass immediately before replying to the user with a summary of completed work, specifically re-reading any section your change could have made wrong.
4. **After editing this file, verify the write actually landed with a direct `ssh ... grep/sed` check** (not just a subsequent `read_file` over the SSH FS mount) — writes to this file over the mounted filesystem have been observed to report success and even read back correctly immediately after, yet silently fail to persist to the real remote file (confirmed 2026-07-26). Treat `read_file` over the mount as unverified for this file until double-checked with plain `ssh ... grep`.

---

## Authoritative sources (read these before guessing)

**Official (Moorebot / Pilot Laboratories, Inc.):**
- Open-source ROS firmware: **https://github.com/Pilot-Labs-Dev/Scout-open-source** (MIT license, C++/C, 154★). Contains the full `roller_eye` catkin package source (`roller_eye/src/nodes/*.cpp`, `roller_eye/msg/*.msg`, `roller_eye/srv/*.srv`, `roller_eye/lib/rollereye.py`, `roller_eye/lib/rollereye_ros_bridge.py`) — this is the exact source the binaries on this robot were built from (ROS distro, message/service names, and function signatures all match what we verified live on-device). README covers build steps (Ubuntu 18.04 container at `https://github.com/Pilot-Labs-Dev/binary.git`), sample snippets (video/audio/light/ToF/IMU topics), a full Python scripting example, and **ROOT access instructions** (tap firmware version 5×, password `123456` — Android app only as of the README).
- VIO (visual-inertial odometry / monocular SLAM) source: **https://github.com/Pilot-Labs-Dev/vio**
- Product/user manual (PDF, v6.0, English + other languages) and 3D-print files (mounts, tracks, bumper): **https://www.moorebot.com/pages/download**
- Product marketing/spec page: **https://www.moorebot.com/pages/moorebot-scout**

**Community (unofficial, useful for practical examples — verify against official source before trusting blindly):**
- `https://github.com/ehonis/school-moorebot-scout` — student project exploring the `roller_eye` package, includes a copy of `rollereye_ros_bridge.py` with commentary.
- `https://github.com/LeiShi1313/scout-controller` — Go controller with gamepad support + video display.
- `https://github.com/GINNOV/Scout-Controller` — utility to drive Scout from a Mac-hosted ROS environment.
- `https://github.com/szmlb/scout_bridge` — ROS 2 bridge that talks to Scout over a rosbridge WebSocket and remaps velocity axes.
- Reddit: `r/moorebotscout` (community hub), and threads on debug-USB/root access — `https://www.reddit.com/r/moorebotscout/comments/12812yg/` and `https://www.reddit.com/r/ROS/comments/12an2ss/` (note: a March 2023 firmware update reportedly changed the SSH root password on some units).
- Home Assistant integration discussion: `https://community.home-assistant.io/t/moorebot-scout-integration/742191`
- `https://livemyself.com/archives/3264` — Japanese-language deep dive into roller_eye control/hooking.

**On this robot itself**, the vendor SDK is installed and importable at:
- `/usr/local/lib/rollereye.py` (1223 lines) and `/usr/local/lib/rollereye_ros_bridge.py` (177 lines) — read these directly over SSH any time the GitHub copy is ambiguous; they are the literal code executing on this hardware. A byte-identical backup copy also lives outside this git repo at `/home/linaro/programming_workspace/pybot_scout_legacy/` on the robot (not tracked in this repository — a pre-refactor stash, not authoritative, just a convenience reference).
- ROS package `roller_eye` is installed at `/opt/ros/melodic/share/roller_eye` (msg/srv/launch/param files readable there; no source/build artifacts, just the installed share directory).

---

## Robot hardware inventory (verified)

| Subsystem | Details |
|---|---|
| Compute | aarch64 SBC, Linux 4.4.189 (`linaro-alip` / `linaro` user), likely Rockchip PX30-based (a `px30.yaml` param file ships in the `roller_eye` package) |
| OS / middleware | ROS 1 **Melodic** on top of a Linux distro (not full desktop Ubuntu — embedded-style Linaro image), Python **2.7.13** only |
| Drivetrain | 4-wheel drive base, holonomic-capable command interface (`cmd_vel` supports simultaneous x/y translation), driven by `MotorNode` |
| Odometry | `MotorNode` publishes `nav_msgs/Odometry` on `/MotorNode/baselink_odom_relative`; VIO (visual-inertial) odometry also available on `/MotorNode/vio_odom_relative` and can be started/stopped via `enable_vio`/`vio_start`/`vio_stop` services. **Correction (verified live 2026-07-26, confirmed again 2026-07-26): NEITHER topic behaves as a continuous accumulating pose when driven the way this codebase actually drives.** Raw `/cmd_vel` publishing for 4s left wheel-odom pose at exactly (0,0,0); a blocking `algo_move` call made it hover near a tiny constant (~0.0145) then reset to exactly (0,0,0) on completion. Re-tested with VIO explicitly enabled (`enable_vio` service, confirmed publishing at 10 Hz) plus a real ~4s `set_translate_4`-driven forward move that visibly displaced the robot off its dock: **both** `/MotorNode/baselink_odom_relative` and `/MotorNode/vio_odom_relative` pose fields stayed at *exactly* `(0.0, 0.0)` throughout. Best-evidence hypothesis: pose integration for both is tied to the blocking `algo_move`/`algo_roll`/`algo_action` service-call state machine, not the continuous `/cmd_vel`-republishing style (`set_translate_4`, etc.) every script in this repo actually uses to drive — so it's simply never exercised, not "broken" so much as built for a different control pattern than ours. Do not trust either odometry topic for dead-reckoning/breadcrumb return-home while driving via continuous Twist — use `/SensorNode/simple_battery_status` (charging flag) as ground truth for "did the robot actually move/dock" instead. `pybot_scout/odometry.py`'s `OdometryTracker.get_stats()["total_distance_m"]` should be treated as unreliable (near-permanently zero) in this drive mode. **Consequence applied 2026-07-26**: `obstacle_avoidance.py`'s odometry-based stuck-detection cross-check was removed (it was silently inert — always compared a real threshold against a value that never moved off zero, so it never once suppressed a recovery). The script was later rewritten (v7-v9.2, see "What we've been working on" below) to drop camera-brightness entirely; stuck-detection today is `_TofStuckDetector`, based purely on the raw `/SensorNode/tof` reading not changing for `STUCK_TOF_TIMEOUT_SECS`, with no odometry or camera-brightness input at all. A genuine fix would mean patching `MotorNode`'s C++ source (open source at `Pilot-Labs-Dev/Scout-open-source`) to integrate pose continuously regardless of `algo_*` state, then cross-compiling/redeploying to the robot — a legitimate option in principle but a heavier, higher-risk embedded undertaking with no easy rollback; treat as a deliberate future task, not something to attempt casually. |
| Proximity / ranging | ToF sensor (`/SensorNode/tof`, `sensor_msgs/Range`, distance in meters) and an "ibeacon" range sensor (`/SensorNode/ibeacon`, `sensor_msgs/Range`) — the iBeacon signal is what the charging-pile docking logic uses to find/align with the dock |
| IMU | `/SensorNode/imu` (`sensor_msgs/Imu`); calibration via `imu_calib`, `imu_patrol_calib` / `getimu_patrolcalib_status` services |
| Light sensor | `/SensorNode/light` (`sensor_msgs/Illuminance`) — per the official README, upper 16 bits of the illuminance value = CH0 channel, lower 16 bits = CH1 channel (needs bit-unpacking, not a plain lux float) |
| Battery | `/SensorNode/simple_battery_status` (`roller_eye/status`) with enum constants `BATTERY_CHARGING=0`, `BATTERY_UNCHARGE=1`, `BATTERY_FULL=2`, `BATTERY_UNKOWN=3` in the `status[]` array |
| Camera / AI vision | `CoreNode` streams H264 (`/CoreNode/h264`) and JPEG (`/CoreNode/jpg`) video (`roller_eye/frame` custom message) plus AAC audio (`/CoreNode/aac`); object/person/pet detection results on `/CoreNode/obj` (`roller_eye/detect`: score, index, class `name`, bounding box top/left/bottom/right, image width/height, `stamp`); generic motion detection on `/CoreNode/motion`. Camera params (resolution, fps, night mode, wide dynamic range, IR light) are exposed as ROS dynamic-reconfigure params under `/ParamNode/video/*` |
| IR / night vision | `night_get` service reports `isNight` + IR LED `brightness`; `adjust_ligth` (sic) service adjusts light (`cmd`: 0-down,1-up,3-max,4-auto); `led_all_on` service |
| Speaker / audio out | Two independent paths exist — see "Speaker/sound: two paths" gotcha below |
| Charging pile / docking | `CoreNode/chargingPile` topic + `BACK_UP_*` state machine in `roller_eye/status` (`BACK_UP_DETECT/ALIGN/BACK/SUCCESS/FAIL/INACTIVE/CANCEL/REDETECT`) drives auto-return-to-dock behavior; `/CoreNode/backing_up` and `/CoreNode/backing_up_status` topics track it; `nav_low_bat` service confirmed to auto-dock the robot, but **only works when the charging pile is currently visible to the camera** — see docking gotcha below |
| Wi-Fi | `WiFiNode` — scan/add/switch SSID, AP vs STA mode (services under `/WiFiNode/*`) |

---

## ROS software architecture (verified live on this robot, 2026-07-26)

- **Distro**: Melodic (`rosversion -d` → `melodic`). No `ROS_MASTER_URI` env var was set in a bare login shell — you must `source /opt/ros/melodic/setup.bash` (and the workspace setup if any) before running `rosnode`/`rostopic`/`rosservice`/`rosparam` over SSH, or they silently fail (`command not found` / empty output).
- **Nodes** (18 confirmed running; `PyBotScoutBridgeNode` from this repo's own `ros_bridge.py` was *not* running during the audit — it only exists while one of our scripts is active): `AppNode, BistNode, CloudNode, CoreNode, DetectRecordNode, MotorNode, NavPathNode, RTMPNode, RecorderAgentNode, S3Node, SchedNode, SensorNode, SpeakerNode, UINode, UpgraderNode, UtilNode, WiFiNode, rosout`.
- **Key topics and verified types**:

  | Topic | Type | Notes |
  |---|---|---|
  | `/cmd_vel` | `geometry_msgs/Twist` | primary motion command; also `/cmd_vel2`, `/cmd_vel3`, `/cmd_vel4`, `/cmd_vel8003`, `/cmd_vel_force` exist (purpose/arbitration between these not yet characterized — treat `/cmd_vel` as the one the vendor SDK and this repo use) |
  | `/SensorNode/tof` | `sensor_msgs/Range` | meters |
  | `/SensorNode/ibeacon` | `sensor_msgs/Range` | charging-pile beacon range |
  | `/SensorNode/imu` | `sensor_msgs/Imu` | |
  | `/SensorNode/light` | `sensor_msgs/Illuminance` | packed CH0/CH1, see hardware table |
  | `/SensorNode/simple_battery_status` | `roller_eye/status` | battery enum in `status[]` |
  | `/CoreNode/obj` | `roller_eye/detect` | AI object/person detection |
  | `/CoreNode/motion` | `roller_eye/detect` | generic motion detection |
  | `/CoreNode/h264`, `/CoreNode/jpg`, `/CoreNode/aac` | `roller_eye/frame` | video/audio streams |
  | `/CoreNode/chargingPile`, `/CoreNode/backing_up`, `/CoreNode/backing_up_status` | — | docking state |
  | `/MotorNode/baselink_odom_relative` | `nav_msgs/Odometry` | wheel odometry |
  | `/MotorNode/vio_odom_relative` | — | VIO odometry |
  | `/speaker_cmd` | `std_msgs/Int32` | published by `/AppNode`, consumed by `/SpeakerNode` — integer sound-effect ID |
  | `/NavPathNode/pose`, `/NavPathNode/status` | — | navigation/patrol pose+status |
  | `/patrol_status` | `roller_eye/patrol_status` | `START_PATROL/END_PATROL/PATROL_LOSE_PILE/PATROL_AVOID_OBS_FAIL` |

- **Services** (105 total; only ~35 are domain services, the rest are the standard `get_loggers`/`set_logger_level` per node). Notable ones by node:
  - `UtilNode`: `algo_move` (x/y distance in m + speed → blocking move), `algo_roll` (angle in rad + rotation speed + timeout/error → blocking turn), `algo_action` (x/y/rotation speed + duration → timed velocity command), `ai_get_detect_setting`/`ai_set_detect_setting` (JSON-string AI detection config)
  - `NavPathNode`: `nav_patrol`/`nav_patrol_stop`/`nav_cancel`/`nav_get_status`, waypoint/path save-load (`nav_path_save`, `nav_path_start`, `nav_waypoint_add`, `nav_list_path`, `nav_delete_path`), `nav_mag_calibra` (magnetometer calibration), `nav_low_bat`
  - `CoreNode`: `adjust_light`, `adjust_exposure_time`, `night_get`, `motion_set_zone` (exclude regions via `contour[]`), `stop_detect`, `nav_cancel`, `saveTmpPicForStartPath`
  - `RecorderAgentNode`: `record_start`/`record_stop`/`record_get_status`/`record_get_files`/`record_delete_file`/`record_clean`
  - `WiFiNode`, `S3Node`, `CloudNode`, `SchedNode`, `UpgraderNode`, `BistNode` (built-in self test), `RTMPNode`, `DetectRecordNode` — app/cloud/maintenance plumbing, generally not needed for autonomy scripting.
  - Global (no node prefix): `imu_calib`, `imu_patrol_calib`/`getimu_patrolcalib_status`, `led_all_on`, `nav_low_bat`, `system_event`, `sys/get_userid`.
- **Custom message package**: `roller_eye` (installed share dir `/opt/ros/melodic/share/roller_eye`). Full msg list: `alexskill, contour, detect, frame, patrol_status, point, record, status, task, wifi_config_info, wifi_info`. `roller_eye/status` is an overloaded "kitchen sink" enum+array message reused across battery/backup/wifi/record/p2p status — always check the value against the constant names, don't assume a fixed meaning per field.
- **ROS params** live under `/ParamNode/video/*` (camera: resolution, fps, night mode, wide dynamic range, IR light) plus standard `/rosdistro`, `/rosversion`, `/run_id`.

---

## On-device vendor SDK (`rollereye.py`) — what our library wraps

`pybot_scout/scout.py` is a direct, faithful port of the class `Rollereye` in `/usr/local/lib/rollereye.py`; `pybot_scout/ros_bridge.py` ports `RollerEyeRosBride` from `/usr/local/lib/rollereye_ros_bridge.py`. When behavior seems surprising, diff against these two files on the robot (or the GitHub copies under `roller_eye/lib/`) — most "bugs" are actually inherited quirks. Key API surface (method names on the `Rollereye` class, roughly grouped):

- **Lifecycle**: `start()`, `release()`, `stop()` — `start()` spins up the ROS bridge + async motion-command sender thread, resets sound volume to 100%, and stops any in-progress recording.
- **Timers**: `timerStart()/timerPause()/timerStop()/getTimerTime()/getRunTime()/getCurrentTime()` — simple wall-clock helpers used by example scripts to bound how long a behavior runs.
- **Motion, via `/cmd_vel`** — despite superficially similar names, these split into two behaviorally different groups (mapped during a full non-blocking-call audit, 2026-07-26 — see `/memories/repo/obstacle_avoidance.md` v9 entry for the investigation):
  - **Truly continuous/non-blocking** (hand off to a background `MotionCmdAsyncSender` thread that keeps re-publishing `Twist` at 100 ms cadence and returns immediately): `set_translationSpeed(speed_m_s)`, `set_rotationSpeed(rotation_speed_deg_s)`, `set_translate(degree)`, `set_translate_4(degree, speed)`, `set_rotate(direction)` (0=none,1=left,2=right), `set_translate_rotate(direction, degree)`, `stop_move()`. Prefer these for anything that should happen "at the same time as" ongoing driving.
  - **Foreground publish-loop (blocks the calling thread for the move's duration, but does NOT call a ROS service)**: `set_translate_2(degree, seconds)`, `set_translate_3(degree, meters)`, `set_translate_smooth(...)` — these call `_stop_async_translate_rotate()` then loop `publish_raw_cmd_vel()`+`time.sleep()` themselves until the duration elapses.
  - **Truly blocking ROS service calls** (synchronous `rospy.ServiceProxy`, blocks until the vendor's own move/rotate completes): `set_rotate_2(direction, seconds)`, `set_rotate_3(direction, degree)` → `/UtilNode/algo_roll`. **Do not use these for anything meant to run concurrently with other logic** — `scripts/obstacle_avoidance.py` used to call `set_rotate_3` this way and it was the single remaining genuinely-blocking call site until fixed in the v9 audit (rewritten to use `set_rotationSpeed`+`set_rotate()` instead, open-loop timed with `time.sleep(degree/rotation_speed)`).
- **Motion (blocking, via `UtilNode` services)**: internal `_move(x, y, speed)` → `algo_move` semantics; `_action(vx, vy, w, time)` → `algo_action`; `_roll(angle, rotation_speed, timeout, error)` → `algo_roll` service.
- **Per-wheel**: `set_wheel(frontLeft, frontRight, rearLeft, rearRight)` — **gotcha: unimplemented stub** in the shipped SDK (`print('to do set_wheel')` and nothing else). Don't rely on it.
- **Sound**: `set_soundVolume(vol)` (`amixer` ALSA mixer control) and `play_sound(effect_id, is_finished)` — **gotcha: this calls `aplay` directly on local WAV files** (`/var/roller_eye/sc_sound_00{1,2,3}.wav`), *not* a ROS call. This is a second, separate path from the `/speaker_cmd` (`std_msgs/Int32`) topic that `AppNode`→`SpeakerNode` use internally for app-triggered sounds. Prefer `aplay`/`play_sound()`-style local playback for scripts (matches vendor SDK); only use `/speaker_cmd` if you need to trigger the same effects `AppNode` uses.
- **Media**: `capture()` (photo via `record_start`/`record_stop` service pair, `MEDIA_TYPE.PIC`), `record_start()`/`record_stop()` (video, `MEDIA_TYPE.VIDEO`).
- **AI detection**: `enable_reg(target)`/`disable_reg(target)` where `target` is the `reg` enum (`person=0, home=1, dog=2, cat=3, motion=1001`); `recogResult()`, `recResult(target)`, `recWait(target)`; `enable_detection()`/`disable_detection()`, `motionDetected()`; `get_ai_last_detect_result()`/`get_ai_last_motion_detect_result()` (subscribes internally to `/CoreNode/obj` and `/CoreNode/motion`).
- **Exception/meta plumbing for the companion app**: `handle_exception(e)`, `handle_meta(msg)`, `handle_msg(msg_type, msg)` — call back into `AppNode` services (`programming_exception_handle`, `programming_meta_handle`, `programming_msg_handle`); mostly relevant if a script needs to surface errors to the mobile app UI, not needed for headless automation.

---

## Algorithmic approaches for steering / autonomy (from the vendor stack)

- **Reactive obstacle avoidance (current design, v7-v9.2, corrected again 2026-07-26)**: `scripts/obstacle_avoidance.py` went camera-brightness reactive turner (worked poorly, false-triggered stuck-detection constantly) -> camera-brightness analog-control explorer with a ToF corroborating safety layer (v1-v6) -> **pure ToF-only straight-line explorer (v7, current)**, after live testing showed the camera-brightness stuck-detector false-triggered ~9x more often than real ToF critical stops even in genuinely open rooms. Current design: drive straight ahead at a fixed speed using only a robustified (median-of-3-debounced) `/SensorNode/tof` reading; stop+backup+`_tof_search_for_room()` (full-circle rotate+sample sweep, commit to the heading with the most room) on either a close-range critical stop or a `_TofStuckDetector`-flagged frozen raw reading (unchanging for too long, whether frozen on a close value or on a genuinely-invalid NaN/-inf/negative reading — **not** on a steady `+inf`, which means confirmed-clear and is the normal/good case, see the ToF `+inf` gotcha below). All `/cmd_vel` motion in this script uses the truly-non-blocking path (see vendor-SDK motion bullet above); no vendor service is paused/killed at startup (removed — pausing `media_core_node` in particular broke the vendor's own low-battery auto-homing, see gotcha below). The old camera-brightness version is archived at `scripts/previous_scripts/obstacle_avoidance_camera_v6.py` for reference, not deleted. The vendor's own `NavPathNode` (`nav_path_node.cpp`) publishes `patrol_status::PATROL_AVOID_OBS_FAIL` when autonomous patrol can't get around an obstacle — i.e. the vendor already implements its own (likely ToF-based) avoidance internally during `nav_patrol`, an alternative if the goal is just "patrol without hitting things" (call `nav_patrol` directly instead of a custom script). Full version-by-version history: `/memories/repo/obstacle_avoidance.md`.
- **Docking / return-to-charger**: driven by the `BACK_UP_*` state machine in `roller_eye/status` (`DETECT → ALIGN → BACK → SUCCESS`, with `FAIL`/`CANCEL`/`REDETECT` fallbacks) combined with the `/SensorNode/ibeacon` range signal *and* camera-vision charging-pile detection (`/CoreNode/chargingPile`) for homing in on the charging pile, surfaced on `/CoreNode/backing_up` / `/CoreNode/backing_up_status`. The `nav_low_bat` service triggers this on-demand (confirmed working: fully redocked the robot once, `UNCHARGE`→`FULL`/charging). **Gotcha verified 2026-07-26: `nav_low_bat` fails fast (does not do a real search) if the charging pile is not currently in the camera's field of view** — `rosout` shows `NavPathNode: "waitObj get obj failured!"` (`algo_utils.cpp`, `AlgoUtils::waitObj2`) followed by `CoreNode: "mStartDetectCharingPile stop"`, and `rosservice call /nav_low_bat "{}"` returns `ERROR: service responded with an error`. Rotating/repositioning blind and retrying is unreliable. **Recommended pattern**: only call `nav_low_bat` when `pybot_scout/charging_pile.py`'s `ChargingPileDetector.was_recently_seen()` is currently True (see `scripts/obstacle_avoidance.py`'s battery watchdog for the implementation) — if the pile hasn't been seen, keep exploring and retry on a later tick rather than forcing the call. `scripts/previous_scripts/return_home.py` (waypoint-replay from logged `move_step`/`step_selected` pose events, archived 2026-07-26 via `git mv`) is **stale/incompatible** with current script logging (those event names aren't emitted anymore) and is superseded by the `nav_low_bat` approach for the low-battery-return use case.
- **Patrol / path following**: `NavPathNode` supports saved waypoint paths (`nav_path_save`, `nav_waypoint_add`, `nav_path_start`, `nav_list_path`) and named patrols (`nav_patrol` service with `isFromOutStart`/`name`), status polled via `nav_get_status` or the `/patrol_status` topic. This is likely built on top of VIO (monocular visual-inertial odometry — see `roller_eye/launch/start_vio*.launch`, `start_vins.launch`, `vins_param.yaml`, and the separate `Pilot-Labs-Dev/vio` repo) for drift-corrected pose rather than wheel odometry alone.
- **AI-detection-driven reactive behavior** (what `scripts/human_detect.py` does): subscribe `/CoreNode/obj`, filter by class `name`/`score`, react (sound/turn/log) with a cooldown — this is a simple, robust pattern for "notice and respond" behaviors and doesn't require touching navigation internals at all.
- **General guidance for custom steering algorithms on this platform**: because the base is holonomic-capable (independent x/y translation via `cmd_vel`), simple potential-field / vector-sum obstacle avoidance (sum repulsive vectors from ToF range readings, drive along the resultant) is a natural fit and simpler than turn-then-drive strategies needed on true differential-drive robots — a genuinely untried upgrade path for `scripts/obstacle_avoidance.py` (current v7-v9.2 design uses a simpler stop-and-sweep reaction, not a continuous vector-sum). Note ToF readings themselves ARE now confirmed reliable/sensible for steering once handled correctly (median-debounced, `+inf` treated as confirmed-clear rather than invalid, non-blocking motion calls) — the earlier "ToF isn't sensible enough" conclusion (see gotcha above) turned out to be caused by camera-brightness stuck-detection false-triggers and blocking-call bugs elsewhere in the script, not by the ToF sensor itself.

---

## Known gotchas / corrections to earlier assumptions

- ~~"differential-drive robot"~~ → 4-wheel-drive, holonomic-capable via `cmd_vel` x/y translation (see drivetrain note above).
- ~~"Speaker: sound playback via service calls"~~ → no ROS service for sound; either publish an int sound-ID to `/speaker_cmd` (mirrors what `AppNode` does) or shell out to `aplay` on local WAV files (mirrors what the vendor SDK's `play_sound()` does — this repo's `feedback.py`/scripts should match whichever path they currently use; verify against `pybot_scout/scout.py`).
- `set_wheel()` is a documented-but-unimplemented stub in the shipped SDK — don't build features around per-wheel RPM control without first confirming it actually does something on this firmware version.
- ROS environment variables are **not** set in a plain SSH login shell — always `source /opt/ros/melodic/setup.bash` before any `rosnode`/`rostopic`/`rosservice`/`rosparam` command over SSH, or the command silently fails/returns nothing.
- `PyBotScoutBridgeNode` (from this repo) only appears in `rosnode list` while one of our own scripts is actively running — its absence is expected at rest, not a bug.
- **`CoreNode` (media_core_node) can get stuck pegged at high CPU for days, silently killing the whole camera/vision pipeline** (verified 2026-07-26: found at 183% CPU, `ELAPSED` uptime 16d19h but cumulative `TIME` of 30d18h — i.e. spinning continuously near 2 full cores the whole time it had been running). Symptoms: `/CoreNode/grey_img` and `/CoreNode/jpg` publish nothing (`rostopic hz` reports "no new messages" indefinitely); `pybot_scout.capture()` always returns the same stale cached file via `record_get_files`; `rosservice call /CoreNode/night_get "{}"` hangs forever; `nav_low_bat` fails (its pile detection depends on the same vision pipeline). **Diagnose**: `ps aux | grep media_core_node` — compare `%CPU` (sustained >100% is abnormal) and cumulative `TIME` vs `ELAPSED` uptime (`ps -o pid,pcpu,time,etime,cmd -p <pid>`). **Fix**: `sudo -n systemctl restart roller_eye.service` (passwordless sudo is configured for this) — restarts the whole vendor ROS stack (roscore + all nodes) cleanly in ~10-15s and resolved the issue instantly in testing (fresh `CoreNode` PID settled back to normal, `grey_img` immediately publishing at a clean 10 Hz). If that doesn't fix it, a full device reboot is the next step (confirmed OK to do with the user first, since it's a bigger interruption). **Always check this first** if camera/capture/docking behavior seems broken or unresponsive — it's cheap to check and was the root cause of several different-looking symptoms in this session.
- **`pybot_scout.start()` calls `disable_print()`, which redirects `sys.stdout` to `os.devnull` for the entire process** — this silences even the calling script's own `print` statements after that point (confirmed: none of our diagnostic `print()` calls in ad-hoc test scripts showed up once `pybot_scout.start()` had run). Use `FeedbackLogger`/JSONL (`pybot_scout/feedback.py`) or write to a file for any post-`start()` diagnostics; don't rely on stdout.
- **A blocking vendor service call left running in a killed/backgrounded SSH session can wedge subsequent calls to the same subsystem.** If an SSH command with a blocking call (e.g. `record_start`, `nav_low_bat`) is killed locally (e.g. terminal killed) without letting the remote Python process exit, the remote `python2` process (and its in-flight service call) keeps running orphaned on the robot and can make later calls to the same service appear to hang or fail for unrelated reasons. Check `ps aux | grep python2` on the robot for stray processes from earlier commands and `kill -9` them if found stuck.
- `capture()`'s "always returns the same stale file" behavior in this session turned out to be a symptom of the `CoreNode` stuck-CPU bug above, not an independent bug — retest after confirming `CoreNode` CPU/uptime looks sane before assuming `capture()` itself is broken.
- **Never add `media_core_node` (`CoreNode`) to any pausable/killable-service list** (found+fixed 2026-07-26, v9.1): `CoreNode` publishes `/CoreNode/chargingPile`, which `charging_pile.ChargingPileDetector.was_recently_seen()` depends on to decide `pile_visible` for the low-battery auto-dock trigger. Pausing it (e.g. via a 'p'-key toggle or an auto-pause-at-startup feature) silently breaks auto-docking — pile sightings stop flowing, `pile_visible` is permanently False, and the robot never calls `nav_low_bat` even sitting right in front of a fully visible charging station. Symptom seen live: "the robot only starts homing once I kill the script" (killing it resumed `CoreNode` via `atexit`). `MotorNode`/`SensorNode`/`SupervisorNode` are excluded from pausable-service lists for the same class of reason (this script's own motion/sensing depends on them) — always check whether a candidate process backs a ROS node this script (or its dock/battery logic) subscribes to before adding it to any pause/kill list.
- **`/SensorNode/tof` reading of `+inf` means "confirmed clear, no obstacle within sensor range"** — it is NOT an invalid/error/unmeasurable reading (confirmed empirically by the user via live testing, 2026-07-26). Only NaN, `-inf`, and negative values are genuinely invalid. An earlier version of `scripts/obstacle_avoidance.py` (and its own comments) wrongly treated `+inf` the same as those invalid cases, which caused wide-open spaces to look like "unmeasurable" and could incorrectly trigger stuck/blind-mode fallback, degrade `_tof_search_for_room()`'s heading comparison, and weaken `_wag_scan_left_right()`'s side-awareness — all fixed in `_tof_distance()`, see `/memories/repo/obstacle_avoidance.md` v9.2 entry. If you add new ToF-consuming code, treat `+inf` as a real, valid, maximally-far distance, not as `None`/invalid.

---

## What we've been working on

1. **`scripts/obstacle_avoidance.py`** — the current "explore the apartment" script (there is no separate `scripts/random_walk.py` in this repo despite older references to that filename — this script *is* the random-walk/exploration behavior now). Originally rewritten 2026-07-26 as a v7 ToF-only straight-line explorer (replacing an older camera-brightness analog controller entirely — see `/memories/repo/obstacle_avoidance.md` for the full v1-v9.2 history), then iterated same-day through v8/v9/v9.1/v9.2 fixes based on live testing: drive straight ahead at a fixed speed, using a robustified (median-of-3 debounced) `/SensorNode/tof` reading as the sole safety/steering signal; stop+backup+full-circle-sweep-and-commit-to-best-heading (`_tof_search_for_room()`) on either a close-range critical stop or a `_TofStuckDetector`-flagged frozen reading. All ROS calls are now non-blocking (v9 audit) and the script no longer pauses/kills any vendor service on start (removed after the v9.1 finding that pausing dominance broke the vendor's own homing-on-low-battery behavior). **Critical corrected sensor fact (v9.2, 2026-07-26, confirmed empirically by the user via live testing): a raw `/SensorNode/tof` reading of `+inf` means "no obstacle within the sensor's range" — the path is completely clear — it is NOT an invalid/unmeasurable reading.** Only NaN, `-inf`, and negative readings are genuinely invalid. `_tof_distance()` returns the real `float('inf')` value for a confirmed-clear reading rather than collapsing it to `None`; downstream threshold/stuck-detector logic already handles `inf` correctly via normal Python float semantics, so no other call sites needed to change. Battery watchdog / auto-dock logic (`PYBOT_SCOUT_RETURN_BATTERY_PCT` / `PYBOT_SCOUT_UNDOCK_BATTERY_PCT`, `/nav_low_bat`, pile-visibility gating) is unchanged and carried over as-is. The old camera-based version is preserved for reference at `scripts/previous_scripts/obstacle_avoidance_camera_v6.py`. `scripts/previous_scripts/` also holds `return_home.py`, `patrol.py`, `example.py`, and `human_detect.py` (moved 2026-07-26 alongside the rewrite so `scripts/` only contains the current exploration script plus `runtime_probe.py`); see their individual notes below for what they still reference (paths unchanged, git history preserved via `git mv`).
2. **`scripts/previous_scripts/patrol.py`** (archived 2026-07-26, moved via `git mv`) — similar long-running charge/explore/return cycle built on the same camera-brightness steering, with its own multi-strategy return-home logic (see file docstring); not re-verified this session, but shares the same underlying camera/pile-detection dependencies documented above. Not currently run; kept for reference only.
3. **`scripts/previous_scripts/human_detect.py`** (archived 2026-07-26, moved via `git mv`) — Subscribes to `/CoreNode/obj`, plays a configurable sound via `/SpeakerNode` each time a person is detected, with a cooldown to avoid rapid re-triggering. Not currently run; kept for reference only.
4. **`scripts/runtime_probe.py`** – Samples all proximity topics for a configurable duration and logs readings. Useful for checking what sensor data is actually available on a given run.
5. **`scripts/previous_scripts/return_home.py`** (archived 2026-07-26, moved via `git mv`) — waypoint-replay return-home approach; confirmed **stale/superseded** this session (see docking gotcha above) — don't use as a reference for current return-home behavior.
6. **`pybot_scout/ros_inventory.py`** – Logs the full ROS graph (nodes, topics with types, services) at script startup for traceability.

---

## How the agent accesses this workspace (current)

**The agent now has direct access to the robot**, via two complementary channels:

1. **SSH FS extension (mounted workspace)** — This repository is opened through the VS Code SSH FS extension, which exposes the robot's filesystem (`/home/linaro/programming_workspace/pybot_scout`) as `ssh://moorebot_scout/...` paths. To the agent's file tools (read/create/edit) this behaves like a normal local workspace — there is no need to `cat`/`scp`/heredoc file contents through a terminal. **Always prefer the normal file-editing tools for reading and writing file contents.** A terminal (SSH or otherwise) should only be used for things file tools can't do, e.g. `git mv`, `git status`, running scripts, or other shell/git operations.
2. **Direct SSH command execution** — The robot is reachable at `192.168.1.55` (hostname `linaro-alip`, user `linaro`) and now uses **public/private key authentication** (an `ed25519` key was installed in `~/.ssh/authorized_keys` on the robot), so commands run non-interactively, e.g.:
   ```
   ssh -o BatchMode=yes 192.168.1.55 '<command>'
   ```
   This allows the agent to run scripts, inspect logs, check ROS topics, and generally test changes on the robot directly in the same session — no more manual round-tripping required.

### Previous workflow (legacy, superseded)

Before the above was set up, the agent had **no direct access to the robot at runtime**. Instead:

1. A separate local working copy (not on the robot) tracked the same git remote as the robot's repo.
2. **Robot runs a script** → the script writes a JSONL feedback file to `run_feedback/` (e.g. `run_feedback/20260620_204730_human_detect.jsonl`).
3. The user manually ran `git pull` on the robot to push logs, and `git push`/`git pull` between the local copy and the robot's repo to shuttle code changes back and forth.
4. **Agent reads the logs** in a new session, diagnoses issues, and improves the code.
5. **Agent commits the improved code**; the user manually synced it back onto the robot.
6. **Repeat.**

This is why `run_feedback/*.jsonl` logs exist and follow the `logger_started` convention below — the mechanism may well continue to be useful, but it was originally designed around the constraint of no direct robot access, which no longer applies. **Adapting the scripts/logging approach to take advantage of direct SSH/file access is an open task for a future session — don't assume the current file-based feedback approach is still the best fit without discussing it first.**

Every log file starts with a `logger_started` event that includes the git commit SHA and branch so logs can always be matched to the exact code version that produced them.

---

## Standard tasks for agents

When starting a session in this repo, check `run_feedback/` first:

- **If there are new log files**: read them, look for errors, anomalies, or opportunities to improve the scripts, then make targeted code changes. Summarise your findings in the commit message.
- **After reviewing logs**: keep logs that are still useful for calibration/trend analysis (for example sensor thresholds, camera-vs-ToF correlations, and regressions that can recur). Delete stale logs that are no longer useful (for example from superseded behavior or missing key signals).
- **If the user asks for cleanup only**: delete only logs explicitly marked as disposable or clearly obsolete.

(See "Keep this file up to date" near the top of this file for the full policy — applies continuously, not just at session end.)

---

## Runtime constraints (must follow)

- Target robot runtime is **Python 2.7.13** (ROS 1). Keep scripts Python-2-compatible.
- **Do not run `apt upgrade` on the robot** (can brick the device).
- Be conservative with package installs/updates on the robot; avoid recent package versions that may break Python 2.7 compatibility.
- **ROOT access exists but is app-gated** (per official README: tap firmware version 5× in the mobile app, password `123456`, Android app only as of the vendor docs) — this SSH session already has shell access as `linaro`, which is normally sufficient. Don't chase full root unless a task specifically requires it (e.g. modifying files outside `linaro`'s permissions), and treat it as a deliberately destructive-capable escalation requiring the same caution as any other irreversible action.
- Remember to `source /opt/ros/melodic/setup.bash` before running any `rosnode`/`rostopic`/`rosservice`/`rosparam` command over a fresh SSH connection — the ROS environment is not sourced by default in a login shell.

---

## Key environment variables

| Variable | Default | Purpose |
|---|---|---|
| `PYBOT_SCOUT_ALLOW_SENSORLESS_FALLBACK` | unset (disabled) | Set to `1` to allow movement without sensor data |
| `PYBOT_SCOUT_PROXIMITY_TOPICS` | auto-discover | Comma-separated list of range topic names |
| `PYBOT_SCOUT_ROS_PACKAGE` | `roller_eye` | ROS package name used by the robot vendor |
| `PYBOT_SCOUT_HUMAN_SOUND_ID` | `1` | Sound effect ID for human detection (1–3) |
| `PYBOT_SCOUT_HUMAN_COOLDOWN` | `3.0` | Seconds between human-detect sound triggers |
| `PYBOT_SCOUT_RETURN_BATTERY_PCT` | `50` | Battery % at/below which `obstacle_avoidance.py`/`patrol.py` attempt a dock (only if charging pile currently visible) |
| `PYBOT_SCOUT_UNDOCK_BATTERY_PCT` | `99` | Battery % at which a charging `obstacle_avoidance.py` run unlodges from the dock and resumes exploring |
