# Spark course: navigation + grasp handoff

English | [中文](README.zh-CN.md)

`course_nav.py` drives the NXROBO Spark through the course:

1. It leaves the start line and goes around the box in the middle of the lane.
2. At the end line it switches to the camera and stops **just short of the blue or yellow object**, close enough for the arm, without pushing it.
3. It comes back to the start line.

It does **no grasping**. It hands over to a grasp/release script through two ROS topics; see [Handoff protocol](#handoff-protocol).

This README explains how the script sits on top of the Spark codebase, and how to reproduce our runs exactly.

| File | What it is |
|---|---|
| `course_nav.py` | The navigation script. The only code you run. |
| `course_nav_start.json` | The course start pose on the map in `map/`. Written by `course_nav.py set-start`. |
| `map/test_map.yaml`, `map/test_map.pgm` | The SLAM map we ran on (made 13 Oct, 17:31). |
| `camera_stream.py` | Optional: shows the robot's camera in a browser. |

---

## How this fits into the Spark codebase

We did **not** modify or replace any Spark code. `course_nav.py` is a plain ROS node that runs on top of the stock Spark navigation stack:

```
course_nav.py   (this repo)  course logic: where to go, detour memory, camera approach,
      |                      stop-short rule, safety checks, grasp handoff topics
      |  move_base goals, /cmd_vel for short moves, reads /scan /odom /tf /map /camera
      v
spark_navigation/amcl_demo_lidar.launch   (Spark)
      |-- map_server        serves the saved SLAM map
      |-- amcl              localizes the robot on it with the lidar
      |-- move_base         plans paths around obstacles (DWA local planner, costmaps)
      |-- velocity smoother (follower_velocity_smoother)
      `-- spark_bringup/driver_bringup.launch
            |-- spark_base        chassis driver (/dev/sparkBase), /odom
            |-- ydlidar_g2        lidar driver, /scan
            |-- astra camera      /camera/rgb/image_raw
            `-- robot description and TF
```

| Spark part | Used for | How |
|---|---|---|
| `spark_slam/2d_slam_teleop.launch` (OneKey option **4**) | Making the map | gmapping + keyboard teleop, then saved with `map_server map_saver` |
| `spark_navigation/amcl_demo_lidar.launch` (OneKey option **6** is the same with RViz) | Drivers, localization, path planning | Started once; `course_nav.py` sends it goals |
| Spark config files (`spark_navigation/param/*.yaml`) | Costmaps, DWA planner, AMCL | Used as-is. `course_nav.py` raises the speed limits at runtime through `dynamic_reconfigure`, without editing any file |
| `move2grasp/scripts/grasp_pro_new.py`, `spark_sagittarius_carry/config/hsv.yaml` | Blue / yellow HSV colour ranges | Values copied into `course_nav.py`; that code isn't called |
| `task1`, `move2grasp` `move.py`, the SwiftPro arm | Nothing | Not used. They must **not** run at the same time (see [Rules](#rules-that-matter)) |

### Versions we ran on

- **Robot computer:** Ubuntu 20.04.5, ROS Noetic.
- **Robot and sensors:** Spark with a YDLIDAR G2, an Astra camera, and a SwiftPro arm (not used by this script).
- **Spark workspace:** `~/spark_noetic`, cloned from `https://github.com/nxrobo/spark_noetic` at commit `a3669fe` and built with `catkin_make`.
  - The only local change that touches navigation is `max_vel_x` in `spark_navigation/param/dwa_local_planner_params.yaml`: 0.3 upstream, 0.1 on our robot.
  - `course_nav.py` sets it to 0.3 at runtime either way, so a stock checkout behaves the same.
- **Python packages:** only what ships with ROS Noetic on the Spark: `rospy`, `actionlib`, `tf`, `cv_bridge`, OpenCV, NumPy and `dynamic_reconfigure`.

---

## Reproduce our run, step by step

Everything below runs **on the robot**, as user `spark`, either over SSH (`ssh spark@<robot-ip>`) or in a terminal on its screen. Our robot was at `192.168.161.145`.

### 0. Get the code onto the robot (once)

```bash
cd ~
git clone https://github.com/RAY2L/xrobot.git
cd ~/xrobot
```

Run `course_nav.py` from this folder. It reads and writes `course_nav_start.json` (and `course_nav_reach.json`, if you teach stop lines) next to itself.

### 1. Map: use ours, or make a new one

- **Same room and course as ours.** Use `~/xrobot/map/test_map.yaml` and the `course_nav_start.json` that comes with it, then skip to step 2.
- **Anywhere else**, or if the furniture moved a lot, make a new map:
  1. **Set up the course.** Put the box in its place and the objects at the end line.
  2. **Start mapping.** Run `cd ~/spark_noetic && ./onekey.sh`, choose **4** (2D map with the lidar), then **1** (gmapping). That runs:
     ```bash
     roslaunch spark_slam 2d_slam_teleop.launch slam_methods_tel:=gmapping camera_type_tel:=astra lidar_type_tel:=ydlidar_g2
     ```
  3. **Drive slowly with the keyboard** (`w` forward, `s` back, `a`/`d` turn).
     - Go from the start line down the lane, past the box on both sides, to beyond the end line, and back.
     - Turn in place once at each end, so walls and furniture all around get mapped.
  4. **Save the map**, while mapping is still running, in a second terminal:
     ```bash
     source ~/spark_noetic/devel/setup.bash
     rosrun map_server map_saver -f ~/xrobot/map/test_map
     ```
     Spark's own helper, `spark_slam/scripts/cmd_save_map.sh`, saves to `spark_slam/scripts/test_map` instead. Either location works.
  5. **Stop mapping** with Ctrl-C. A new map always needs `set-start` (step 3).

### 2. Start the navigation stack: exactly one copy

```bash
source ~/spark_noetic/devel/setup.bash
roslaunch spark_navigation amcl_demo_lidar.launch camera_type_tel:=astra lidar_type_tel:=ydlidar_g2 \
    start_camera:=true map_file:=$HOME/xrobot/map/test_map.yaml
```

Leave it running. Then, in a second terminal, turn off `move_base`'s spin-in-place recovery and restart `move_base`. When localization was lost it made the robot spin wildly, and these settings are only read at startup:

```bash
source ~/spark_noetic/devel/setup.bash
rosparam set /move_base/recovery_behavior_enabled false
rosparam set /move_base/clearing_rotation_allowed false
rosnode kill /move_base
rosrun move_base move_base cmd_vel:=/navigation_velocity_smoother/raw_cmd_vel odom:=odom scan:=scan __name:=move_base
```

Leave that running too.

Optional: RViz on the robot's screen, to see the map, the robot and its planned path:

```bash
rviz -d ~/spark_noetic/src/spark_app/spark_navigation/rviz/amcl_camera_astra.rviz
```

You can use OneKey option **6** instead of the `roslaunch` above. It's the same stack plus RViz, but it always loads `spark_slam/scripts/test_map.yaml`, so save the map there if you use it.

**Never start a second stack, mapping, teleop or another navigation script while this is running.** Two launches on one ROS master kill each other's nodes, and we lost the lidar and the base this way more than once.

### 3. Tell the script where the robot is

Put the robot **on the start line, centred, facing down the lane**. Neither command moves it.

- **After making a new map**, record the start pose once:
  ```bash
  cd ~/xrobot
  python3 course_nav.py set-start --min-match 0.45
  ```
  This searches the whole map for the pose that matches the lidar and saves it to `course_nav_start.json`. The lane is taken to run the way the robot faces. `--min-match 0.45` accepts a 45 % match, needed when the map is smaller than what the lidar sees.
- **Every time the stack is (re)started**, or the robot was carried by hand:
  ```bash
  python3 course_nav.py localize
  ```
  This finds the robot near the course and tells AMCL. It prints a match percentage: we saw 75–85 %.

### 4. Check, then run

```bash
python3 course_nav.py --dry-run --targets any,any     # checks map, goals and localization; no motion
python3 course_nav.py round-trip --targets any,any    # the exact command of our final run
```

`round-trip --targets any,any` runs **two trips**. Each trip:
1. Navigates on the map to 2.3 m down the lane, past the box.
2. Camera-approaches whichever of blue or yellow it sees, and stops just short of it.
3. Reverses 0.2 m, turns, and navigates back to the start line.

Our final run took **2 min 42 s** for both trips. The robot ended within 9 cm of the start.

Other useful commands:

```bash
python3 course_nav.py round-trip --targets blue,yellow            # trip 1 blue, trip 2 yellow
python3 course_nav.py round-trip --targets blue,yellow --handoff  # waits for the grasp script at each end
python3 course_nav.py to-end --targets blue                       # one way, stop at blue
python3 course_nav.py to-start                                    # come back from wherever it is
python3 course_nav.py round-trip                                  # map only: out to 2.75 m and back, no camera
python3 course_nav.py --help                                      # every option
```

Stop at any time with **Ctrl-C**: the robot stops.

### 5. Watching from a laptop (optional)

- **Camera.** Run `python3 ~/xrobot/camera_stream.py` on the robot, then open `http://<robot-ip>:8080/`. It shows blue/yellow detections and the red stop line. It never moves anything.
- **RViz.** It runs on the robot's own screen. Our robot also runs a VNC server on port 5900.

---

## How the run works

- **Start pose.** Comes from `course_nav_start.json`. Everything else is measured from it: x runs down the lane, y to the left.
- **Map leg.**
  - `move_base` drives to 2.3 m down the lane (`--handover-distance`). The box is in the saved map and the live lidar sees it, so the planner goes around it.
  - The first pass remembers the point where it swung out, kept at least 0.5 m from the box. Every later leg, including the way back, goes through that point, so the detour mirrors the way out.
  - `--side left|right` forces a side.
- **Camera approach.**
  - It searches straight ahead for the colour, then steers at it.
  - It stops when the object's **bottom edge** in the image reaches the stop line, by default image row 400 of 480 (`--stop-row`). The object is still in view when it stops.
  - It never drives blind. If it loses the object, it stops and waits up to 3 s for it to reappear (`--lost-timeout`). It also stops if the lidar sees anything within 25 cm ahead.
- **Way back.**
  - It reverses 0.2 m (`--backoff`) so turning can't sweep into the objects. Backing stops if anything is within 35 cm behind.
  - It then navigates back through the remembered detour point to the start pose and turns to face down the lane again.
- **Speeds for this run** (set at startup, nothing written to config files):

  | What | Speed |
  |---|---|
  | `move_base` | up to 0.3 m/s (`--nav-speed`; Spark default 0.1 on our robot) |
  | Approach, far from the line | 0.12 m/s |
  | Approach, near the line | 0.04 m/s |
  | Back-off | 0.1 m/s |

### Teaching the stop line (recommended)

Row 400 is a generic guess. To make the robot stop exactly where **your** arm reaches:
1. Put the robot where the arm can grab the object, with the object still visible on the camera stream.
2. Run this (no motion):
   ```bash
   python3 course_nav.py teach-reach --targets blue
   ```
3. Repeat for yellow.

This saves where that object's bottom edge and centre sit in the image to `course_nav_reach.json`. The approach then stops there and steers to keep the object at that column. Re-teach whenever the camera is re-aimed.

To test only the approach, put the robot 1–1.5 m in front of an object and run `python3 course_nav.py approach --targets blue`.

### Rules that matter

- **One stack, one driver.**
  - Only `course_nav.py` may command the base while it runs.
  - Stop OneKey teleop, mapping, `task1` and `move2grasp` `move.py`/`grasp_pro_new.py` first. In one run, a `move2grasp` goal took over `move_base` mid-trip.
  - The grasp script must use the [handoff](#handoff-protocol) and never publish `/cmd_vel` or `move_base` goals.
- **The start pose belongs to one map.** Re-making the map means re-running `set-start`.
- **If the robot was carried or bumped, run `localize`.** The script refuses to drive if under 50 % of the lidar matches the map at AMCL's pose. This saved us several times.
- **If the battery dies, everything has to be restarted:** steps 2 and 3. Nothing starts by itself on boot.

---

## Handoff protocol

Both topics are `std_msgs/String`. Status messages are JSON and latched, so a late subscriber still gets the last state.

| `course_nav.py` | grasp script |
|---|---|
| owns the base: `/cmd_vel`, `move_base` goals | owns the arm: `/position_write_topic`, `/pump_topic` |
| publishes `/course_nav/status` | listens on `/course_nav/status` |
| listens on `/course_nav/continue` | publishes `/course_nav/continue` |

With `--targets blue,yellow --handoff` the status sequence is:

```
TO_END        {"trip": 1, "target": "blue"}
AT_END        {"trip": 1}
APPROACHING   {"color": "blue"}
IN_REACH      {"color": "blue", "pixel": [u, v], "bottom_row": 401, "stop_row": 400, "taught": true, "stopped_by": "camera", "pose": [x, y, yaw], "travelled_m": 0.6}
   (or NOT_IN_REACH {"reason": "not_found" | "lost" | "blocked" | "too_far", ...})
WAITING       {"waiting_for": "grasp"}      <- grasp now, then publish /course_nav/continue
TO_START      {"trip": 1}
AT_START      {"trip": 1}
WAITING       {"waiting_for": "release"}    <- release now, then publish /course_nav/continue
TO_END        {"trip": 2, "target": "yellow"}
... same for yellow ...
DONE
```

- To continue, publish any string on `/course_nav/continue`, for example `"done"`.
- To abort, publish `"abort"`. The robot stops where it is and the script exits.
- If nothing arrives within `--wait-timeout` (600 s), the robot stays put and the script exits.
- If anything fails, the robot stops, `FAILED {"error": "..."}` is published, and the script exits.

In `IN_REACH`:
- `pixel` is the object's colour-blob centre in the 640×480 image `/camera/rgb/image_raw` when the robot stopped. For the blue bottle that's the **blue cap on top**, not its base. The object is still in view, so the grasp script can look at it again.
- `stopped_by` is `camera` (reached the stop line) or `lidar` (something within 25 cm ahead).

### Minimal grasp script skeleton

```python
#!/usr/bin/env python3
import json, rospy
from std_msgs.msg import String
from swiftpro.msg import position, status

rospy.init_node('course_grasp')
arm = rospy.Publisher('/position_write_topic', position, queue_size=1)
pump = rospy.Publisher('/pump_topic', status, queue_size=1)
go_on = rospy.Publisher('/course_nav/continue', String, queue_size=1)
last_in_reach = {}

def grasp(info):
    # info = the IN_REACH record: colour, pixel [u, v], robot pose ...
    # TODO: pixel -> arm x, y (needs a calibration for the CURRENT camera position), then
    # hover, lower, pump on, lift to a carry pose that keeps the camera/lidar clear.
    pass

def release():
    # TODO: lower at the start line, pump off, arm back to its carry/home pose.
    pass

def on_status(msg):
    s = json.loads(msg.data)
    if s['state'] in ('IN_REACH', 'NOT_IN_REACH'):
        last_in_reach.clear(); last_in_reach.update(s)
    elif s['state'] == 'WAITING':
        if s['waiting_for'] == 'grasp':
            if last_in_reach.get('state') != 'IN_REACH':
                go_on.publish('abort'); return
            grasp(last_in_reach)
        else:
            release()
        go_on.publish('done')

rospy.Subscriber('/course_nav/status', String, on_status, queue_size=10)
rospy.spin()
```

Start the arm driver (`roslaunch swiftpro pro_control_nomoveit.launch`). Run the grasp script in its own terminal, then `course_nav.py round-trip --targets blue,yellow --handoff`.

## Arm notes for the grasp side

- **Arm.** SwiftPro on `/dev/ttyACM0`; `roslaunch swiftpro pro_control_nomoveit.launch` starts its driver.
- **Topics.**
  - `/position_write_topic` takes `swiftpro/position`: x, y, z in mm, in the arm frame.
  - `/pump_topic` takes `swiftpro/status`: `1` is suction on, `0` is off.
  - The arm's state just echoes commands. It doesn't confirm suction, so check a grasp with the camera after lifting.
- **Reference sequence.** `~/spark_noetic/src/task1/scripts/mission.py`, in `pick()` / `release()`:

  | Step | Arm position |
  |---|---|
  | Hover | z = 20 |
  | Lower | z = -50 |
  | Pump on, then carry | (20, 150, 130) |
  | Drop | (200, 0, -50) |
- **The camera was re-aimed during our session**, so the old pixel-to-arm calibrations don't fit it. That covers `task1/config/arm_calibration.txt` and `~/thefile.txt`; the latter is uncalibrated, `0 227.5 0 105`.
  - The format is `a b c d`, meaning `arm_x = a*v + b`, `arm_y = c*u + d`.
  - To recalibrate, use OneKey option **8** (camera–arm calibration), which writes `~/thefile.txt`.
  - Simpler alternative: grasp at a fixed arm position, where the object sits when the robot is at the taught stop line.
- **Grasp height differs per object.** The blue bottle is taller than the yellow cube, and for blue the detected point is its top.
