# Spark course: navigation + grasp handoff

`course_nav.py` drives the Spark from the start line to the blue/yellow objects at the end line, around the box in the middle, and back. When it gets there, it stops with the object within the arm's reach. It does **no grasping**. This README is for whoever writes the grasp/ungrasp script that plugs into it.

On the robot (`spark@192.168.161.145`) the scripts are in `~/spark_noetic/src/3rd_app/move2grasp/scripts/`. Copies are in this repo.

- `course_nav.py`: the navigation.
- `camera_stream.py`: shows the robot's camera in any browser on the network.

## Watching the robot from your laptop

- **Camera.** On the robot, run `python3 camera_stream.py`, then open **http://192.168.161.145:8080/**. The overlay shows:
  - boxes around blue and yellow detections;
  - the dashed **stop line** (red until a colour is taught, then one per colour, with its column);
  - a grey dashed line at row 150; blobs above it (far background, such as the yellow sofa) are ignored.
- **RViz.** It runs on the robot's own screen. To see it from a laptop, connect a VNC viewer to `192.168.161.145:5900`.

## The split

| | `course_nav.py` (done) | grasp script (to write) |
|---|---|---|
| Owns | the base: `/cmd_vel`, `move_base` goals | the arm: `/position_write_topic`, `/pump_topic` |
| Does | start → past the box on the map, camera approach that stops just short of the object, back to start | grasp at the end, release at the start |
| Talks on | publishes `/course_nav/status`, listens on `/course_nav/continue` | listens on `/course_nav/status`, publishes `/course_nav/continue` |

Never publish `/cmd_vel` from the grasp script while `course_nav.py` is running, and the navigation script never touches the arm. During navigation, keep the arm in a pose that doesn't block the lidar or the camera.

## Running the navigation

1. **Start the navigation stack.** Either use OneKey 6 (`./onekey.sh`, option 6, which also opens RViz) or run it headless:
   ```bash
   roslaunch spark_navigation amcl_demo_lidar.launch camera_type_tel:=astra lidar_type_tel:=ydlidar_g2 start_camera:=true
   ```
   Recommended: turn off `move_base`'s spin-in-place recovery. When localization was lost it made the robot spin wildly. The parameters are read at startup, so `move_base` has to be restarted after setting them:
   ```bash
   rosparam set /move_base/recovery_behavior_enabled false
   rosparam set /move_base/clearing_rotation_allowed false
   rosnode kill /move_base
   rosrun move_base move_base cmd_vel:=/navigation_velocity_smoother/raw_cmd_vel odom:=odom scan:=scan __name:=move_base &
   ```
   Run only **one** stack. Two launches on the same ROS master kill each other's nodes.

2. **Localize.** This doesn't move the robot. It matches the lidar against the saved map and tells AMCL where the robot is:
   ```bash
   cd ~/spark_noetic/src/3rd_app/move2grasp/scripts
   python3 course_nav.py localize
   ```

3. **How the end of the run works.**
   - **Map leg.** The map gets the robot just past the box: 2.3 m down the lane (`--handover-distance`).
   - **Camera approach.** The camera searches straight ahead for the target colour, then steers at it. The robot **stops just short of the object while it is still in view**: when the object's bottom edge in the image reaches the stop line.
   - **Never pushes the object.** There is no blind driving. If the object is lost after being seen, the robot stops. It also stops if the lidar sees anything within 25 cm ahead.
   - **Stop line.** By default it's image row 400 (`--stop-row`; the image is 480 rows tall). Watch it on the camera stream.

   **Teach the stop line (recommended, no motion).** Put the robot where the arm can grab the object, with the object still visible on the camera stream, then run:
   ```bash
   python3 course_nav.py teach-reach --targets blue
   ```
   Repeat for `--targets yellow`. This records where that object's bottom edge and centre sit in the image, and saves them to `course_nav_reach.json` next to the script. From then on the approach stops with that object right there and steers to keep it at that column. Re-teach whenever the camera is re-aimed.

   To test the approach alone, put the robot about 1–1.5 m in front of an object and run `python3 course_nav.py approach --targets blue`.

4. **Drive.**
   ```bash
   python3 course_nav.py round-trip                              # plain out-and-back, no approach
   python3 course_nav.py round-trip --targets blue               # there, stop short of blue, back
   python3 course_nav.py to-end --targets blue                   # go and stop with blue in reach
   python3 course_nav.py to-start                                # come back
   python3 course_nav.py round-trip --targets blue,yellow --handoff   # full run with your grasp script
   ```
   `--dry-run` checks everything without moving. `--help` lists all options.

## Handoff protocol

Both topics are `std_msgs/String`. Status messages are JSON, latched, so a late subscriber still gets the last state.

With `--targets blue,yellow --handoff` the status sequence is:

```
TO_END        {"trip": 1, "target": "blue"}
AT_END        {"trip": 1}
APPROACHING   {"color": "blue"}
IN_REACH      {"color": "blue", "pixel": [u, v], "bottom_row": 401, "stop_row": 400, "taught": true, "stopped_by": "camera", "pose": [x, y, yaw], "travelled_m": 1.1}
   (or NOT_IN_REACH {"reason": "not_found" | "lost" | "blocked" | "too_far", ...})
WAITING       {"waiting_for": "grasp"}      <- grasp now, then publish /course_nav/continue
TO_START      {"trip": 1}
AT_START      {"trip": 1}
WAITING       {"waiting_for": "release"}    <- release now, then publish /course_nav/continue
TO_END        {"trip": 2, "target": "yellow"}
... same for yellow ...
DONE
```

- To continue, publish any string on `/course_nav/continue` (for example `"done"`).
- To abort, publish `"abort"`. The navigation stops where it is and exits.
- If nothing arrives within `--wait-timeout` (600 s), it stays put and exits.
- If anything fails, the robot stops, `FAILED {"error": "..."}` is published, and the script exits.

In `IN_REACH`:
- `pixel` is the object's colour blob centre in the 640×480 image `/camera/rgb/image_raw` when the robot stopped. For the blue bottle that's the **blue cap on top**, not its base. The object is still in view, so the grasp script can look at it again.
- `stopped_by` is `camera` (reached the stop line) or `lidar` (something closer than 25 cm ahead).

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

Run it in a second terminal before or after starting `course_nav.py ... --handoff`. The status topic is latched.

## Arm notes for the grasp side

- **Arm driver.** `roslaunch swiftpro pro_control_nomoveit.launch` starts it. The arm is the SwiftPro on `/dev/ttyACM0`.
- **Topics.**
  - `/position_write_topic` takes `swiftpro/position` x, y, z in mm, in the arm frame.
  - `/pump_topic` takes `swiftpro/status`: `1` is suction on, `0` is off.
  - The arm's state echoes commands. It doesn't confirm suction, so check a grasp by looking at the camera after lifting.
- **Reference sequence.** `~/spark_noetic/src/task1/scripts/mission.py`, in `pick()` and `release()`, does hover at z=20, lower to z=-50, pump on, carry at (20, 150, 130), drop at (200, 0, -50). Reuse those numbers as a starting point.
- **The camera has moved.** It has been re-aimed more than once today. The old pixel-to-arm calibration in `task1/config/arm_calibration.txt` is therefore wrong for the current camera. `~/thefile.txt` is uncalibrated: `0 227.5 0 105`.
  - The format is `a b c d`, meaning `arm_x = a*v + b`, `arm_y = c*u + d`.
  - Redo the Spark camera–arm calibration (the `cali_pos_node` / `cali_cam_cv3_node` tools write `~/thefile.txt`).
  - Simpler alternative: grasp at a fixed arm position, the spot where the object sits when the robot is at the taught stop line. Record that arm x/y/z while teaching.
- **Grasp height differs per object.** The blue bottle is taller than the yellow cube, so they need different heights, and for blue the detected point is its top.

## Things to know

- **Start pose.** It's tied to the saved map (`spark_slam/scripts/test_map.yaml`) and stored in `course_nav_start.json` next to the script. Whenever the map is re-made, put the robot on the start line facing down the lane and run `python3 course_nav.py set-start` (no motion; add `--min-match 0.45` if the map is small). It finds the robot anywhere on the map and saves that as the start. Current start on the 17:31 map: x=0.00, y=0.00, heading 64°.
- **Where it stops.**
  - With `--targets`, map navigation ends 2.3 m down the lane (`--handover-distance`), and the camera approach may cover up to 2.2 m more (`--approach-max`).
  - Without `--targets`, the map goes all the way to `--end-distance` (2.75 m). `move_base` usually stops about 10 cm short of its goals.
  - Approach speed is 5 cm/s, halved near the stop line.
- **Leaving the objects.** Before turning for home, the robot reverses 0.2 m straight back (`--backoff`), so turning in place can't sweep into the objects. It stops backing if anything is within 35 cm behind it. `to-start` does the same if it starts while parked at the objects.
- **Detour around the box.** The box is in the saved map, and the live lidar also sees it. The first pass lets the planner choose a side. After that, every leg goes through the same remembered point, kept ≥ 0.5 m from the box, so the way back mirrors the way out. `--side left|right` forces a side.
  - On the very first outbound pass the robot can get close: measured gaps were about 9 cm between the robot's edge and the box.
- **Safety.**
  - It refuses to move unless AMCL has converged and at least half of the live lidar points match the map.
  - It stops at once if the robot's map pose goes stale. AMCL crashed once mid-run (exit code -6).
  - During the approach it stops if the lidar sees anything within 25 cm ahead, and the moment it loses sight of the object.
  - Ctrl-C stops the base.
- **Logs.** Every status message is also printed as `[status] {...}`.
