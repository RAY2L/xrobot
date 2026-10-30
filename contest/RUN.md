# Run course_nav (quick reference)

Robot: `ssh spark@10.43.20.145` (password `spark`), or a terminal on the robot.

## One command

Put the robot on the start line, centred, **facing down the lane**, then:

```bash
~/spark_noetic/src/3rd_app/move2grasp/scripts/run.sh
```

It starts whatever isn't running (navigation stack, arm driver), localizes, and runs
`round-trip` (trip 1 blue, trip 2 yellow). It refuses to drive if the robot can't be
found on the map, and refuses to start if another program already holds ROS.
**Ctrl-C** stops the robot.

```bash
./run.sh --check            # start + localize only, robot does not move
./run.sh to-start           # drive back to the start line
./run.sh round-trip --targets none
```

Only one person can use the robot's ROS at a time. If `run.sh` says the stack is
incomplete, someone else's programs are running: ask them, don't kill them.

The manual steps follow, if you want to run each part yourself.

## 1. Check the stack is running (start it only if not)

```bash
source ~/spark_noetic/devel/setup.bash
rosnode list | grep -E "amcl|move_base"      # both should appear
```

If nothing is running (e.g. after a reboot or power loss), start **one** copy:

```bash
roslaunch spark_navigation amcl_demo_lidar.launch camera_type_tel:=astra lidar_type_tel:=ydlidar_g2 start_camera:=true
```

Second terminal (stops `move_base` spinning in place when lost):

```bash
source ~/spark_noetic/devel/setup.bash
rosparam set /move_base/recovery_behavior_enabled false
rosparam set /move_base/clearing_rotation_allowed false
rosnode kill /move_base
rosrun move_base move_base cmd_vel:=/navigation_velocity_smoother/raw_cmd_vel odom:=odom scan:=scan __name:=move_base
```

Never start a second stack: they kill each other's nodes.

## 2. Arm out of the way

Keep the arm tucked and nothing within 25 cm in front of the robot, or the camera
approach stops (`blocked`). Starting the arm driver also moves it clear:

```bash
roslaunch swiftpro pro_control_nomoveit.launch
```

## 3. Place the robot

On the start line, centred, **facing down the lane** toward the objects.

## 4. Localize (no motion)

```bash
cd ~/spark_noetic/src/3rd_app/move2grasp/scripts
python3 course_nav.py localize
```

Want a match of **75% or more**. Low means the robot is off the course. Run it again any
time the robot was carried or bumped.

## 5. Run

```bash
python3 course_nav.py round-trip
```

Two trips, about 2 minutes: trip 1 stops just short of **blue**, trip 2 just short of
**yellow**, each returns to the start line.

## Other commands

| Command | Does |
|---|---|
| `python3 course_nav.py --dry-run` | Checks map and goals, no motion |
| `python3 course_nav.py to-start` | Drive back to the start line from anywhere |
| `python3 course_nav.py round-trip --targets none` | Map only, no camera approach |
| `python3 course_nav.py round-trip --targets yellow,blue` | Other order |
| `python3 course_nav.py --help` | All options |

## Stop

**Ctrl-C** in that terminal: the robot stops.

## If it refuses or fails

| Message | Meaning / fix |
|---|---|
| `Live lidar does not match the map` | Robot isn't where it thinks: put it on the start line, run `localize` |
| `AMCL has not converged` | Run `localize` |
| `NOT_IN_REACH ... blocked` | Something within 25 cm ahead (often the arm): tuck it, clear the area |
| `Localization lost` | AMCL stopped: restart the stack (step 1) |
