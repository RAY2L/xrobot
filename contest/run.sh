#!/usr/bin/env bash
# One command to run the course: ./run.sh [command] [course_nav options]
#
#   ./run.sh                   start what is missing, localize, then round-trip (blue, yellow)
#   ./run.sh to-start          ... then drive back to the start line instead
#   ./run.sh round-trip --targets none
#   ./run.sh --check           start what is missing and localize only; the robot does not move
#
# Put the robot on the start line, centred, facing down the lane, before running.
# Ctrl-C stops the robot. The navigation stack and arm driver are left running.

source /opt/ros/noetic/setup.bash
source "$HOME/spark_noetic/devel/setup.bash"
set -u
cd "$(dirname "$(readlink -f "$0")")"

CHECK=0
ARGS=()
for a in "$@"; do
  if [[ "$a" == "--check" ]]; then CHECK=1; else ARGS+=("$a"); fi
done
[[ ${#ARGS[@]} -eq 0 ]] && ARGS=(round-trip)

say() { printf '\n== %s\n' "$*"; }
die() { printf '\nERROR: %s\n' "$*" >&2; exit 1; }
node_up() { rosnode list 2>/dev/null | grep -qx "$1"; }
wait_node() {   # wait_node /node seconds
  local end=$((SECONDS + $2))
  while (( SECONDS < end )); do node_up "$1" && return 0; sleep 1; done
  return 1
}
start_move_base() {
  rosparam set /move_base/recovery_behavior_enabled false      # no spinning in place when lost
  rosparam set /move_base/clearing_rotation_allowed false
  rosnode kill /move_base >/dev/null 2>&1; sleep 2
  nohup /opt/ros/noetic/lib/move_base/move_base cmd_vel:=/navigation_velocity_smoother/raw_cmd_vel \
      odom:=odom scan:=scan __name:=move_base > "$HOME/.ros/course_nav_move_base.log" 2>&1 &
  wait_node /move_base 20 || die "move_base did not start (see ~/.ros/course_nav_move_base.log)"
  sleep 3
}

say "navigation stack"
if rosnode list >/dev/null 2>&1; then
  node_up /amcl && node_up /move_base && node_up /spark_base_server || \
    die "A ROS master is running but the navigation stack is incomplete (another program owns it).
       Stop everything ROS first, then run this again."
  echo "already running"
  if [[ "$(rosparam get /move_base/recovery_behavior_enabled 2>/dev/null)" != "False" ]]; then
    echo "turning off move_base spin-in-place recovery"; start_move_base
  fi
else
  echo "starting it (about 40 s)"
  mkdir -p "$HOME/.ros"
  nohup roslaunch spark_navigation amcl_demo_lidar.launch camera_type_tel:=astra \
      lidar_type_tel:=ydlidar_g2 start_camera:=true > "$HOME/.ros/course_nav_stack.log" 2>&1 &
  wait_node /move_base 60 || die "navigation did not come up (see ~/.ros/course_nav_stack.log)"
  sleep 8
  start_move_base
fi
timeout 6 rostopic hz /scan 2>&1 | grep -m1 average >/dev/null || die "no lidar data on /scan"

say "arm driver (moves the arm out of the way)"
if node_up /swiftpro_write_node; then
  echo "already running"
else
  nohup roslaunch swiftpro pro_control_nomoveit.launch > "$HOME/.ros/arm_driver.log" 2>&1 &
  wait_node /swiftpro_write_node 20 && sleep 5 || echo "WARNING: arm driver did not start; the arm may be in front of the lidar"
fi

say "localize (no motion)"
python3 course_nav.py localize || die "Could not find the robot on the map. Put it on the start line, facing down the lane."

if (( CHECK )); then
  say "check done: everything is up and the robot is localized"
  exit 0
fi

say "course_nav.py ${ARGS[*]}"
exec python3 course_nav.py "${ARGS[@]}"
