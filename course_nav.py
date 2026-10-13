#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Map-based navigation along the course: start line <-> end line, around the obstacle,
stopping with the blue/yellow object within the arm's reach.

Uses the running move_base + AMCL stack (spark_navigation amcl_demo_lidar) on the saved
SLAM map. The obstacle is avoided dynamically by move_base (saved map + live lidar), and
every leg after the first passes it through the same detour point, so the way back mirrors
the way out. No grasping here: with --handoff the script publishes its state on
/course_nav/status and waits on /course_nav/continue for the grasp/release script.
See COURSE_NAV_README.md.

Course frame: origin = START pose (map frame), x along the lane toward the end line,
y to the left.

    rosrun move2grasp course_nav.py set-start                    # after a new map: robot on the start line
    rosrun move2grasp course_nav.py localize                     # after a restart, no motion
    rosrun move2grasp course_nav.py round-trip                   # plain navigation
    rosrun move2grasp course_nav.py teach-reach --targets blue   # robot placed with blue in reach; no motion
    rosrun move2grasp course_nav.py to-end --targets blue        # stop with blue in reach
    rosrun move2grasp course_nav.py round-trip --targets blue        # there, stop at blue, back
    rosrun move2grasp course_nav.py round-trip --targets blue,yellow --handoff
"""

import argparse
import json
import math
import os
import threading
import time

import actionlib
import cv2
import numpy as np
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty

# Start pose in the saved test_map, found by matching the live lidar scan to the map
# with the robot on the start line facing the course. Re-measure if the map is re-saved.
DEFAULT_START = (-0.05, -0.25, -70.0)   # x, y (m), yaw (deg)

# Course start pose on the current map, saved by `set-start`.
START_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'course_nav_start.json')
# Taught stop line per colour (teach-reach): where the object's bottom edge and centre sit
# in the image when the arm can reach it.
DEFAULT_REACH_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  'course_nav_reach.json')

# HSV ranges the team already uses: blue from move2grasp/scripts/grasp_pro_new.py,
# yellow from spark_app/spark_sagittarius_carry/config/hsv.yaml.
COLORS = {
    'blue': ([95, 70, 60], [140, 255, 255]),
    'yellow': ([20, 65, 63], [35, 255, 255]),
}


def load_start():
    """Start pose saved by set-start for the current map: (x, y, yaw_deg) or None."""
    try:
        with open(START_FILE) as stream:
            d = json.load(stream)
        return (float(d['x']), float(d['y']), float(d['yaw_deg']))
    except (IOError, ValueError, KeyError):
        return None


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clip(value, low, high):
    return max(low, min(high, value))


class CourseNav:
    def __init__(self, args):
        self.args = args
        start = args.start or load_start() or DEFAULT_START
        self.start = (start[0], start[1], math.radians(start[2]))
        yaw = self.start[2]
        self.axis = (math.cos(yaw), math.sin(yaw))        # along the lane
        self.normal = (-math.sin(yaw), math.cos(yaw))     # to the left
        # With --targets the map only takes the robot past the box; the camera does the rest.
        d = args.handover_distance if args.targets else args.end_distance
        lat = 0.0 if args.targets else args.end_lateral
        self.end = (self.start[0] + d * self.axis[0] + lat * self.normal[0],
                    self.start[1] + d * self.axis[1] + lat * self.normal[1], yaw)
        self.apex = None            # (x, y) of the detour, remembered after the first pass
        self.trace = []
        self.running = True
        self.scan = None
        self.frame = None
        self.exclude = None
        self.wanted = []            # colours the camera callback is looking for
        self.target = None          # latest stable detection
        self.streak = 0
        self.last_blob = None
        self.calibration = None
        self.continue_msg = None
        self.bridge = CvBridge()
        self.lis = tf.TransformListener()
        self.load_map()

    # ------------------------------------------------------------------ setup
    def connect(self):
        self.lis.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(10))
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        rospy.Subscriber('/scan', LaserScan, lambda m: setattr(self, 'scan', m), queue_size=1)
        rospy.Subscriber(self.args.image_topic, Image, self.image_cb, queue_size=1)
        self.status_pub = rospy.Publisher('/course_nav/status', String, queue_size=10, latch=True)
        rospy.Subscriber('/course_nav/continue', String,
                         lambda m: setattr(self, 'continue_msg', m.data), queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if not self.client.wait_for_server(rospy.Duration(15)):
            raise RuntimeError('move_base is not running. Start the navigation stack '
                               '(OneKey 6 / spark_navigation amcl_demo_lidar) first.')
        threading.Thread(target=self.sampler, daemon=True).start()
        rospy.on_shutdown(self.shutdown)
        self.set_speed()

    def set_speed(self):
        """Raise the planner's and velocity smoother's speed limits for this session."""
        v, w = self.args.nav_speed, self.args.turn_speed
        try:
            from dynamic_reconfigure.client import Client
        except ImportError:
            rospy.logwarn('dynamic_reconfigure not available; keeping move_base speeds.')
            return
        wanted = {
            '/move_base/DWAPlannerROS': dict(max_vel_x=v, max_vel_trans=v, max_vel_theta=w,
                                             acc_lim_x=self.args.nav_accel),
            '/navigation_velocity_smoother': dict(speed_lim_v=v, speed_lim_w=w,
                                                  accel_lim_v=self.args.nav_accel),
        }
        for node, values in wanted.items():
            try:
                client = Client(node, timeout=5)
                current = client.get_configuration(timeout=5) or {}
                update = {k: val for k, val in values.items() if k in current}
                if update:
                    client.update_configuration(update)
                rospy.loginfo('Speed limits on %s: %s', node,
                              ', '.join('%s %.2f -> %.2f' % (k, current[k], update[k])
                                        for k in update))
            except Exception as exc:                    # node missing or no reconfigure
                rospy.logwarn('Could not raise speed limits on %s: %s', node, exc)

    def load_map(self):
        grid = rospy.wait_for_message('/map', OccupancyGrid, 15)
        self.W, self.H, self.res = grid.info.width, grid.info.height, grid.info.resolution
        self.ox, self.oy = grid.info.origin.position.x, grid.info.origin.position.y
        data = np.array(grid.data, dtype=np.int8).reshape(self.H, self.W)
        self.occupied = data > 50
        self.free = data == 0
        # distance (m) from each cell to the nearest occupied cell / to the nearest non-free cell
        self.dist_occ = cv2.distanceTransform((~self.occupied).astype(np.uint8),
                                              cv2.DIST_L2, 5) * self.res
        self.dist_free = cv2.distanceTransform(self.free.astype(np.uint8),
                                               cv2.DIST_L2, 5) * self.res

    def cell(self, x, y):
        i, j = int((x - self.ox) / self.res), int((y - self.oy) / self.res)
        return (i, j) if 0 <= i < self.W and 0 <= j < self.H else None

    def clearance(self, x, y):
        c = self.cell(x, y)
        return float(self.dist_free[c[1], c[0]]) if c else 0.0

    def obstacle_distance(self, x, y):
        c = self.cell(x, y)
        return float(self.dist_occ[c[1], c[0]]) if c else 9.0

    # ------------------------------------------------------------------ poses
    def check_tf(self):
        """Stop everything if AMCL stops publishing (map -> robot goes stale)."""
        try:
            latest = self.lis.getLatestCommonTime('map', 'base_footprint')
            age = (rospy.Time.now() - latest).to_sec()
        except tf.Exception:
            age = float('inf')
        if age > self.args.tf_timeout:
            try:
                self.client.cancel_all_goals()
            except Exception:
                pass
            self.stop()
            raise RuntimeError('Localization lost: map -> robot transform is %.1f s old. '
                               'Is AMCL still running? Robot stopped.' % age)

    def pose(self):
        (x, y, _), rot = self.lis.lookupTransform('map', 'base_footprint', rospy.Time(0))
        return x, y, tf.transformations.euler_from_quaternion(rot)[2]

    def lane_coords(self, x, y):
        dx, dy = x - self.start[0], y - self.start[1]
        return (dx * self.axis[0] + dy * self.axis[1],
                dx * self.normal[0] + dy * self.normal[1])

    def sampler(self):
        while self.running and not rospy.is_shutdown():
            try:
                self.trace.append(self.pose())
            except tf.Exception:
                pass
            time.sleep(0.2)

    # ----------------------------------------------------------------- checks
    def check_goal(self, name, pose):
        c = self.clearance(pose[0], pose[1])
        rospy.loginfo('%-5s goal x=%.2f y=%.2f yaw=%.0f deg, %.2f m clear of obstacles/unknown',
                      name, pose[0], pose[1], math.degrees(pose[2]), c)
        if c < self.args.goal_clearance:
            raise RuntimeError('%s goal is not in free mapped space with %.2f m clearance '
                               '(found %.2f m). Adjust --start/--end-distance/--end-lateral.'
                               % (name, self.args.goal_clearance, c))

    def check_localization(self):
        """Refuse to drive if the live scan does not match the map at the AMCL pose."""
        amcl = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, 5).pose
        if amcl.covariance[0] > 0.1 or amcl.covariance[7] > 0.1:
            raise RuntimeError('AMCL has not converged (position variance %.2f). Put the robot '
                               'on the start mark and run with --init-pose, or set 2D Pose '
                               'Estimate in RViz.' % max(amcl.covariance[0], amcl.covariance[7]))
        scan = rospy.wait_for_message('/scan', LaserScan, 5)
        x, y, yaw = self.pose()
        r = np.array(scan.ranges)
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        ok = np.isfinite(r) & (r > 0.25) & (r < 6.0)
        lx, ly = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        wx = x + math.cos(yaw) * lx - math.sin(yaw) * ly
        wy = y + math.sin(yaw) * lx + math.cos(yaw) * ly
        near = [self.obstacle_distance(px, py) < 0.10 for px, py in zip(wx, wy)]
        share = float(np.mean(near)) if near else 0.0
        rospy.loginfo('Localization check: %.0f%% of lidar points sit on mapped walls '
                      '(pose x=%.2f y=%.2f yaw=%.0f deg)', 100 * share, x, y, math.degrees(yaw))
        if share < self.args.min_match:
            raise RuntimeError('Live lidar does not match the map at the current pose '
                               '(%.0f%% < %.0f%%). The robot is not where AMCL thinks, or the '
                               'map no longer matches the room.'
                               % (100 * share, 100 * self.args.min_match))

    def set_initial_pose(self, pose=None):
        x, y, yaw = self.start if pose is None else pose
        pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
        rospy.sleep(1.0)
        m = PoseWithCovarianceStamped()
        m.header.frame_id, m.header.stamp = 'map', rospy.Time.now()
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z = math.sin(yaw / 2)
        m.pose.pose.orientation.w = math.cos(yaw / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.0025
        m.pose.covariance[35] = math.radians(3.0) ** 2
        pub.publish(m)
        rospy.loginfo('AMCL initial pose set to x=%.2f y=%.2f yaw=%.0f deg; letting it settle ...',
                      x, y, math.degrees(yaw))
        rospy.sleep(5.0)

    def scan_points(self):
        scan = rospy.wait_for_message('/scan', LaserScan, 5)
        r = np.array(scan.ranges)
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        ok = np.isfinite(r) & (r > 0.25) & (r < 6.0)
        return r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])

    def match(self, lx, ly, x, y, yaw):
        """Mean distance (m, capped at 1) from the scan points to mapped obstacles at a pose."""
        wx = x + math.cos(yaw) * lx - math.sin(yaw) * ly
        wy = y + math.sin(yaw) * lx + math.cos(yaw) * ly
        i = ((wx - self.ox) / self.res).astype(int)
        j = ((wy - self.oy) / self.res).astype(int)
        inside = (i >= 0) & (i < self.W) & (j >= 0) & (j < self.H)
        d = np.ones(len(lx))
        d[inside] = np.minimum(self.dist_occ[j[inside], i[inside]], 1.0)
        return float(d.mean()), float((d < 0.10).mean())

    def relocalize(self, whole_map=False):
        """Find the robot in the map from the lidar alone (no motion), then seed AMCL with it.

        Searches poses in and around the course (1 m behind the start to 1 m past the end,
        1.5 m either side, every heading)."""
        lx, ly = self.scan_points()
        rospy.loginfo('Searching the course area for the pose that best matches the lidar ...')
        if whole_map:
            js, is_ = np.where(self.dist_free >= 0.2)
            cells = [(self.ox + (i + 0.5) * self.res, self.oy + (j + 0.5) * self.res)
                     for j, i in zip(js, is_) if i % 3 == 0 and j % 3 == 0]
        else:
            cells = []
            for s in np.arange(-1.0, self.args.end_distance + 1.0, 0.1):
                for l in np.arange(-1.5, 1.51, 0.1):
                    x = self.start[0] + s * self.axis[0] + l * self.normal[0]
                    y = self.start[1] + s * self.axis[1] + l * self.normal[1]
                    if self.clearance(x, y) >= 0.15:
                        cells.append((x, y))
        candidates = []
        for x, y in cells:
            for yaw in np.radians(np.arange(-180, 180, 10)):
                candidates.append((self.match(lx, ly, x, y, yaw)[0], x, y, yaw))
        candidates.sort()
        refined = []
        for _, x0, y0, t0 in candidates[:20]:
            best = (9.0, x0, y0, t0)
            for dx in np.arange(-0.1, 0.101, 0.025):
                for dy in np.arange(-0.1, 0.101, 0.025):
                    for dt in np.radians(np.arange(-6, 6.1, 1.5)):
                        v = self.match(lx, ly, x0 + dx, y0 + dy, t0 + dt)[0]
                        if v < best[0]:
                            best = (v, x0 + dx, y0 + dy, t0 + dt)
            refined.append(best)
        refined.sort()
        _, x, y, yaw = refined[0]
        share = self.match(lx, ly, x, y, yaw)[1]
        rivals = [r for r in refined[1:] if math.hypot(r[1] - x, r[2] - y) > 0.4
                  or abs(wrap(r[3] - yaw)) > math.radians(25)]
        s, l = self.lane_coords(x, y)
        rospy.loginfo('Best match: x=%.2f y=%.2f yaw=%.0f deg (lane: %.2f m along, %.2f m left, '
                      'heading %+.0f deg from the lane); %.0f%% of lidar points on mapped walls',
                      x, y, math.degrees(yaw), s, l, math.degrees(wrap(yaw - self.start[2])),
                      100 * share)
        if rivals:
            r = rivals[0]
            rospy.loginfo('Next-best distinct pose: x=%.2f y=%.2f yaw=%.0f deg, %.0f%%',
                          r[1], r[2], math.degrees(r[3]), 100 * self.match(lx, ly, *r[1:])[1])
        if share < self.args.min_match:
            raise RuntimeError('No pose in the course area matches the lidar well enough '
                               '(%.0f%%); is the robot on the course?' % (100 * share))
        self.set_initial_pose((x, y, yaw))
        return x, y, yaw

    # ----------------------------------------------------------- detour memory
    def pinned_apex(self):
        """With --side left|right: a via point beside the mapped obstacle on the lane line."""
        if self.args.side == 'auto':
            return None
        sign = 1.0 if self.args.side == 'left' else -1.0
        hits = []
        for i in range(1, int(self.args.end_distance / 0.05) + 1):
            c = self.cell(self.start[0] + i * 0.05 * self.axis[0],
                          self.start[1] + i * 0.05 * self.axis[1])
            if c is not None and self.occupied[c[1], c[0]]:
                hits.append(i * 0.05)
        if not hits:
            rospy.loginfo('No mapped obstacle on the lane line; no pinned detour needed.')
            return None
        s = 0.5 * (hits[0] + hits[-1])
        for off in (0.55, 0.70, 0.85, 1.00):
            x = self.start[0] + s * self.axis[0] + sign * off * self.normal[0]
            y = self.start[1] + s * self.axis[1] + sign * off * self.normal[1]
            if self.clearance(x, y) >= self.args.goal_clearance:
                rospy.loginfo('Pinned detour: %s of the obstacle at %.2f m along the lane, '
                              'via x=%.2f y=%.2f', self.args.side, s, x, y)
                return (x, y)
        raise RuntimeError('No free space to pass the obstacle on the %s side.' % self.args.side)

    def learn_apex(self, segment):
        """Remember where the first pass swung out, so every later leg takes the same detour."""
        if self.apex is not None or not segment:
            return
        lat = [self.lane_coords(p[0], p[1])[1] for p in segment]
        k = max(range(len(lat)), key=lambda i: abs(lat[i]))
        if abs(lat[k]) < self.args.min_detour:
            return
        x, y = segment[k][0], segment[k][1]
        sign = 1.0 if lat[k] > 0 else -1.0
        # The robot's own track can sit inside move_base's keep-out zone around the obstacle,
        # which makes it an unplannable goal. Push the via point outward until it is clear.
        for _ in range(10):
            if self.obstacle_distance(x, y) >= self.args.apex_clearance:
                break
            x, y = x + 0.05 * sign * self.normal[0], y + 0.05 * sign * self.normal[1]
        if self.clearance(x, y) < self.args.goal_clearance:
            rospy.logwarn('No usable via point beside the obstacle; the planner will choose.')
            return
        self.apex = (x, y)
        rospy.loginfo('Detour remembered: %.2f m %s of the lane line (x=%.2f y=%.2f, '
                      '%.2f m from the obstacle)', abs(self.lane_coords(x, y)[1]),
                      'left' if sign > 0 else 'right', x, y, self.obstacle_distance(x, y))

    # --------------------------------------------------------------- driving
    def stop(self):
        for _ in range(4):
            self.cmd_pub.publish(Twist())
            rospy.sleep(0.04)

    def shutdown(self):
        self.running = False
        try:
            self.client.cancel_all_goals()
            self.stop()
        except Exception:
            pass

    def navigate(self, pose, label):
        x, y, yaw = pose
        for attempt in range(self.args.retries + 1):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x, goal.target_pose.pose.position.y = x, y
            goal.target_pose.pose.orientation = Quaternion(
                *tf.transformations.quaternion_from_euler(0, 0, yaw))
            self.client.send_goal(goal)
            rospy.loginfo('-> %s (x=%.2f y=%.2f)%s', label, x, y,
                          '' if attempt == 0 else '  [retry %d]' % attempt)
            deadline = time.time() + self.args.leg_timeout
            finished = False
            while not rospy.is_shutdown() and time.time() < deadline:
                if self.client.wait_for_result(rospy.Duration(0.2)):
                    finished = True
                    break
                self.check_tf()
            if finished:
                state = self.client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    return
                why = '%s (%s)' % (GoalStatus.to_string(state) if hasattr(GoalStatus, 'to_string')
                                   else state, self.client.get_goal_status_text())
            else:
                self.client.cancel_goal()
                why = 'timed out after %.0f s' % self.args.leg_timeout
            self.stop()
            rospy.logwarn('%s failed: %s', label, why)
            if attempt < self.args.retries:
                try:
                    rospy.ServiceProxy('/move_base/clear_costmaps', Empty)()
                except rospy.ServiceException:
                    pass
                rospy.sleep(1.0)
        raise RuntimeError('Could not reach %s after %d attempts.' % (label, self.args.retries + 1))

    def align(self, yaw):
        """Rotate in place to the lane heading (move_base only gets within about 8 deg)."""
        if not self.args.no_align:
            self.rotate_to(yaw)

    def rotate_to(self, yaw):
        end = time.time() + 20.0
        while not rospy.is_shutdown() and time.time() < end:
            self.check_tf()
            err = wrap(yaw - self.pose()[2])
            if abs(err) < math.radians(3.0):
                break
            msg = Twist()
            msg.angular.z = math.copysign(clip(abs(1.5 * err), 0.15, 0.6), err)
            self.cmd_pub.publish(msg)
            rospy.sleep(0.05)
        self.stop()

    def leg(self, name, goal):
        """Travel to `goal`, via the remembered detour if there is one."""
        n0 = len(self.trace)
        t0 = time.time()
        if self.apex is not None:
            # Face the way we are travelling at the via point, so there is no turn-around there.
            heading = math.atan2(goal[1] - self.apex[1], goal[0] - self.apex[0])
            self.navigate((self.apex[0], self.apex[1], heading), name + ' via detour')
        self.navigate(goal, name)
        self.align(goal[2])
        segment = self.trace[n0:]
        self.learn_apex(segment)
        lat = [self.lane_coords(p[0], p[1])[1] for p in segment]
        x, y, yaw = self.pose()
        s, l = self.lane_coords(x, y)
        rospy.loginfo('%s done in %.0f s: pose x=%.2f y=%.2f yaw=%.0f deg (lane: %.2f m along, '
                      '%.2f m left); max swing %.2f m; closest to mapped obstacle %.2f m',
                      name, time.time() - t0, x, y, math.degrees(yaw), s, l,
                      max(lat, key=abs) if lat else 0.0,
                      min((self.obstacle_distance(p[0], p[1]) for p in segment), default=9.0))

    # --------------------------------------------------------- arm's-reach approach
    def color_mask(self, image, color):
        lower, upper = COLORS[color]
        return cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV),
                           np.array(lower, np.uint8), np.array(upper, np.uint8))

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError:
            return
        self.frame = frame
        colors = self.wanted
        if not colors:
            return
        image = frame.copy()
        if self.exclude is not None:
            image[self.exclude > 0] = 0
        best = None
        for color in colors:
            mask = cv2.morphologyEx(self.color_mask(image, color), cv2.MORPH_OPEN,
                                    np.ones((5, 5), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.args.min_area or (best is not None and area <= best[0]):
                    continue
                m = cv2.moments(contour)
                if m['m01'] / m['m00'] < self.args.min_row:
                    continue        # far background (e.g. the yellow sofa), not on the lane floor
                _, top, _, height = cv2.boundingRect(contour)
                best = (area, color, m['m10'] / m['m00'], m['m01'] / m['m00'], top + height)
        if best is None:
            self.streak, self.last_blob = 0, None
            return
        _, color, u, v, bottom = best
        last = self.last_blob
        if last and last[0] == color and math.hypot(u - last[1], v - last[2]) < 15:
            self.streak += 1
        else:
            self.streak = 1
        self.last_blob = (color, u, v)
        if self.streak >= 4:
            self.target = (color, u, v, time.time(), msg.width, msg.height, bottom)

    def learn_static_colors(self):
        """Blue/yellow seen while no object is in view belongs to the robot (arm): ignore it."""
        union = None
        end = time.time() + 1.5
        while time.time() < end and not rospy.is_shutdown():
            if self.frame is not None:
                for color in COLORS:
                    # same speckle filter as detection: only solid blobs count as robot parts
                    mask = cv2.morphologyEx(self.color_mask(self.frame, color), cv2.MORPH_OPEN,
                                            np.ones((5, 5), np.uint8))
                    union = mask if union is None else cv2.bitwise_or(union, mask)
            rospy.sleep(0.1)
        if union is None:
            raise RuntimeError('No camera frames on %s' % self.args.image_topic)
        self.exclude = cv2.dilate(union, np.ones((25, 25), np.uint8))
        share = float(np.count_nonzero(self.exclude)) / self.exclude.size
        rospy.loginfo('Ignoring %.1f%% of the camera view (always blue/yellow with no '
                      'object in view: parts of the robot).', 100 * share)
        if share > 0.25:
            raise RuntimeError('Too much of the camera view is blue/yellow now; clear the '
                               'area in front of the robot and retry.')

    def front_blocked(self):
        scan = self.scan
        if scan is None:
            return False
        for i, r in enumerate(scan.ranges):
            if scan.range_min < r < scan.range_max and r < self.args.stop_distance \
                    and abs(wrap(scan.angle_min + i * scan.angle_increment)) < math.radians(20):
                return True
        return False

    def drive(self, v, heading):
        """One control tick: speed v (m/s, negative = back up) holding a map heading."""
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = clip(1.5 * wrap(heading - self.pose()[2]), -0.4, 0.4)
        self.cmd_pub.publish(msg)

    def rear_blocked(self):
        scan = self.scan
        if scan is None:
            return False
        for i, r in enumerate(scan.ranges):
            # ignore returns under 0.2 m: the robot's own body shows up there behind the lidar
            if 0.2 < r < 0.35 and scan.range_min < r < scan.range_max and \
                    abs(wrap(scan.angle_min + i * scan.angle_increment - math.pi)) < math.radians(20):
                return True
        return False

    def back_off(self):
        """Reverse straight back a little before turning for home, so turning in place near
        the objects cannot sweep into them."""
        dist = self.args.backoff
        if dist <= 0:
            return
        sx, sy, heading = self.pose()
        rospy.loginfo('Backing off %.2f m from the objects before turning for home ...', dist)
        try:
            while not rospy.is_shutdown():
                self.check_tf()
                x, y, _ = self.pose()
                if math.hypot(x - sx, y - sy) >= dist:
                    break
                if self.rear_blocked():
                    rospy.logwarn('Something is right behind the robot; stopped backing off.')
                    break
                self.drive(-0.10, heading)
                rospy.sleep(0.05)
        finally:
            self.stop()

    def load_reach(self):
        path = os.path.expanduser(self.args.reach_file)
        if not os.path.exists(path):
            return {}
        with open(path) as stream:
            return json.load(stream)

    def reach_for(self, color, reach):
        """Stop line for a colour: taught with teach-reach, else --stop-row at image centre."""
        if color in reach:
            return reach[color]
        return dict(u=320.0, bottom=float(self.args.stop_row))

    def fresh_target(self):
        t = self.target
        if t is None or time.time() - t[3] > 0.5:
            return None
        if (t[4], t[5]) != (640, 480):
            raise RuntimeError('Camera image is %dx%d; expected 640x480.' % (t[4], t[5]))
        return t

    def teach_reach(self, colors):
        """Robot already placed where the arm can grab the object: record where the object's
        bottom edge and centre sit in the image. That becomes the stop line. No motion."""
        self.wanted = list(colors)
        self.target, self.streak, self.last_blob = None, 0, None
        seen = {}
        end = time.time() + 4.0
        while time.time() < end and not rospy.is_shutdown():
            t = self.fresh_target()
            if t is not None:
                seen.setdefault(t[0], []).append(t)
                self.wanted = [c for c in colors if c not in seen] or list(colors)
                self.target = None
            rospy.sleep(0.1)
        self.wanted = []
        if not seen:
            raise RuntimeError('No %s object visible. If the arm-reach position hides it from '
                               'the camera, back the robot off until it is just visible and '
                               'teach there.' % '/'.join(colors))
        reach = self.load_reach()
        for color, ts in seen.items():
            reach[color] = dict(u=round(float(np.median([t[1] for t in ts])), 1),
                                bottom=round(float(np.median([t[6] for t in ts])), 1))
            rospy.loginfo('Taught %s: stop when its bottom edge reaches row %.0f (column %.0f).',
                          color, reach[color]['bottom'], reach[color]['u'])
        path = os.path.expanduser(self.args.reach_file)
        with open(path, 'w') as stream:
            json.dump(reach, stream, indent=2)
        rospy.loginfo('Saved %s', path)

    def approach(self, color):
        """After the box: steer at `color` and stop just short of it, while it is still in view.

        Stops when the object's bottom edge reaches its stop line (taught, or --stop-row), when
        the lidar sees something close ahead, or the moment the object is lost after being
        seen. Never drives on without seeing the object once it has been found."""
        reach = self.load_reach()
        self.target, self.streak, self.last_blob = None, 0, None
        self.wanted = list(COLORS) if color == 'any' else [color]
        sx, sy, heading = self.pose()
        px_per_rad = 640 / math.radians(self.args.camera_hfov)
        result, info, last_seen = 'not_found', {}, None
        self.event('APPROACHING', color=color)
        try:
            while not rospy.is_shutdown():
                self.check_tf()
                x, y, yaw = self.pose()
                travelled = math.hypot(x - sx, y - sy)
                t = self.fresh_target()
                if t is None:
                    if last_seen is not None:
                        self.stop()                     # found it before: never move blind
                        # standing still while it is out of sight, so waiting a little is safe
                        if time.time() - last_seen > self.args.lost_timeout:
                            result = 'lost'
                            break
                    elif travelled >= self.args.approach_max:
                        break
                    elif self.front_blocked():
                        result = 'blocked'
                        break
                    else:
                        self.drive(self.args.approach_speed, heading)   # not seen yet: search ahead
                    rospy.sleep(0.05)
                    continue
                last_seen = time.time()
                ref = self.reach_for(t[0], reach)
                info = dict(color=t[0], pixel=[round(t[1]), round(t[2])], bottom_row=round(t[6]),
                            stop_row=ref['bottom'], taught=t[0] in reach)
                if t[6] >= ref['bottom']:
                    result = 'in_reach'
                    break
                if self.front_blocked():
                    result = 'in_reach_lidar'
                    break
                if travelled >= self.args.approach_max:
                    result = 'too_far'
                    break
                near = t[6] >= ref['bottom'] - self.args.slow_rows
                speed = self.args.approach_slow if near else self.args.approach_speed
                heading = wrap(yaw + clip((ref['u'] - t[1]) / px_per_rad, -0.3, 0.3))
                self.drive(speed, heading)
                rospy.sleep(0.05)
        finally:
            self.wanted = []
            self.stop()
        x, y, yaw = self.pose()
        info.update(pose=[round(x, 3), round(y, 3), round(yaw, 3)],
                    travelled_m=round(math.hypot(x - sx, y - sy), 3))
        if result.startswith('in_reach'):
            self.event('IN_REACH', stopped_by='lidar' if result == 'in_reach_lidar' else 'camera',
                       **info)
            return True
        self.event('NOT_IN_REACH', reason=result, **info)
        return False

    # -------------------------------------------------------------- handoff
    def event(self, state, **extra):
        record = dict(state=state, **extra)
        self.status_pub.publish(String(data=json.dumps(record)))
        rospy.loginfo('[status] %s', json.dumps(record))

    def wait_for_continue(self, waiting_for):
        """Hand control to the grasp script and wait until it says to carry on."""
        self.continue_msg = None
        self.event('WAITING', waiting_for=waiting_for)
        end = time.time() + self.args.wait_timeout
        while not rospy.is_shutdown() and time.time() < end:
            msg = self.continue_msg
            if msg is not None:
                if msg.strip().lower() == 'abort':
                    raise RuntimeError('Aborted by /course_nav/continue.')
                rospy.loginfo('Continue received (%s).', msg)
                return
            rospy.sleep(0.1)
        raise RuntimeError('No /course_nav/continue within %.0f s; staying put.'
                           % self.args.wait_timeout)

    # --------------------------------------------------------------- mission
    def run(self):
        if self.args.command == 'set-start':
            self.check_goal = lambda *a: None
        self.check_goal('START', self.start)
        self.check_goal('END', self.end)
        self.apex = self.pinned_apex()
        targets = [t.strip() for t in self.args.targets.split(',') if t.strip()]
        for t in targets:
            if t not in list(COLORS) + ['any']:
                raise RuntimeError('Unknown target %r (blue, yellow or any).' % t)
        reach = self.load_reach()
        if self.args.command not in ('teach-reach', 'localize'):
            needed = sorted({c for t in targets for c in (list(COLORS) if t == 'any' else [t])})
            missing = [c for c in needed if c not in reach]
            if missing:
                rospy.logwarn('No taught stop line for %s: stopping when its bottom edge '
                              'reaches image row %d (--stop-row). To teach: place the robot '
                              'where the arm can grab it, run course_nav.py teach-reach '
                              '--targets %s', ', '.join(missing), self.args.stop_row,
                              ','.join(missing))
        if self.args.dry_run:
            rospy.loginfo('Dry run: %s, targets %s; nothing moved.', self.args.command,
                          targets or 'none')
            return
        if self.args.command == 'localize':
            self.relocalize()
            self.check_localization()
            return
        if self.args.command == 'set-start':
            x, y, yaw = self.relocalize(whole_map=True)
            self.check_localization()
            with open(START_FILE, 'w') as stream:
                json.dump(dict(x=round(x, 3), y=round(y, 3),
                               yaw_deg=round(math.degrees(yaw), 1)), stream, indent=2)
            rospy.loginfo('Saved the course start (x=%.2f y=%.2f yaw=%.0f deg) to %s. The lane runs '
                          'the way the robot is facing.', x, y, math.degrees(yaw), START_FILE)
            return
        if self.args.command == 'teach-reach':
            self.teach_reach(targets or list(COLORS))
            return
        if self.args.init_pose:
            self.set_initial_pose()
        if not self.args.no_loc_check:
            self.check_localization()
        if self.args.command == 'approach':
            for t in targets:
                self.approach(t)
            return
        x, y, _ = self.pose()
        if targets:
            if math.hypot(x - self.end[0], y - self.end[1]) > 1.0:
                self.learn_static_colors()
            else:
                rospy.logwarn('Starting near the objects: not learning which colours belong to '
                              'the robot (objects may be in view).')
        if self.args.command != 'to-start' and math.hypot(x - self.start[0], y - self.start[1]) > 0.6:
            rospy.logwarn('Robot is %.1f m from the start pose.',
                          math.hypot(x - self.start[0], y - self.start[1]))
        trips = targets if targets else [None] * self.args.loops
        for i, target in enumerate(trips):
            if len(trips) > 1:
                rospy.loginfo('=== trip %d/%d%s ===', i + 1, len(trips),
                              ': ' + target if target else '')
            if self.args.command in ('to-end', 'round-trip'):
                self.event('TO_END', trip=i + 1, target=target)
                self.leg('END', self.end)
                self.event('AT_END', trip=i + 1)
                if target:
                    self.approach(target)
                if self.args.handoff:
                    self.wait_for_continue('grasp')
            if self.args.command in ('to-start', 'round-trip'):
                x, y, _ = self.pose()
                if self.lane_coords(x, y)[0] > self.args.handover_distance + 0.3:
                    self.back_off()                     # parked at the objects
                self.event('TO_START', trip=i + 1)
                self.leg('START', self.start)
                self.event('AT_START', trip=i + 1)
                if self.args.handoff:
                    self.wait_for_continue('release')
        self.event('DONE')


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('command', nargs='?', default='round-trip',
                   choices=['to-end', 'to-start', 'round-trip', 'localize', 'set-start',
                            'teach-reach', 'approach'],
                   help='set-start: robot on the start line facing the lane, after a new map; '
                        'finds it anywhere on the map and saves that as the course start. '
                        'localize: find the robot on the map from the lidar and seed AMCL. '
                        'teach-reach: robot placed where the arm can grab the object; records '
                        'its stop line. (Neither moves the robot.) approach: only the final '
                        'camera approach to --targets')
    p.add_argument('--start', nargs=3, type=float, metavar=('X', 'Y', 'YAW_DEG'),
                   default=None, help='start pose in the map frame (default: the one saved by '
                                      'set-start)')
    p.add_argument('--end-distance', type=float, default=2.75,
                   help='metres from the start along the lane to where the robot parks '
                        'in front of the objects (default 2.75)')
    p.add_argument('--handover-distance', type=float, default=2.3,
                   help='with --targets: metres along the lane where map navigation stops '
                        '(just past the box) and the camera approach takes over')
    p.add_argument('--end-lateral', type=float, default=0.0,
                   help='sideways offset of the end pose, left positive (aim at one object)')
    p.add_argument('--side', choices=['auto', 'left', 'right'], default='auto',
                   help='pin the detour side; auto lets the planner choose once, then repeats it')
    p.add_argument('--init-pose', action='store_true',
                   help='tell AMCL the robot is on the start mark (do this once after a restart)')
    p.add_argument('--loops', type=int, default=1, help='round trips when no --targets')
    p.add_argument('--retries', type=int, default=2)
    p.add_argument('--nav-speed', type=float, default=0.3,
                   help='move_base top speed for this run, m/s (stack default 0.1)')
    p.add_argument('--turn-speed', type=float, default=1.0, help='move_base top turn rate, rad/s')
    p.add_argument('--nav-accel', type=float, default=0.6, help='m/s^2')
    p.add_argument('--approach-speed', type=float, default=0.12,
                   help='camera approach speed while far from the stop line, m/s')
    p.add_argument('--approach-slow', type=float, default=0.04,
                   help='camera approach speed near the stop line, m/s')
    p.add_argument('--slow-rows', type=int, default=120,
                   help='slow down when the object is this many image rows from its stop line')
    p.add_argument('--leg-timeout', type=float, default=240.0)
    p.add_argument('--goal-clearance', type=float, default=0.35)
    p.add_argument('--min-detour', type=float, default=0.25)
    p.add_argument('--apex-clearance', type=float, default=0.5,
                   help='keep the remembered detour point this far (m) from mapped obstacles')
    p.add_argument('--min-match', type=float, default=0.5)
    p.add_argument('--tf-timeout', type=float, default=1.5,
                   help='stop if the map -> robot transform is older than this (s)')
    p.add_argument('--no-loc-check', action='store_true')
    p.add_argument('--no-align', action='store_true')
    p.add_argument('--targets', default='',
                   help="comma list, one trip each, e.g. blue,yellow: at the end line, approach "
                        "that colour until it is within the arm's reach (blue, yellow, any)")
    p.add_argument('--handoff', action='store_true',
                   help='wait for /course_nav/continue at the end (grasp) and at the start '
                        '(release) before moving on')
    p.add_argument('--wait-timeout', type=float, default=600.0)
    p.add_argument('--backoff', type=float, default=0.2,
                   help='reverse this far (m) from the objects before turning for home')
    p.add_argument('--lost-timeout', type=float, default=3.0,
                   help='approach: seconds to wait (stopped) for a lost object to reappear')
    p.add_argument('--approach-max', type=float, default=2.2,
                   help='furthest the approach may creep past the end pose (m)')
    p.add_argument('--stop-distance', type=float, default=0.25,
                   help='lidar: stop the approach if anything is this close ahead (m)')
    p.add_argument('--reach-file', default=DEFAULT_REACH_FILE,
                   help='taught reach per colour (written by teach-reach)')
    p.add_argument('--stop-row', type=int, default=400,
                   help="untaught colours: stop when the object's bottom edge reaches this "
                        "image row (480 = bottom of the image)")
    p.add_argument('--pixel-tolerance', type=float, default=12.0)
    p.add_argument('--camera-hfov', type=float, default=60.0, help='degrees')
    p.add_argument('--image-topic', default='/camera/rgb/image_raw')
    p.add_argument('--min-area', type=float, default=150.0)
    p.add_argument('--min-row', type=int, default=150,
                   help='ignore colour blobs above this image row (far background)')
    p.add_argument('--dry-run', action='store_true',
                   help='check the goals and the map, move nothing')
    return p.parse_args(rospy.myargv()[1:])


if __name__ == '__main__':
    args = parse_args()
    rospy.init_node('course_nav', anonymous=True)
    nav = None
    try:
        nav = CourseNav(args)
        nav.connect()
        nav.run()
    except (RuntimeError, tf.Exception, rospy.ROSException) as exc:
        rospy.logerr('Navigation stopped: %s', exc)
        if nav is not None and hasattr(nav, 'status_pub'):
            nav.event('FAILED', error=str(exc))
            rospy.sleep(0.3)
        raise SystemExit(1)
    except rospy.ROSInterruptException:
        pass
    finally:
        if nav is not None:
            nav.shutdown()
