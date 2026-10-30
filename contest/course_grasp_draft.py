#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Reach down, grasp and release the blue or yellow object with the SwiftPro arm.

One file, built to plug into course_nav.py. It owns the ARM (/position_write_topic,
/pump_topic) and never drives the base: it publishes no /cmd_vel and sends no move_base
goals. course_nav.py owns the base.

The grasp is CONDITION-BASED. The Spark arm has no suction or contact sensor (its state
topic only echoes our commands), so the camera is the sensor:
  1. Tuck the arm out of view and find the object in the image.
  2. Turn its pixel into an arm x/y (camera model learned by `calibrate`, else ~/thefile.txt,
     else a fixed spot) and lower ALL THE WAY down onto it, pump on.
  3. Lift and tuck the arm, look again: object gone from the floor = grasped.
     Still there  -> pump off, try again a little deeper and nudged sideways.
     Moved        -> follow it and try again.
  It reports success only when the object has really left the floor.

Two ways to be triggered while it runs (command `serve`, the default):
  1. Spark's own protocol (as move2grasp/scripts/grasp_pro_new.py and move.py):
       /grasp         std_msgs/String  "1" grasp, "0" release.  "1:yellow" / "1:blue" picks the
                                       colour; otherwise ROS param /grasp_color, else --color.
       /grasp_status  std_msgs/String  "1" grasped, "-1" failed, "0" released.
  2. course_nav.py's handoff (--follow-nav): on /course_nav/status WAITING {"waiting_for":
     "grasp"} it grasps the colour of the last IN_REACH record (looking near its pixel), on
     "release" it releases, then publishes /course_nav/continue. Progress is also published
     on /course_grasp/status (GRASPED / RELEASED / FAILED, JSON).

Commands (arm only; nothing here moves the robot):
    python3 course_grasp.py calibrate              # learn camera -> arm from the arm tip
    python3 course_grasp.py show                   # where would the arm go? (no motion)
    python3 course_grasp.py grasp --color yellow   # one-off grasp
    python3 course_grasp.py release                # swing out to the left and let go
    python3 course_grasp.py cycle --color yellow   # grasp, then swing left and release
    python3 course_grasp.py serve --follow-nav     # run beside course_nav.py --handoff
    python3 course_grasp.py set-spot --color blue --x 200 --y 0     # fixed-aim fallback
    python3 course_grasp.py move --x 120 --y 0 --z 35
    python3 course_grasp.py ... --dry-run          # print the arm sequence, publish nothing

`calibrate` needs the arm's workspace clear of objects: it moves the arm tip around at two
heights and watches the yellow-orange piece on the tip. Release swings the arm out to the
robot's left and drops the object from about 60 mm; keep that side clear.
"""

import argparse
import json
import math
import os
import threading
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from swiftpro.msg import position, status
except ImportError:                                       # same message definitions
    from spark_carry_object.msg import position, status

# HSV ranges (OpenCV, H 0-179) the team already uses:
#   blue   move2grasp/scripts/grasp_pro_new.py  LowerBlue / UpperBlue
#   yellow spark_app/spark_sagittarius_carry/config/hsv.yaml  (h 20-35, s 65-255, v 63-255)
COLORS = {
    'blue': ([95, 70, 60], [140, 255, 255]),
    'yellow': ([20, 65, 63], [35, 255, 255]),
}
# The yellow-orange piece on the arm tip, used to find the tip in the image.
TIP_HSV = ([5, 70, 70], [38, 255, 255])

# Arm poses in mm (x forward, y left, z up). Spark's calibration touches the floor at z=-110,
# and grasp_pro_new.py grasps its ~6 cm blue block at z=-50, so a short object needs lower.
READY = (120.0, 0.0, 35.0)
CARRY = (20.0, 150.0, 130.0)     # tucked in: out of the camera's view and clear of the lidar
DROP = (160.0, 150.0, 60.0)      # release pose, swung out to the robot's LEFT
GRASP_Z = {'blue': -50.0, 'yellow': -78.0}    # cup height touching the top of the object
FLOOR_Z = -108.0                 # never go below this
ENVELOPE = dict(x=(0.0, 320.0), y=(-250.0, 250.0), z=(-115.0, 170.0), reach=(60.0, 300.0))
CAL_Z = (30.0, -25.0)            # tip heights used by `calibrate`, above every object
# Where to try, in arm mm, if the first attempt does not lift the object.
NUDGES = [(0, 0), (8, 0), (-8, 0), (0, 8), (0, -8), (12, 12), (-12, -12), (12, -12)]

HERE = os.path.dirname(os.path.abspath(__file__))
SPOT_FILE = os.path.join(HERE, 'course_grasp_spots.json')    # taught fixed grasp spots
CAL_FILE = os.path.join(HERE, 'course_grasp_cal.json')       # camera model from `calibrate`


class ArmError(RuntimeError):
    def __init__(self, text, fatal=False):
        super().__init__(text)
        self.fatal = fatal              # True: do not let the robot carry on


# ------------------------------------------------------------ camera model
def _norm_xyz(p):
    return np.array([p[0] / 100.0, p[1] / 100.0, p[2] / 100.0, 1.0])


def fit_projection(pairs):
    """3x4 projection P with pixel ~ P [x y z 1] (mm and px, normalised), from tip sightings."""
    rows = []
    for (x, y, z), (u, v) in pairs:
        X, un, vn = _norm_xyz((x, y, z)), (u - 320.0) / 320.0, (v - 240.0) / 320.0
        rows.append(list(X) + [0, 0, 0, 0] + list(-un * X))
        rows.append([0, 0, 0, 0] + list(X) + list(-vn * X))
    _, _, vt = np.linalg.svd(np.array(rows))
    return vt[-1].reshape(3, 4)


def project(P, x, y, z):
    h = P @ _norm_xyz((x, y, z))
    return h[0] / h[2] * 320.0 + 320.0, h[1] / h[2] * 320.0 + 240.0


def unproject(P, u, v, z):
    """Arm (x, y) in mm for a pixel that lies at height z."""
    un, vn = (u - 320.0) / 320.0, (v - 240.0) / 320.0
    r0, r1 = P[0] - un * P[2], P[1] - vn * P[2]
    a = np.array([[r0[0], r0[1]], [r1[0], r1[1]]])
    b = -np.array([r0[2] * z / 100.0 + r0[3], r1[2] * z / 100.0 + r1[3]])
    xy = np.linalg.solve(a, b)
    return float(xy[0] * 100.0), float(xy[1] * 100.0)


class Grasper:
    def __init__(self, args):
        self.args = args
        self.dry = args.dry_run
        self.holding = None                 # colour currently on the suction cup
        self.busy = threading.Lock()
        self.cancelled = False
        self.bridge = CvBridge()
        self.frame = None
        self.frame_time = 0.0
        self.wanted = None
        self.target = None                  # (u, v, area, time) latest stable detection
        self.streak, self.last_blob = 0, None
        self.last_in_reach = {}
        self.anchor = None                  # pixel course_nav stopped at: look near it
        self.at = None                      # last commanded arm position
        self.z = dict(GRASP_Z, blue=args.blue_z, yellow=args.yellow_z)
        if self.dry:
            return
        self.arm_pub = rospy.Publisher('position_write_topic', position, queue_size=10)
        self.pump_pub = rospy.Publisher('pump_topic', status, queue_size=2)
        self.status_pub = rospy.Publisher('grasp_status', String, queue_size=1)
        self.cont_pub = rospy.Publisher('/course_nav/continue', String, queue_size=1)
        self.report_pub = rospy.Publisher('/course_grasp/status', String, queue_size=10, latch=True)
        rospy.Subscriber(args.image_topic, Image, self.on_image, queue_size=1)
        rospy.on_shutdown(self.shutdown)

    # ------------------------------------------------------------------- arm
    def report(self, state, **extra):
        """Latched JSON on /course_grasp/status: GRASPED, RELEASED or FAILED."""
        if not self.dry:
            self.report_pub.publish(String(data=json.dumps(dict(state=state, **extra))))

    def say(self, text, *a):
        if self.dry:
            print('[dry-run] ' + (text % a if a else text))
        else:
            rospy.loginfo(text, *a)

    def check_envelope(self, x, y, z):
        r = math.hypot(x, y)                    # SwiftPro reach is about 50-320 mm from its base
        if not (ENVELOPE['x'][0] <= x <= ENVELOPE['x'][1] and ENVELOPE['y'][0] <= y <= ENVELOPE['y'][1]
                and ENVELOPE['z'][0] <= z <= ENVELOPE['z'][1]
                and ENVELOPE['reach'][0] <= r <= ENVELOPE['reach'][1]):
            raise ArmError('Arm target (%.0f, %.0f, %.0f) mm is outside the safe envelope.' % (x, y, z))

    def wait_for_arm(self):
        if self.dry:
            return
        end = time.time() + 10.0
        while time.time() < end and not rospy.is_shutdown():
            if self.arm_pub.get_num_connections() and self.pump_pub.get_num_connections():
                return
            time.sleep(0.1)
        raise ArmError('The arm driver is not running (nothing listens on position_write_topic / '
                       'pump_topic). Start: roslaunch swiftpro pro_control_nomoveit.launch',
                       fatal=True)

    def goto(self, x, y, z, label='', wait=None):
        """Move and wait until the arm has got there. The SwiftPro takes 3-4 s for a long move
        and gives no feedback, so the wait grows with the distance travelled."""
        self.check_envelope(x, y, z)
        dist = 400.0 if self.at is None else math.dist(self.at, (x, y, z))
        travel = self.args.settle + dist / self.args.arm_speed
        self.say('arm -> (%.0f, %.0f, %.0f) mm %s', x, y, z, label)
        if self.dry:
            return
        if self.cancelled:
            raise ArmError('Cancelled.')
        msg = position()
        msg.x, msg.y, msg.z = float(x), float(y), float(z)
        self.arm_pub.publish(msg)
        self.at = (x, y, z)
        self.sleep(max(travel, 0.0 if wait is None else wait))

    def pump(self, on):
        self.say('pump %s', 'ON' if on else 'off')
        if self.dry:
            return
        msg = status()
        msg.status = 1 if on else 0
        self.pump_pub.publish(msg)
        self.sleep(self.args.pump_wait)

    def sleep(self, seconds):
        end = time.time() + seconds
        while time.time() < end and not rospy.is_shutdown():
            if self.cancelled:
                raise ArmError('Cancelled.')
            time.sleep(0.05)

    def shutdown(self):
        # Never drop a held object on shutdown: leave the pump as it is.
        self.cancelled = True

    # ---------------------------------------------------------- aiming data
    def thefile(self):
        """(a, b, c, d) from ~/thefile.txt, or None if it is missing or malformed."""
        path = os.path.expanduser(self.args.calibration_file)
        try:
            with open(path) as stream:
                values = [float(v) for v in stream.read().split()]
        except (IOError, ValueError):
            return None
        return values if len(values) == 4 and all(math.isfinite(v) for v in values) else None

    @staticmethod
    def usable(values):
        return values is not None and abs(values[0]) >= 0.01 and abs(values[2]) >= 0.01

    def camera_model(self):
        """3x4 matrix from `calibrate`, or None."""
        try:
            with open(CAL_FILE) as stream:
                return np.array(json.load(stream)['P'])
        except (IOError, ValueError, KeyError):
            return None

    def aim_mode(self):
        mode = self.args.aim
        if mode == 'auto':
            if self.camera_model() is not None:
                return 'camera'
            return 'calibration' if self.usable(self.thefile()) else 'fixed'
        if mode == 'camera' and self.camera_model() is None:
            raise ArmError('No camera model yet: run `course_grasp.py calibrate` first.')
        if mode == 'calibration' and not self.usable(self.thefile()):
            raise ArmError('%s is missing or uncalibrated (slopes are 0).' % self.args.calibration_file)
        return mode

    def fixed_spot(self, color):
        """Taught arm x, y for the colour; else Spark's uncalibrated default (a1, b1)."""
        spots = {}
        if os.path.exists(SPOT_FILE):
            with open(SPOT_FILE) as stream:
                spots = json.load(stream)
        if color in spots:
            return float(spots[color]['x']), float(spots[color]['y']), 'taught'
        cal = self.thefile()
        if cal is not None and not self.usable(cal):
            return cal[1], cal[3], 'Spark default (thefile.txt a1, b1)'
        raise ArmError('No fixed grasp spot for %s: run `course_grasp.py set-spot --color %s '
                       '--x X --y Y`, or `calibrate`.' % (color, color))

    def arm_xy(self, mode, color, u=None, v=None):
        z = self.z[color]
        if mode == 'camera':
            x, y = unproject(self.camera_model(), u, v, z)
            return x, y, 'camera model at z=%.0f' % z
        if mode == 'calibration':
            a, b, c, d = self.thefile()
            return a * v + b, c * u + d, '~/thefile.txt'
        return self.fixed_spot(color)

    # --------------------------------------------------------------- vision
    def mask(self, image, color=None, rng=None):
        lower, upper = rng if rng else COLORS[color]
        m = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV),
                        np.array(lower, np.uint8), np.array(upper, np.uint8))
        return cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

    def blobs(self, image, color, min_area=None):
        """[(area, u, v)] for the colour, biggest first."""
        contours, _ = cv2.findContours(self.mask(image, color), cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in contours:
            area = cv2.contourArea(c)
            if area >= (self.args.min_area if min_area is None else min_area):
                m = cv2.moments(c)
                out.append((area, m['m10'] / m['m00'], m['m01'] / m['m00']))
        return sorted(out, reverse=True)

    def on_image(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.frame_time = time.time()
        except CvBridgeError:
            return
        color = self.wanted
        if color is None:
            return
        best = None
        for area, u, v in self.blobs(self.frame, color):
            a = self.anchor
            if a is not None and math.hypot(u - a[0], v - a[1]) > self.args.anchor_radius:
                continue                    # not the object course_nav stopped at
            best = (area, u, v)
            break
        if best is None:
            self.streak, self.last_blob = 0, None
            return
        area, u, v = best
        last = self.last_blob
        self.streak = self.streak + 1 if last and math.hypot(u - last[0], v - last[1]) <= 6 else 1
        self.last_blob = (u, v)
        if self.streak >= self.args.stable_frames:
            self.target = (u, v, area, time.time())

    def fresh_frame(self, settle=0.4):
        """A frame captured after `settle` seconds from now."""
        self.sleep(settle)
        end = time.time() + 3.0
        while time.time() < end and (self.frame is None or self.frame_time < time.time() - 0.5):
            time.sleep(0.05)
        if self.frame is None or time.time() - self.frame_time > 1.5:
            raise ArmError('No camera frames on %s.' % self.args.image_topic)
        return self.frame.copy()

    def find_object(self, color, anchor=None):
        """Stable pixel (u, v, area) of the colour, or an error after --find-timeout seconds."""
        self.fresh_frame(0.0)
        self.target, self.streak, self.last_blob, self.wanted = None, 0, None, color
        self.anchor = anchor
        end = time.time() + self.args.find_timeout
        try:
            while time.time() < end and not rospy.is_shutdown():
                t = self.target
                if t is not None and time.time() - t[3] < 0.5:
                    return t[0], t[1], t[2]
                time.sleep(0.05)
        finally:
            self.wanted = None
            self.anchor = None
        raise ArmError('%s object not visible to the camera.' % color)

    def tip_pixel(self, reference, current):
        """Pixel of the yellow-orange tip piece: it is what changed against the tucked-arm frame."""
        changed = cv2.absdiff(current, reference).max(axis=2) > 50
        tip = cv2.bitwise_and(self.mask(current, rng=TIP_HSV), changed.astype(np.uint8) * 255)
        tip = cv2.morphologyEx(tip, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(tip, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = max(contours, key=cv2.contourArea, default=None)
        if best is None or cv2.contourArea(best) < 120:
            return None
        m = cv2.moments(best)
        return m['m10'] / m['m00'], m['m01'] / m['m00']

    # ---------------------------------------------------------- calibration
    def calibrate(self):
        """Learn camera -> arm by moving the tip around and watching it. Workspace must be clear."""
        self.wait_for_arm()
        grid = [(x, y) for x in (150.0, 200.0, 250.0) for y in (-80.0, 0.0, 80.0, 140.0)]
        self.goto(*CARRY, label='tuck: reference view', wait=2.5)
        reference = self.fresh_frame(1.0)
        pairs = []
        for z in CAL_Z:
            for x, y in grid:
                self.goto(x, y, z, label='calibration pose', wait=2.0)
                seen = []
                for _ in range(4):
                    tip = self.tip_pixel(reference, self.fresh_frame(0.15))
                    if tip:
                        seen.append(tip)
                if len(seen) >= 3:
                    u, v = np.median([s[0] for s in seen]), np.median([s[1] for s in seen])
                    pairs.append(((x, y, z), (float(u), float(v))))
                    self.say('   tip seen at pixel (%.0f, %.0f)', u, v)
                else:
                    self.say('   tip not seen here, skipped')
        self.goto(*CARRY, label='tuck', wait=2.5)
        if len(pairs) < 10:
            raise ArmError('The camera saw the arm tip at only %d of %d poses (need 10). Is the '
                           'workspace clear and the yellow-orange tip piece in view?'
                           % (len(pairs), len(grid) * len(CAL_Z)))
        P = fit_projection(pairs)
        err = [math.hypot(project(P, *p)[0] - q[0], project(P, *p)[1] - q[1]) for p, q in pairs]
        rms = float(np.sqrt(np.mean(np.square(err))))
        self.say('camera model fitted from %d poses, RMS error %.1f px (worst %.1f)',
                 len(pairs), rms, max(err))
        if rms > 15.0:
            raise ArmError('Calibration error %.1f px is too large; check the tip detection.' % rms)
        with open(CAL_FILE, 'w') as stream:
            json.dump(dict(P=P.tolist(), points=len(pairs), rms_px=rms, made=time.ctime()), stream)
        self.say('saved %s', CAL_FILE)

    def show(self):
        """No motion: where each visible object is, and where the arm would aim."""
        mode = self.aim_mode()
        frame = self.fresh_frame(0.0)
        print('aim mode: %s' % mode)
        for color in COLORS:
            for area, u, v in self.blobs(frame, color)[:3]:
                try:
                    x, y, how = self.arm_xy(mode, color, u, v)
                    print('%-6s pixel (%3.0f, %3.0f) area %4.0f -> arm (%.0f, %.0f) mm [%s], '
                          'grasp z %.0f' % (color, u, v, area, x, y, how, self.z[color]))
                except ArmError as exc:
                    print('%-6s pixel (%3.0f, %3.0f): %s' % (color, u, v, exc))

    # ----------------------------------------------------------- operations
    def check_gone(self, color, original):
        """After the lift, with the arm tucked: 'gone', 'still' (on the floor) or ('moved', pos)."""
        u0, v0, area0 = original
        seen = []
        for _ in range(5):
            seen.extend(self.blobs(self.fresh_frame(0.2), color, min_area=0.5 * area0))
        if not seen:
            return 'gone', None
        near = [b for b in seen if math.hypot(b[1] - u0, b[2] - v0) <= self.args.gone_radius]
        if near:
            return 'still', None
        far = [b for b in seen if math.hypot(b[1] - u0, b[2] - v0) <= 3 * self.args.gone_radius]
        return ('moved', (far[0][1], far[0][2], far[0][0])) if far else ('gone', None)

    def grasp(self, color, anchor=None):
        """Reach down, grasp and lift; True only when the object has really left the floor
        (or, with no camera, after a single unverified attempt)."""
        if color not in COLORS:
            raise ArmError('Unknown colour %r (blue or yellow).' % color)
        if self.holding:
            raise ArmError('Already holding the %s object.' % self.holding)
        mode = self.aim_mode()
        self.wait_for_arm()
        self.say('grasp %s (aim: %s)', color, mode)
        self.goto(*CARRY, label='tuck: out of the way, camera sees the object')
        vision = mode != 'fixed' or self.args.verify
        original = None
        if vision and not self.dry:
            try:
                original = self.find_object(color, anchor)
            except ArmError:
                if mode != 'fixed':
                    raise
                self.say('no camera view: grasping the fixed spot unverified')
        elif self.dry:
            original = (320.0, 300.0, 400.0)
        for attempt in range(self.args.attempts):
            u = v = None
            if original is not None:
                u, v = original[0], original[1]
            x, y, how = self.arm_xy(mode, color, u, v)
            dx, dy = NUDGES[attempt % len(NUDGES)]
            z = max(self.z[color] - self.args.depth_step * attempt, FLOOR_Z)
            self.say('attempt %d/%d: %s at (%.0f, %.0f) mm [%s] nudge (%d, %d), down to z=%.0f',
                     attempt + 1, self.args.attempts, color, x + dx, y + dy, how, dx, dy, z)
            self.check_envelope(x + dx, y + dy, z)
            self.goto(x + dx, y + dy, self.args.hover_z, label='hover')
            self.pump(True)
            self.goto(x + dx, y + dy, z, label='lower all the way onto the %s object' % color,
                      wait=self.args.hold_time)
            self.sleep(self.args.hold_time)
            self.holding = color
            self.goto(x + dx, y + dy, self.args.hover_z, label='lift', wait=self.args.lift_wait)
            self.goto(*CARRY, label='tuck and look', wait=self.args.lift_wait)
            if original is None or self.dry:
                self.say('grasp done (not verified: no camera view of the object)')
                return True
            verdict, where = self.check_gone(color, original)
            self.say('check: object %s', verdict)
            if verdict == 'gone':
                return True
            self.pump(False)
            self.holding = None
            if verdict == 'moved':
                self.say('the object was pushed to pixel (%.0f, %.0f); following it', where[0], where[1])
                original = where
        raise ArmError('The %s object is still on the floor after %d attempts.'
                       % (color, self.args.attempts))

    def release(self):
        """Swing the arm out to the robot's left and let go. The object is not put down: it
        falls from the drop height, off to the left of the robot."""
        self.wait_for_arm()
        self.say('release (holding: %s)', self.holding or 'nothing known')
        self.goto(*self.args.drop, label='swing out to the left', wait=self.args.drop_wait)
        self.pump(False)
        self.sleep(0.0 if self.dry else self.args.drop_settle)
        self.holding = None
        self.goto(*CARRY, label='back to the tucked pose', wait=self.args.drop_wait)
        return True

    # ------------------------------------------------------------- protocol
    def publish_status(self, value):
        self.status_pub.publish(String(data=value))

    def run_job(self, fn, *a):
        if not self.busy.acquire(False):
            rospy.logwarn('Arm is busy; ignoring the request.')
            return None
        try:
            self.cancelled = False
            return fn(*a)
        finally:
            self.busy.release()

    def pick_color(self, text):
        if ':' in text:
            return text.split(':', 1)[1].strip().lower()
        return str(rospy.get_param('/grasp_color', self.args.color)).lower()

    def on_grasp_topic(self, msg):
        text = msg.data.strip()
        if text.startswith('1'):
            threading.Thread(target=self.do_grasp_job, args=(self.pick_color(text),),
                             daemon=True).start()
        elif text == '0':
            threading.Thread(target=self.do_release_job, daemon=True).start()
        elif text == 'cancel':
            self.cancelled = True

    def do_grasp_job(self, color, anchor=None):
        """Returns (ok, fatal)."""
        ok, fatal = False, False
        try:
            ok = bool(self.run_job(self.grasp, color, anchor))
            self.report('GRASPED' if ok else 'FAILED', color=color)
        except ArmError as exc:
            rospy.logerr('Grasp failed: %s', exc)
            fatal = exc.fatal
            self.report('FAILED', color=color, error=str(exc))
        self.publish_status('1' if ok else self.args.fail_code)
        return ok, fatal

    def do_release_job(self):
        fatal = False
        try:
            self.run_job(self.release)
            self.report('RELEASED')
        except ArmError as exc:
            rospy.logerr('Release failed: %s', exc)
            fatal = exc.fatal
            self.report('FAILED', error=str(exc))
        self.publish_status('0')
        return fatal

    def on_nav_status(self, msg):
        try:
            s = json.loads(msg.data)
            if not isinstance(s, dict):
                raise ValueError('not an object')
        except ValueError:
            return
        state = s.get('state')
        if state in ('TO_END', 'AT_START', 'TO_START'):
            self.last_in_reach = {}                 # a new leg: any earlier arrival is stale
        elif state in ('IN_REACH', 'NOT_IN_REACH'):
            self.last_in_reach = s
        elif state == 'FAILED':
            self.cancelled = True                   # navigation gave up: stop arm work too
        elif state == 'WAITING':
            threading.Thread(target=self.nav_job, args=(s.get('waiting_for'),), daemon=True).start()

    def carry_on(self, text):
        """Tell course_nav.py to continue (or abort); warn if it is not listening."""
        if self.cont_pub.get_num_connections() == 0:
            rospy.logwarn('Nothing listens on /course_nav/continue; is course_nav.py --handoff '
                          'running?')
        self.cont_pub.publish(String(data=text))

    def nav_job(self, waiting_for):
        if waiting_for == 'grasp':
            info = self.last_in_reach
            color = str(info.get('color') or rospy.get_param('/grasp_color', self.args.color))
            if info.get('state') != 'IN_REACH' or color not in COLORS:
                rospy.logwarn('Nothing in reach to grasp (%s); telling course_nav to carry on.',
                              info.get('reason', 'no IN_REACH'))
                self.report('FAILED', error='no object in reach: %s' % info.get('reason', 'none'))
                self.carry_on('done')
                return
            pixel = info.get('pixel')
            anchor = tuple(pixel) if isinstance(pixel, list) and len(pixel) == 2 else None
            ok, fatal = self.do_grasp_job(color, anchor)
            self.carry_on('abort' if fatal else 'done')     # a failed grasp: carry on empty-handed
        elif waiting_for == 'release':
            fatal = self.do_release_job() if self.holding else False
            self.carry_on('abort' if fatal else 'done')

    def serve(self):
        rospy.Subscriber('grasp', String, self.on_grasp_topic, queue_size=1)
        if self.args.follow_nav:
            rospy.Subscriber('/course_nav/status', String, self.on_nav_status, queue_size=10)
        self.wait_for_arm()
        self.goto(*CARRY, label='tucked')
        rospy.loginfo('Ready. /grasp "1"/"1:yellow" grasps, "0" releases%s.',
                      '; also following /course_nav/status' if self.args.follow_nav else '')
        rospy.spin()


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('command', nargs='?', default='serve',
                   choices=['serve', 'grasp', 'release', 'cycle', 'calibrate', 'show', 'set-spot',
                            'move'])
    p.add_argument('--color', default='blue', choices=sorted(COLORS))
    p.add_argument('--aim', default='auto', choices=['auto', 'camera', 'calibration', 'fixed'],
                   help='camera = model from `calibrate`; calibration = ~/thefile.txt; fixed = '
                        'taught spot. auto picks the best one available')
    p.add_argument('--follow-nav', action='store_true',
                   help="also follow course_nav.py's /course_nav/status handoff")
    p.add_argument('--fail-code', default='-1',
                   help='/grasp_status value for a failed grasp: -1 (what grasp_pro_new.py '
                        'sends); the PDF says 0')
    p.add_argument('--x', type=float)
    p.add_argument('--y', type=float)
    p.add_argument('--z', type=float)
    p.add_argument('--blue-z', type=float, default=GRASP_Z['blue'],
                   help='cup height on top of the blue object, mm')
    p.add_argument('--yellow-z', type=float, default=GRASP_Z['yellow'],
                   help='cup height on top of the yellow cube, mm (floor is about -110)')
    p.add_argument('--attempts', type=int, default=4, help='tries before giving up')
    p.add_argument('--depth-step', type=float, default=5.0,
                   help='go this much deeper on each retry, mm')
    p.add_argument('--hover-z', type=float, default=20.0)
    p.add_argument('--calibration-file', default='~/thefile.txt')
    p.add_argument('--image-topic', default='/camera/rgb/image_raw')
    p.add_argument('--min-area', type=float, default=150.0)
    p.add_argument('--stable-frames', type=int, default=8)
    p.add_argument('--find-timeout', type=float, default=6.0)
    p.add_argument('--anchor-radius', type=float, default=80.0,
                   help='with --follow-nav: only blobs this close (px) to where course_nav '
                        'stopped count as the object')
    p.add_argument('--gone-radius', type=float, default=60.0,
                   help='the object counts as still on the floor if seen within this many px')
    p.add_argument('--arm-speed', type=float, default=80.0,
                   help='measured SwiftPro travel speed, mm/s (waits scale with distance)')
    p.add_argument('--settle', type=float, default=0.8, help='extra seconds after each move')
    p.add_argument('--hold-time', type=float, default=1.2, help='suction time before lifting')
    p.add_argument('--lift-wait', type=float, default=2.5)
    p.add_argument('--drop-wait', type=float, default=2.0)
    p.add_argument('--drop', type=lambda t: tuple(float(v) for v in t.split(',')), default=DROP,
                   help='release pose x,y,z in mm (default: out to the left, 60 mm up)')
    p.add_argument('--drop-settle', type=float, default=1.0,
                   help='seconds to let the object fall before the arm moves back')
    p.add_argument('--pump-wait', type=float, default=0.6)
    p.add_argument('--no-verify', dest='verify', action='store_false',
                   help='fixed aim only: skip the camera check that the object left the floor')
    p.add_argument('--dry-run', action='store_true', help='print the arm sequence; publish nothing')
    return p.parse_args(rospy.myargv()[1:])


def main():
    args = parse_args()
    if not args.dry_run:
        rospy.init_node('course_grasp', anonymous=False)
    g = Grasper(args)
    try:
        g.check_envelope(*args.drop)
        if args.command == 'serve':
            g.serve()
        elif args.command == 'grasp':
            g.grasp(args.color)
        elif args.command == 'release':
            g.release()
        elif args.command == 'cycle':
            g.grasp(args.color)
            if not args.dry_run:
                g.sleep(2.0)
            g.release()
        elif args.command == 'calibrate':
            if args.dry_run:
                print('[dry-run] calibrate: tuck, then 12 tip poses at each of z=%s, fit a camera '
                      'model' % (CAL_Z,))
            else:
                g.calibrate()
        elif args.command == 'show':
            g.show()
        elif args.command == 'move':
            if None in (args.x, args.y, args.z):
                raise ArmError('move needs --x --y --z (mm).')
            g.wait_for_arm()
            g.goto(args.x, args.y, args.z)
        elif args.command == 'set-spot':
            if args.x is None or args.y is None:
                raise ArmError('set-spot needs --x and --y (mm).')
            g.check_envelope(args.x, args.y, args.hover_z)
            spots = json.load(open(SPOT_FILE)) if os.path.exists(SPOT_FILE) else {}
            spots[args.color] = dict(x=args.x, y=args.y)
            with open(SPOT_FILE, 'w') as stream:
                json.dump(spots, stream, indent=2)
            print('Saved %s spot (%.0f, %.0f) mm to %s' % (args.color, args.x, args.y, SPOT_FILE))
    except ArmError as exc:
        if args.dry_run:
            print('ERROR: %s' % exc)
        else:
            rospy.logerr('%s', exc)
        raise SystemExit(1)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
