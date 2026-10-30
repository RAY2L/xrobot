#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Grasp and release the blue or yellow object with the SwiftPro arm.

The grasp follows Spark OneKey 9 -> 2 -> 1 (spark_carry/nodes/s_carry_object_cv3.py): same
detection, same ~/thefile.txt aim from OneKey 8 (arm_x = a*v + b, arm_y = c*u + d), hover
z=20, down, pump on, sent 0.1 s apart as OneKey does. The arm only moves between the centred
stow pose (120, 0, 35) and the object: it starts at stow, reaches down, comes straight up and
back to stow, and carries the object there; `drop` lets go at stow. Grasp heights and aim
corrections are per colour (--blue-*/--yellow-*). The base never moves.

Live view while it runs: open http://<robot-ip>:8081/ in a browser. It shows the camera with
the detected object (green box), where the arm will land (red cross), the arm's reach (grey
ellipse), and the current step, pump state and log.

On Spark, source ~/spark_noetic/devel/setup.bash with the camera and SwiftPro drivers running:
  python3 course_grasp.py view --color blue      # live view only, never moves the arm
  python3 course_grasp.py test --color blue      # grasp, hold, release on the same spot
  python3 course_grasp.py grasp --color blue     # grasp and keep holding
  python3 course_grasp.py release                # pump off where the arm is
  python3 course_grasp.py drop                   # same: let go in the air, no arm motion
  python3 course_grasp.py test --drop            # grasp, hold, then drop instead of put down
  python3 course_grasp.py calibrate              # OneKey 8 redone on a grid; writes ~/thefile.txt
  python3 course_grasp.py serve                  # wait for a coordinator's commands:
      rostopic pub -1 /course_grasp/command std_msgs/String "data: 'grasp yellow'"
      ... "data: 'drop'"  or  "data: 'release'"; results on /course_grasp/status (JSON)
  ... --dry-run                                  # view + plan, no arm or pump commands
--dx/--dy (mm, +y is LEFT) nudge the aim. A calibration with tiny slopes is refused.
There is no suction sensor: watch the live view, where the held object should hang under the
cup at the stow pose.
No pump-off on exit, so a held object is never dropped by Ctrl+C.
"""
import argparse
import collections
import json
import math
import os
import shutil
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from swiftpro.msg import position, status

# OpenCV HSV ranges (H 0-179), identical to course_nav.py COLORS so the two scripts agree on
# what blue and yellow are; keep them in sync. Blue from move2grasp/scripts/grasp_pro_new.py,
# yellow from spark_app/spark_sagittarius_carry/config/hsv.yaml.
COLORS = {
    'blue': ([95, 70, 60], [140, 255, 255]),
    'yellow': ([20, 65, 63], [35, 255, 255]),
}
# Explicit colour check on top of the range: a blob is that colour only if its median hue lies
# in this core band. The arm's own orange marker reads median H 17-20, inside the bottom of the
# yellow range, and was grasped as "yellow"; the yellow cubes read H 28-30.
CORE_HUE = {'blue': (100, 130), 'yellow': (24, 35)}
# Colours whose side faces fall inside the colour mask: aim at the top face only. (Blue's
# sides are white, so its mask is already just the top face.)
TOP_FACE = {'yellow'}
# The orange calibration marker above the suction cup (OneKey 8's "red" marker). Measured on
# this robot at H 16-19, S ~200; the yellow cubes are H 28-30.
MARKER_HSV = ([8, 90, 120], [26, 255, 255])
GREEN, RED, GREY, WHITE, ORANGE = ((0, 220, 0), (0, 0, 255), (170, 170, 170), (255, 255, 255),
                                   (0, 140, 255))


def find_blob(frame, color, args, near=None):
    """Largest rotated rectangle of the colour, as OneKey 9 picks it, among blobs that pass
    the CORE_HUE check. With `near` (u, v), e.g. where course_nav saw the object, the blob
    closest to it within --near-radius px instead."""
    lo, hi = COLORS[color]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    raw = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    if args.self_mask:
        # The arm's own orange marker at the stow pose (where every detection happens): it is
        # overexposed there and reads as yellow (median H 26), so blank it out before blobs
        # are merged, or it gets grasped (or fused with a cube next to it).
        u0, v0, u1, v1 = args.self_mask
        raw[v0:v1, u0:u1] = 0
    # OneKey 9 thresholds the masked H channel at 90; that would remove yellow (H 20-35).
    _, mask = cv2.threshold(cv2.blur(raw, (9, 9)), 127, 255, cv2.THRESH_BINARY)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                            cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25)))
    mask = cv2.dilate(cv2.erode(mask, None, iterations=4), None, iterations=4)
    best, best_contour, best_score = None, None, None
    for contour in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]:
        rect = cv2.minAreaRect(contour)
        (u, v), (w, h), _ = rect
        if cv2.contourArea(contour) < args.min_area or v < args.min_row:
            continue
        m = cv2.moments(contour)                # centroid of the whole colour area
        cu, cv_ = m['m10'] / m['m00'], m['m01'] / m['m00']
        if near is not None:
            gap = math.hypot(cu - near[0], cv_ - near[1])
            if gap > args.near_radius:
                continue                        # not the object course_nav stopped at
            score = -gap                        # nearest wins
        else:
            score = w * h                       # biggest wins (OneKey 9)
        if (best_score is None or score > best_score) and core_hue_ok(hsv, raw, contour, color):
            best, best_contour, best_score = dict(
                u=float(u), v=float(v), area=float(w * h), cu=cu, cv=cv_,
                box=cv2.boxPoints(rect).astype(int).tolist()), contour, score
    if best is not None and color in TOP_FACE:
        top = top_face(frame, best_contour)
        if top is not None:
            best.update(top)
    return best


def core_hue_ok(hsv, raw, contour, color):
    """Median hue of the blob's in-range pixels lies in CORE_HUE[color]."""
    region = np.zeros(raw.shape, np.uint8)
    cv2.drawContours(region, [contour], -1, 255, -1)
    hues = hsv[:, :, 0][(region > 0) & (raw > 0)]
    lo, hi = CORE_HUE[color]
    return len(hues) > 0 and lo <= float(np.median(hues)) <= hi


def top_face(frame, contour):
    """The camera looks down at an angle, so a yellow cube shows its top face plus the darker
    side facing the camera, which pulls the centre toward the robot. Keep the brighter part
    (Otsu on V inside the blob) when the split is clear; return its centre and outline."""
    region = np.zeros(frame.shape[:2], np.uint8)
    cv2.drawContours(region, [contour], -1, 255, -1)
    value = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[:, :, 2]
    inside = value[region > 0]
    level, _ = cv2.threshold(inside.reshape(-1, 1), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    bright, dark = inside[inside > level], inside[inside <= level]
    if len(dark) == 0 or bright.mean() - dark.mean() < 25 or not 0.3 <= len(bright) / len(inside) <= 0.9:
        return None                             # no clear top/side split: keep the whole blob
    face = np.where((value > level) & (region > 0), 255, 0).astype(np.uint8)
    face = cv2.morphologyEx(face, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours = cv2.findContours(face, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    biggest = max(contours, key=cv2.contourArea, default=None)
    if biggest is None or cv2.contourArea(biggest) < 0.25 * len(inside):
        return None
    m = cv2.moments(biggest)
    return dict(u=m['m10'] / m['m00'], v=m['m01'] / m['m00'],
                top=cv2.boxPoints(cv2.minAreaRect(biggest)).astype(int).tolist())


def find_marker(frame, reference):
    """Pixel of the arm's orange marker. Only marker-coloured pixels that changed since the
    reference frame count, so orange/yellow things that do not move with the arm are ignored."""
    mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV),
                       np.array(MARKER_HSV[0], np.uint8), np.array(MARKER_HSV[1], np.uint8))
    changed = (cv2.absdiff(frame, reference).max(axis=2) > 40).astype(np.uint8) * 255
    mask = cv2.morphologyEx(cv2.bitwise_and(mask, changed), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    best = max(contours, key=cv2.contourArea, default=None)
    if best is None or cv2.contourArea(best) < 80:
        return None
    m = cv2.moments(best)
    return m['m10'] / m['m00'], m['m01'] / m['m00']


def calibration_grid(args):
    """Arm (x, y) points for `calibrate`: a grid, unlike OneKey 8's single diagonal line, so
    both x and y vary independently. Snake order keeps the moves short."""
    points = []
    for i, x in enumerate((170.0, 210.0, 250.0)):
        ys = (-120.0, -60.0, 0.0, 60.0, 120.0, 180.0)
        for y in (ys if i % 2 == 0 else ys[::-1]):
            if 150 <= math.hypot(x, y) <= args.cal_max_r:
                points.append((x, y))
    return points


def read_calibration(args):
    """(a, b, c, d) from ~/thefile.txt, refusing a degenerate fit."""
    with open(os.path.expanduser(args.calibration_file)) as stream:
        values = [float(v) for v in stream.read().split()]
    if len(values) != 4 or not all(math.isfinite(v) for v in values):
        raise ValueError('%s must contain four finite numbers: a b c d' % args.calibration_file)
    a, b, c, d = values
    # Slopes are mm per pixel; working Spark calibrations are about 0.8-1.0. A tiny slope
    # sends the arm to roughly the same spot wherever the object is.
    if min(abs(a), abs(c)) < args.min_slope and not args.force_calibration:
        raise ValueError('%s looks degenerate: slopes %.3f and %.3f mm/px. Recalibrate '
                         '(OneKey 8) or pass --force-calibration.' % (args.calibration_file, a, c))
    return a, b, c, d


def arm_to_pixel(cal, x, y):
    a, b, c, d = cal
    return (y - d) / c, (x - b) / a


class Grasper:
    """grasp(color) and release() for a later course_nav adapter; serves the live view."""

    def __init__(self, args):
        self.args = args
        self.bridge = CvBridge()
        self.cv = threading.Condition()
        self.frame, self.frame_id, self.frame_time = None, 0, 0.0
        self.jpeg = None
        self.closed = threading.Event()
        self.lock = threading.Lock()
        self.cal = None
        # What the live view shows; written by the arm thread, read by the view thread.
        self.state = dict(step='idle', color=args.color, pump=False, holding=None,
                          detected=None, aim=None, dry_run=args.dry_run,
                          cal_plan=None, cal_seen=[])    # calibrate: planned and measured pixels
        self.log = collections.deque(maxlen=14)
        self.spot = None                      # (x, y, z) where the held object was picked up
        self.stowed = False                   # arm in the stow (carry) pose
        self.arm_pub = self.pump_pub = self.result_pub = None
        if not args.dry_run:
            self.arm_pub = rospy.Publisher('/position_write_topic', position, queue_size=10)
            self.pump_pub = rospy.Publisher('/pump_topic', status, queue_size=1)
            self.result_pub = rospy.Publisher('/course_grasp/status', String, queue_size=10,
                                              latch=True)
        rospy.Subscriber(args.image_topic, Image, self.image_cb, queue_size=1, buff_size=2 ** 24)
        rospy.on_shutdown(self.close)

    # ------------------------------------------------------------------ helpers
    def say(self, text, *a):
        text = text % a if a else text
        rospy.loginfo(text)
        self.log.append(time.strftime('%H:%M:%S ') + text)

    def step(self, name):
        self.state['step'] = name
        self.say('STEP: %s', name)

    def close(self):
        self.closed.set()
        with self.cv:
            self.cv.notify_all()

    def pause(self, seconds):
        if self.closed.wait(seconds) or rospy.is_shutdown():
            raise RuntimeError('Interrupted; no further arm or pump commands sent')

    def calibration(self):
        if self.cal is None:
            self.cal = read_calibration(self.args)
        return self.cal

    # ------------------------------------------------------------------- camera
    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5, str(exc))
            return
        with self.cv:
            self.frame, self.frame_time = frame, time.monotonic()
            self.frame_id += 1
            self.cv.notify_all()

    def next_frame(self, after_id, deadline):
        with self.cv:
            while not self.closed.is_set():
                if self.frame_id > after_id and time.monotonic() - self.frame_time < 1.0:
                    return self.frame_id, self.frame.copy()
                if time.monotonic() >= deadline:
                    raise RuntimeError('No camera images on %s' % self.args.image_topic)
                self.cv.wait(0.1)
        raise RuntimeError('Interrupted')

    def detect(self, color, near=None):
        """Pixel of the object once it has stayed within 2 px for 20 frames (OneKey 9)."""
        self.step('detecting %s' % color)
        deadline = time.monotonic() + self.args.detect_timeout
        with self.cv:
            seq = self.frame_id
        previous, count = None, 0
        while time.monotonic() < deadline:
            seq, frame = self.next_frame(seq, deadline)
            blob = find_blob(frame, color, self.args, near)
            still = (blob and previous
                     and abs(blob['u'] - previous['u']) <= self.args.pixel_tolerance
                     and abs(blob['v'] - previous['v']) <= self.args.pixel_tolerance)
            count = count + 1 if still else (1 if blob else 0)
            previous = blob
            if count >= self.args.stable_frames:
                self.say('%s stable at pixel (%.0f, %.0f)', color, blob['u'], blob['v'])
                return blob
        raise RuntimeError('No stable %s object within %.0f s' % (color, self.args.detect_timeout))

    # ---------------------------------------------------------------------- arm
    def check_pose(self, x, y, z, z_min=None):
        r = math.hypot(x, y)
        z_min = self.args.z_min if z_min is None else z_min
        if not (self.args.min_xy <= r <= self.args.max_xy and z_min <= z <= 150):
            raise ValueError('Arm target (%.0f, %.0f, %.0f) mm is out of reach (radius %.0f, '
                             'allowed %.0f-%.0f); move the object closer to the arm'
                             % (x, y, z, r, self.args.min_xy, self.args.max_xy))

    def connect(self):
        if self.args.dry_run:
            return
        deadline = time.monotonic() + 10
        while not (self.arm_pub.get_num_connections() and self.pump_pub.get_num_connections()):
            if time.monotonic() > deadline:
                raise RuntimeError('SwiftPro driver not running (roslaunch swiftpro '
                                   'pro_control_nomoveit.launch)')
            self.pause(0.05)

    def move(self, x, y, z, label, wait=None, z_min=None):
        self.check_pose(x, y, z, z_min)
        self.step('%s -> arm (%.0f, %.0f, %.0f) mm' % (label, x, y, z))
        if not self.args.dry_run:
            self.arm_pub.publish(position(x=x, y=y, z=z))
        self.pause(0 if self.args.dry_run else self.args.move_wait if wait is None else wait)

    def pump(self, on, wait=None):
        self.step('suction %s' % ('ON' if on else 'OFF'))
        self.state['pump'] = on
        if not self.args.dry_run:
            self.pump_pub.publish(status(status=int(on)))
        self.pause(0 if self.args.dry_run else self.args.pump_wait if wait is None else wait)

    def report(self, action, ok=True, **extra):
        """Latched JSON on /course_grasp/status, e.g. {"action": "grasp", "color": "yellow",
        "ok": true, ...}. ok means the sequence ran, not that a sensor confirmed the object."""
        record = dict(action=action, ok=ok, dry_run=self.args.dry_run, object_confirmed=False,
                      **extra)
        if self.result_pub is not None:
            self.result_pub.publish(String(data=json.dumps(record)))

    # ------------------------------------------------------- coordinator interface
    def serve(self):
        """Wait for a coordinator (later: the node combining course_nav and course_grasp) to say
        what to do on /course_grasp/command (std_msgs/String): "grasp blue", "grasp yellow",
        optionally with the pixel where the object was seen ("grasp yellow 323 301": take the
        blob nearest it), "stow", "drop" (pump off) or "release" (put down where picked up).
        Each command answers on /course_grasp/status. One at a time; others are rejected."""
        rospy.Subscriber('/course_grasp/command', String, self.on_command, queue_size=5)
        self.step('ready: waiting for /course_grasp/command')
        rospy.spin()

    def on_command(self, msg):
        words = msg.data.strip().lower().split()
        action, color = (words + [None, None])[:2]
        near = None
        if action == 'grasp' and len(words) == 4:
            try:
                near = (float(words[2]), float(words[3]))
            except ValueError:
                action = None
        if action not in ('grasp', 'release', 'drop', 'stow') or \
                (action == 'grasp' and color not in COLORS) or len(words) not in (1, 2, 4):
            self.say('rejected command %r (use "grasp blue|yellow", "stow", "drop" or "release")',
                     msg.data)
            self.report('rejected', ok=False, command=msg.data, error='unknown command')
            return
        if self.lock.locked():
            self.say('busy: rejected %r', msg.data)
            self.report(action, ok=False, color=color, error='busy')
            return
        self.say('command: %s', msg.data.strip())
        threading.Thread(target=self.run_command, args=(action, color, near), daemon=True).start()

    def run_command(self, action, color, near=None):
        try:
            if action == 'grasp':
                self.grasp(color, near)
            elif action == 'drop':
                self.drop()
            elif action == 'stow':
                self.stow()
                self.report('stow')
            else:
                self.release()
        except (RuntimeError, ValueError, OSError, cv2.error) as exc:
            self.step('FAILED: %s' % exc)
            self.report(action, ok=False, color=color, error=str(exc))

    # --------------------------------------------------------------- operations
    def grasp(self, color, near=None):
        """From the stow pose: hover, down onto the object, pump on, straight back up, back to
        stow, each sent 0.1 s apart as OneKey 9 does (the arm firmware queues G-code and runs it
        in order), then wait --lift-wait. The arm only ever reaches out to grasp and comes back
        to stow; it carries the object there. Keeps holding."""
        with self.lock:
            if self.state['holding']:
                raise RuntimeError('Already holding %s; release first' % self.state['holding'])
            self.state.update(color=color, aim=None)
            cal = self.calibration()
            # Always start from the same, centred stow pose, so the arm is never somewhere odd
            # (or in the camera's way) when the object is detected and the moves begin.
            self.check_pose(*self.args.stow_pose)
            self.connect()
            if not self.stowed:
                self.move(*self.args.stow_pose, label='start from stow')
                self.stowed = True
            blob = self.detect(color, near)
            x, y = self.aim_xy(cal, blob, color)
            z = getattr(self.args, color + '_grasp_z')
            u, v = arm_to_pixel(cal, x, y)
            self.state['aim'] = dict(x=x, y=y, z=z, u=u, v=v)
            self.say('aim: arm x=%.0f y=%.0f mm (y + is LEFT), down to z=%.0f', x, y, z)
            for pose in ((x, y, self.args.hover_z), (x, y, z)):
                self.check_pose(*pose)          # refuse before the first motion
            self.connect()
            self.stowed = False
            gap = self.args.onekey_gap
            self.move(x, y, self.args.hover_z, 'hover above (queued)', gap)
            self.move(x, y, z, 'lower onto object (queued)', gap)
            self.pump(True, gap)
            self.state['holding'] = color       # unverified: there is no suction sensor
            # Straight up first so the object does not drag along the floor, then home to stow.
            self.move(x, y, self.args.hover_z, 'lift straight up (queued)', gap)
            self.move(*self.args.stow_pose, label='back to stow, holding', wait=self.args.lift_wait)
            self.stowed = True
            self.spot = (x, y, z)
            self.step('holding %s at stow (check the view: is it under the cup?)' % color)
            self.report('grasp', color=color, pixel=[blob['u'], blob['v']], arm=[x, y, z])

    def release(self):
        """Put the object back down where it was picked up, suction off, then OneKey's home
        pose (120, 0, 35), which keeps the arm out of the way of the camera."""
        with self.lock:
            self.connect()
            if self.spot:
                x, y, z = self.spot
                self.move(x, y, self.args.hover_z, 'back over the pick-up spot', self.args.lift_wait / 2)
                self.move(x, y, z + self.args.release_clearance, 'lower to put down')
                self.pump(False)
                self.move(x, y, self.args.hover_z, 'lift away')
                self.move(*self.args.home_pose, label='home')
            else:
                self.say('no grasp spot known in this run: suction off in place')
                self.pump(False)
            self.state['holding'], self.spot = None, None
            self.step('released')
            self.report('release')

    def aim_xy(self, cal, blob, color):
        """Arm (x, y) for a detected object. ~/thefile.txt is fitted on the marker above the
        cup, so it is right for things about as high as the marker (blue's top). A shorter
        object's top is lower and, seen from the camera's angle, looks nearer the robot and the
        middle than it is; scaling the aim outward from the arm base by --<colour>-radial undoes
        that. Then --dx/--dy."""
        a, b, c, d = cal
        u, v = (blob['cu'], blob['cv']) if self.args.aim_point == 'centroid' else (blob['u'], blob['v'])
        s = getattr(self.args, color + '_radial')
        return (a * v + b) * s + self.args.dx, (c * u + d) * s + self.args.dy

    def stow(self):
        """Tuck the arm (holding or not) into the centred stow pose (--stow-pose) for driving."""
        with self.lock:
            self.connect()
            self.move(*self.args.stow_pose, label='stow for driving')
            self.stowed = True

    def drop(self):
        """Let go where the arm is (normally stow): pump off, the object falls; no arm motion."""
        with self.lock:
            self.connect()
            self.pump(False, self.args.drop_wait)
            self.state['holding'], self.spot = None, None
            self.step('dropped')
            self.report('release', mode='drop')

    def grab(self, settle=0.0):
        self.pause(settle)
        with self.cv:
            seq = self.frame_id
        return self.next_frame(seq, time.monotonic() + 3)[1]

    def calibrate(self):
        """OneKey 8 redone: put the cup down at z=-110 (as OneKey 8 does) on a grid of arm
        points, find the orange marker in the image, fit arm_x = a*v + b, arm_y = c*u + d and
        write ~/thefile.txt in OneKey's format, so OneKey 9 uses it too. Writes only a sane fit
        and keeps a backup; the points, fit and images are saved under --output-dir."""
        with self.lock:
            args = self.args
            grid = calibration_grid(args)
            for x, y in grid:
                self.check_pose(x, y, args.cal_z, z_min=args.cal_z)
            try:                                # show where the cup will touch down
                cal = self.calibration()
                self.state['cal_plan'] = [list(arm_to_pixel(cal, x, y)) for x, y in grid]
            except (ValueError, OSError):
                self.state['cal_plan'] = None
            self.state['cal_seen'] = []
            stamp = time.strftime('%Y%m%d-%H%M%S')
            out = os.path.join(os.path.expanduser(args.output_dir), 'cal-' + stamp)
            self.say('calibration: %d points, cup down to z=%.0f; keep the floor under them clear',
                     len(grid), args.cal_z)
            self.connect()
            self.move(*args.home_pose, label='home: reference view', wait=4)
            reference = None if args.dry_run else self.grab(1.0)
            points = []
            for i, (x, y) in enumerate(grid):
                n = 'point %d/%d' % (i + 1, len(grid))
                self.move(x, y, args.cal_hover_z, n + ': above', wait=args.cal_wait)
                self.move(x, y, args.cal_z, n + ': cup down', wait=2.0, z_min=args.cal_z)
                if not args.dry_run:
                    seen = [p for p in (find_marker(self.grab(0.1), reference) for _ in range(6)) if p]
                    u = float(np.median([p[0] for p in seen])) if len(seen) >= 4 else None
                    v = float(np.median([p[1] for p in seen])) if len(seen) >= 4 else None
                    if u is None or max(math.hypot(p[0] - u, p[1] - v) for p in seen) > 4:
                        self.say('%s: marker not found steadily (%d/6 frames), skipped', n, len(seen))
                    else:
                        points.append(dict(x=x, y=y, u=u, v=v))
                        self.state['cal_seen'].append([u, v])
                        self.say('%s: arm (%.0f, %.0f) -> pixel (%.0f, %.0f)', n, x, y, u, v)
                        frame = self.grab()
                        cv2.circle(frame, (int(u), int(v)), 8, ORANGE, 2)
                        os.makedirs(out, exist_ok=True)
                        cv2.imwrite(os.path.join(out, 'point%02d.jpg' % (i + 1)), frame)
                self.move(x, y, args.cal_hover_z, n + ': up', wait=1.5)
            self.move(*args.home_pose, label='home')
            if not args.dry_run:
                self.save_fit(points, out, stamp)

    def save_fit(self, points, out, stamp):
        args = self.args
        if len(points) < 10:
            raise RuntimeError('Marker seen at only %d points (need 10); %s unchanged'
                               % (len(points), args.calibration_file))
        x, y, u, v = (np.array([p[k] for p in points]) for k in 'xyuv')
        a, b = np.polyfit(v, x, 1)
        c, d = np.polyfit(u, y, 1)
        err = np.hypot(a * v + b - x, c * u + d - y)
        # A full affine fit (x and y each from u and v) shows whether the camera is rotated
        # against the arm, which OneKey's separate fits cannot represent.
        A = np.column_stack([u, v, np.ones_like(u)])
        ax, ay = (np.linalg.lstsq(A, t, rcond=None)[0] for t in (x, y))
        rms = float(np.sqrt(np.mean(err ** 2)))
        rms_affine = float(np.sqrt(np.mean(np.hypot(A @ ax - x, A @ ay - y) ** 2)))
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, 'calibration.json'), 'w') as stream:
            json.dump(dict(made=stamp, points=points, onekey=[a, b, c, d], rms_mm=rms,
                           worst_mm=float(err.max()), affine_x=ax.tolist(), affine_y=ay.tolist(),
                           affine_rms_mm=rms_affine), stream, indent=2)
        self.say('fit: x = %.4f*v + %.1f, y = %.4f*u + %.1f | error RMS %.1f mm, worst %.1f mm '
                 '(affine fit RMS %.1f mm)', a, b, c, d, rms, err.max(), rms_affine)
        if not (0.3 <= abs(a) <= 1.5 and 0.3 <= abs(c) <= 1.5 and rms <= args.cal_max_rms):
            raise RuntimeError('Fit rejected (slopes %.3f, %.3f mm/px, RMS %.1f mm); %s unchanged. '
                               'Details in %s' % (a, c, rms, args.calibration_file, out))
        path = os.path.expanduser(args.calibration_file)
        if os.path.exists(path):
            shutil.copy2(path, path + '.bak-' + stamp)
        with open(path, 'w') as stream:
            stream.write('%s %s %s %s\n' % (float(a), float(b), float(c), float(d)))
        self.cal = None
        self.say('wrote %s (previous kept as %s.bak-%s); details in %s',
                 args.calibration_file, args.calibration_file, stamp, out)

    # --------------------------------------------------------------- live view
    def render_loop(self):
        seq = 0
        while not self.closed.is_set():
            try:
                seq, frame = self.next_frame(seq, time.monotonic() + 2.0)
            except RuntimeError:
                continue
            self.annotate(frame)
            ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                with self.cv:
                    self.jpeg = buf.tobytes()
            self.closed.wait(1.0 / self.args.fps)

    def annotate(self, img):
        s = self.state
        # on the clean frame, before drawing; release/drop/calibrate have no colour
        blob = find_blob(img, s['color'], self.args) if s['color'] else None
        try:
            cal = self.calibration()
        except (ValueError, OSError):
            cal = None
        if cal:                                 # where the arm can reach, on the floor
            bu, bv = arm_to_pixel(cal, 0, 0)
            axes = (int(self.args.max_xy / abs(cal[2])), int(self.args.max_xy / abs(cal[0])))
            cv2.ellipse(img, (int(bu), int(bv)), axes, 0, 0, 360, GREY, 1, cv2.LINE_AA)
            cv2.putText(img, 'reach %.0f mm' % self.args.max_xy, (6, img.shape[0] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, GREY, 1)
        s['detected'] = None
        if blob:
            pts = np.array(blob['box'], np.int32)
            cv2.polylines(img, [pts], True, GREEN, 2)
            if 'top' in blob:                   # the top face the aim is taken from
                cv2.polylines(img, [np.array(blob['top'], np.int32)], True, (255, 255, 0), 1)
            cv2.circle(img, (int(blob['u']), int(blob['v'])), 4, GREEN, -1)
            label = '%s (%.0f, %.0f)' % (s['color'], blob['u'], blob['v'])
            if cal:
                label += ' -> arm %.0f, %.0f' % self.aim_xy(cal, blob, s['color'])
            cv2.putText(img, label, (int(blob['u']) - 60, int(pts[:, 1].min()) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, GREEN, 1)
            s['detected'] = dict(u=blob['u'], v=blob['v'])
        aim = s['aim']
        if aim:                                 # locked target of the current grasp
            p = (int(aim['u']), int(aim['v']))
            cv2.drawMarker(img, p, RED, cv2.MARKER_CROSS, 26, 2)
            cv2.putText(img, 'aim %.0f, %.0f, z%.0f' % (aim['x'], aim['y'], aim['z']),
                        (p[0] + 14, p[1] + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, RED, 1)
        if self.args.self_mask and self.args.self_mask[2] > self.args.self_mask[0]:
            u0, v0, u1, v1 = self.args.self_mask           # ignored: the arm marker at stow
            cv2.rectangle(img, (u0, v0), (u1, v1), GREY, 1)
        for i, (pu, pv) in enumerate(s['cal_plan'] or []):     # calibrate: where the cup goes
            cv2.circle(img, (int(pu), int(pv)), 6, GREY, 1)
            cv2.putText(img, str(i + 1), (int(pu) + 7, int(pv) - 4), cv2.FONT_HERSHEY_SIMPLEX,
                        0.35, GREY, 1)
        for pu, pv in s['cal_seen']:                            # calibrate: marker found here
            cv2.circle(img, (int(pu), int(pv)), 4, ORANGE, -1)
        banner = '%s%s | pump %s' % ('[DRY RUN] ' if s['dry_run'] else '', s['step'],
                                     'ON' if s['pump'] else 'off')
        cv2.rectangle(img, (0, 0), (img.shape[1], 24), (0, 0, 0), -1)
        cv2.putText(img, banner[:80], (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    RED if s['pump'] else WHITE, 1)

    def serve_view(self):
        server = ThreadingHTTPServer(('0.0.0.0', self.args.port), make_handler(self))
        server.daemon_threads = True
        threading.Thread(target=server.serve_forever, daemon=True).start()
        threading.Thread(target=self.render_loop, daemon=True).start()
        self.say('live view: http://<robot-ip>:%d/', self.args.port)


PAGE = b"""<!doctype html><html><head><meta charset="utf-8"><title>Spark grasp</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>body{margin:0;background:#111;color:#ddd;font:14px sans-serif;display:flex;flex-wrap:wrap;
gap:12px;padding:12px}img{max-width:100%;width:640px;height:auto;background:#000}
#s{flex:1;min-width:260px;font-family:monospace;white-space:pre-wrap}b{color:#fff}
.on{color:#f55}.off{color:#8c8}</style></head><body>
<img id="v" src="/stream"><div id="s">connecting...</div>
<script>
const v=document.getElementById('v'),s=document.getElementById('s');let down=false;
async function poll(){try{const r=await fetch('/status',{cache:'no-store'});const t=await r.json();
 if(down){down=false;v.src='/stream?'+Date.now();}
 const d=t.detected,a=t.aim;
 s.innerHTML='<b>step</b>     '+t.step+(t.dry_run?'  [DRY RUN]':'')+
 '\\n<b>pump</b>     <span class="'+(t.pump?'on':'off')+'">'+(t.pump?'ON':'off')+'</span>'+
 '\\n<b>holding</b>  '+(t.holding||'-')+
 '\\n<b>detected</b> '+(d?t.color+' at pixel ('+d.u.toFixed(0)+', '+d.v.toFixed(0)+')':t.color+': not seen')+
 '\\n<b>aim</b>      '+(a?'arm x='+a.x.toFixed(0)+' y='+a.y.toFixed(0)+' z='+a.z.toFixed(0)+' mm':'-')+
 '\\n\\n'+t.log.join('\\n');
}catch(e){down=true;s.textContent='disconnected, retrying...';}setTimeout(poll,500);}
poll();
</script></body></html>"""


def make_handler(g):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def send(self, kind, body):
            self.send_response(200)
            self.send_header('Content-Type', kind)
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            try:
                if self.path in ('/', '/index.html'):
                    self.send('text/html; charset=utf-8', PAGE)
                elif self.path == '/status':
                    self.send('application/json',
                              json.dumps(dict(g.state, log=list(g.log))).encode())
                elif self.path.startswith('/stream'):
                    self.send_response(200)
                    self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=f')
                    self.end_headers()
                    last = None
                    while not g.closed.is_set():
                        jpeg = g.jpeg
                        if jpeg is not None and jpeg is not last:
                            last = jpeg
                            self.wfile.write(b'--f\r\nContent-Type: image/jpeg\r\n'
                                             b'Content-Length: %d\r\n\r\n' % len(jpeg))
                            self.wfile.write(jpeg + b'\r\n')
                        time.sleep(0.03)
                else:
                    self.send_error(404)
            except (BrokenPipeError, ConnectionResetError):
                pass
    return Handler


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('command', nargs='?', default='view',
                   choices=['view', 'test', 'grasp', 'release', 'drop', 'calibrate', 'serve'])
    p.add_argument('--drop', action='store_true',
                   help='test: end by dropping (pump off in the air) instead of putting it down')
    p.add_argument('--drop-wait', type=float, default=1.5,
                   help='seconds after pump off for the object to fall')
    p.add_argument('--color', choices=sorted(COLORS),
                   help='object to grasp; required for view, test and grasp (course_nav will '
                        'pass the colour it stopped at)')
    p.add_argument('--dry-run', action='store_true', help='view and plan only; no arm/pump commands')
    p.add_argument('--port', type=int, default=8081, help='live view port')
    p.add_argument('--fps', type=float, default=10)
    p.add_argument('--calibration-file', default='~/thefile.txt')
    p.add_argument('--min-slope', type=float, default=0.2, help='refuse slopes below this, mm/px')
    p.add_argument('--force-calibration', action='store_true')
    p.add_argument('--dx', type=float, default=0, help='add to aimed arm x, mm (forward)')
    p.add_argument('--dy', type=float, default=0, help='add to aimed arm y, mm (+ is LEFT)')
    p.add_argument('--image-topic', default='/camera/rgb/image_raw')
    p.add_argument('--min-row', type=int, default=150, help='ignore blobs above this image row')
    p.add_argument('--min-area', type=float, default=150)
    p.add_argument('--stable-frames', type=int, default=20)
    p.add_argument('--pixel-tolerance', type=float, default=2)
    p.add_argument('--detect-timeout', type=float, default=15)
    p.add_argument('--hover-z', type=float, default=20)
    p.add_argument('--blue-grasp-z', type=float, default=-60,
                   help="the blue block; OneKey 9's -50 missed it in the full run, 1 cm deeper")
    p.add_argument('--yellow-grasp-z', type=float, default=-120,
                   help='the lighter, small yellow cube (-105 suits the darker, larger one)')
    p.add_argument('--blue-radial', type=float, default=1.0,
                   help="scale blue's aim outward from the arm base (height parallax); 1 = none")
    p.add_argument('--yellow-radial', type=float, default=1.075,
                   help="scale yellow's aim outward from the arm base: its top is far below the "
                        "calibration marker, so it looks nearer than it is (about 15 mm at 20 cm)")
    p.add_argument('--self-mask', nargs=4, type=int, default=[344, 304, 390, 358],
                   metavar=('U0', 'V0', 'U1', 'V1'),
                   help='image box ignored by colour detection: the arm marker at the stow pose '
                        '(measured at u 349-384, v 309-352). Re-measure if the stow pose or '
                        'camera changes; --self-mask 0 0 0 0 turns it off')
    p.add_argument('--near-radius', type=float, default=60,
                   help='"grasp <colour> U V": only blobs within this many px of (U, V) count')
    p.add_argument('--aim-point', choices=['top', 'centroid'], default='top',
                   help='top: centre of the top face (yellow) / of the rotated box (blue); '
                        'centroid: centre of the whole colour area, side faces included')
    p.add_argument('--release-clearance', type=float, default=5,
                   help='put the object down this far above the grasp height, mm')
    p.add_argument('--move-wait', type=float, default=3, help='seconds per arm move')
    p.add_argument('--pump-wait', type=float, default=1)
    p.add_argument('--onekey-gap', type=float, default=0.1,
                   help='seconds between the queued grasp commands (OneKey 9: rospy.Rate(10))')
    p.add_argument('--lift-wait', type=float, default=7.0,
                   help='seconds after queuing the grasp, until the arm is back at stow')
    p.add_argument('--home-pose', nargs=3, type=float, default=[120, 0, 35])
    p.add_argument('--stow-pose', nargs=3, type=float, default=[120, 0, 35],
                   help='arm tucked in, centred (OneKey home pose); every grasp starts here, '
                        'and serve "stow" parks it here for driving')
    p.add_argument('--hold', type=float, default=3, help='test: seconds to hold before releasing')
    p.add_argument('--linger', type=float, default=20,
                   help='test/grasp/release: keep the live view up this long afterwards')
    p.add_argument('--cal-z', type=float, default=-110, help='calibrate: cup height (OneKey 8: -110)')
    p.add_argument('--cal-hover-z', type=float, default=-60, help='calibrate: height between points')
    p.add_argument('--cal-max-r', type=float, default=280, help='calibrate: farthest point, mm')
    p.add_argument('--cal-wait', type=float, default=2.5, help='calibrate: seconds per move')
    p.add_argument('--cal-max-rms', type=float, default=8, help='calibrate: reject a worse fit, mm')
    p.add_argument('--output-dir', default='~/grasp_test_images')
    p.add_argument('--min-xy', type=float, default=80)
    p.add_argument('--max-xy', type=float, default=300, help='SwiftPro reaches about 320 mm')
    p.add_argument('--z-min', type=float, default=-125,
                   help="lowest grasp z; OneKey's -110 is not the floor here (that is about -140)")
    args = p.parse_args(argv)
    if args.command in ('view', 'test', 'grasp') and args.color is None:
        p.error('%s needs --color blue or --color yellow' % args.command)
    return args


def main():
    args = parse_args(rospy.myargv()[1:])
    rospy.init_node('course_grasp', anonymous=False)
    g = Grasper(args)
    g.serve_view()
    try:
        if args.command == 'view':
            g.step('viewing only (no arm commands)')
            rospy.spin()
            return 0
        if args.command == 'serve':
            g.serve()
            return 0
        if args.command == 'calibrate':
            g.calibrate()
        if args.command in ('test', 'grasp'):
            g.grasp(args.color)
        if args.command == 'test':
            g.pause(args.hold)
        if args.command == 'drop' or (args.command == 'test' and args.drop):
            g.drop()
        elif args.command in ('test', 'release'):
            g.release()
        g.step('done: %s' % args.command)
        g.pause(args.linger)
        return 0
    except (RuntimeError, ValueError, OSError, cv2.error) as exc:
        g.step('FAILED: %s' % exc)
        rospy.logerr('%s', exc)
        try:
            g.pause(args.linger)
        except RuntimeError:
            pass
        return 1
    finally:
        g.close()


if __name__ == '__main__':
    raise SystemExit(main())
