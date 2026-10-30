#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Run the whole course: course_nav.py drives, course_grasp.py grasps, this node says when.

The task: from the start, drive round the obstacle to the blue block, grasp it, carry it back
round the obstacle past the start line (still facing home, so the arm points away from the
finish line), drop it, turn around, then do the same for the yellow block.

This node only relays messages; it never moves the base or the arm itself.
  course_nav.py round-trip --targets blue,yellow --handoff
      drives; publishes /course_nav/status and waits on /course_nav/continue at the grasp
      point (end) and the release point (past the start line), then turns around.
  course_grasp.py serve
      moves the arm; takes /course_grasp/command, answers on /course_grasp/status.

What it does:
  on start                       "stow"            arm centred and tucked before any driving
  course_nav IN_REACH {color}    remember the colour in front of the arm
  course_nav WAITING grasp       "grasp <colour> <u> <v>" (that object: reach, grasp, back to
                                 stow)  -> continue "done"
  course_nav WAITING release     "drop" (pump off at stow, no arm motion)  -> continue "done"
  course_nav DONE / FAILED       exit
A grasp the arm reports as failed is retried (--grasp-attempts); if it still fails, or anything
else goes wrong, course_nav is told "abort" and stays put.

On the Spark, after the navigation stack and the arm driver, each in its own terminal:
  python3 course_grasp.py serve
  python3 course_coordinator.py
  python3 course_nav.py localize          # robot on the start pose; expect x~0 y~0 yaw~63
  python3 course_nav.py round-trip --targets blue,yellow --handoff --stop-row 320 --hazard-margin 0
"""
import argparse
import json
import threading
import time

import rospy
from std_msgs.msg import String


class Coordinator:
    def __init__(self, args):
        self.args = args
        self.in_reach = None            # colour course_nav last stopped within reach of
        self.in_reach_pixel = None
        self.holding = None
        self.started = False            # course_nav has begun a round trip
        self.reply = None
        self.reply_cv = threading.Condition()
        self.busy = threading.Lock()
        self.finished = threading.Event()
        self.grasp_cmd = rospy.Publisher('/course_grasp/command', String, queue_size=5)
        self.nav_continue = rospy.Publisher('/course_nav/continue', String, queue_size=1)
        rospy.Subscriber('/course_grasp/status', String, self.on_grasp_status, queue_size=10)
        rospy.Subscriber('/course_nav/status', String, self.on_nav_status, queue_size=10)

    # ------------------------------------------------------------------- arm
    def on_grasp_status(self, msg):
        try:
            record = json.loads(msg.data)
        except ValueError:
            return
        with self.reply_cv:
            self.reply = record
            self.reply_cv.notify_all()

    def connect_arm(self):
        end = time.time() + 30
        while self.grasp_cmd.get_num_connections() == 0:
            if time.time() > end or rospy.is_shutdown():
                raise RuntimeError('course_grasp.py serve is not running')
            rospy.sleep(0.1)
        rospy.sleep(1.0)                # let its latched old status arrive before we ask

    def arm(self, command, timeout):
        """Send one course_grasp command and wait for its answer."""
        self.connect_arm()
        with self.reply_cv:
            self.reply = None
        rospy.loginfo('-> course_grasp: %s', command)
        self.grasp_cmd.publish(String(data=command))
        end = time.time() + timeout
        with self.reply_cv:
            while self.reply is None and not rospy.is_shutdown():
                left = end - time.time()
                if left <= 0:
                    raise RuntimeError('course_grasp did not answer %r within %.0f s'
                                       % (command, timeout))
                self.reply_cv.wait(min(left, 0.5))
            reply = self.reply
        rospy.loginfo('<- course_grasp: %s', json.dumps(reply))
        if not reply or not reply.get('ok'):
            raise RuntimeError('course_grasp could not %s: %s'
                               % (command, (reply or {}).get('error', 'no answer')))
        return reply

    # ------------------------------------------------------------ navigation
    def on_nav_status(self, msg):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        state = s.get('state')
        rospy.loginfo('course_nav: %s', msg.data)
        if state == 'TO_END':
            self.started = True
        if state in ('TO_END', 'NOT_IN_REACH'):
            self.in_reach = None
        elif state == 'IN_REACH':
            self.in_reach = s.get('color')
            self.in_reach_pixel = s.get('pixel')     # where course_nav sees it, [u, v]
        elif state == 'WAITING':
            threading.Thread(target=self.handoff, args=(s.get('waiting_for'),),
                             daemon=True).start()
        elif state in ('DONE', 'FAILED') and self.started:   # not a failed `localize` beforehand
            rospy.loginfo('Course %s.', 'finished' if state == 'DONE' else
                          'stopped: %s' % s.get('error'))
            self.finished.set()

    def handoff(self, waiting_for):
        if not self.busy.acquire(False):
            return                      # a repeated WAITING while we are already on it
        try:
            if waiting_for == 'grasp':
                self.grasp()
            elif waiting_for == 'release':
                if self.holding:
                    self.arm('drop', self.args.move_timeout)
                    rospy.loginfo('Dropped the %s object past the start line.', self.holding)
                    self.holding = None
                else:
                    rospy.logwarn('Nothing held; nothing to drop.')
            rospy.loginfo('-> course_nav: done')
            self.nav_continue.publish(String(data='done'))
        except RuntimeError as exc:
            rospy.logerr('%s. Telling course_nav to abort (it stays put).', exc)
            self.nav_continue.publish(String(data='abort'))
        finally:
            self.busy.release()

    def grasp(self):
        color = self.in_reach
        if color is None:
            raise RuntimeError('course_nav is waiting for a grasp but reported nothing in reach')
        # Name the very object course_nav stopped at, so the arm cannot pick something else
        # of that colour (it once took its own orange marker for yellow).
        command = 'grasp %s' % color
        pixel = self.in_reach_pixel
        if isinstance(pixel, list) and len(pixel) == 2:
            command += ' %d %d' % (pixel[0], pixel[1])
        for attempt in range(1, self.args.grasp_attempts + 1):
            try:
                self.arm(command, self.args.grasp_timeout)
                break
            except RuntimeError as exc:
                if attempt == self.args.grasp_attempts:
                    raise
                rospy.logwarn('%s; retrying (%d/%d).', exc, attempt + 1, self.args.grasp_attempts)
        self.holding = color            # the grasp ends back at stow, carrying it


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--grasp-attempts', type=int, default=2)
    p.add_argument('--grasp-timeout', type=float, default=60, help='seconds per grasp command')
    p.add_argument('--move-timeout', type=float, default=30, help='seconds for stow/drop')
    p.add_argument('--no-stow', action='store_true', help='do not tuck the arm at start-up')
    args = p.parse_args(rospy.myargv()[1:])
    rospy.init_node('course_coordinator')
    c = Coordinator(args)
    try:
        if not args.no_stow:
            c.arm('stow', args.move_timeout)
        rospy.loginfo('Ready: start course_nav.py round-trip --targets blue,yellow --handoff')
        while not rospy.is_shutdown() and not c.finished.wait(0.5):
            pass
    except RuntimeError as exc:
        rospy.logerr('%s', exc)
        raise SystemExit(1)


if __name__ == '__main__':
    main()
