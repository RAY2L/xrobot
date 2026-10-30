#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Stream the Spark camera to a browser: open http://<robot-ip>:8080/ on any computer on the
same network. Read-only: it never moves the robot or the arm.

The overlay shows what course_nav.py looks for: blue/yellow detections (boxes) and, once
taught, the reference row and column for each colour (dashed lines).

    rosrun move2grasp camera_stream.py            # or: python3 camera_stream.py --port 8080
"""

import argparse
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

COLORS = {
    'blue': (([95, 70, 60], [140, 255, 255]), (255, 80, 0)),       # grasp_pro_new.py
    'yellow': (([20, 65, 63], [35, 255, 255]), (0, 220, 255)),     # sagittarius_carry hsv.yaml
}
MIN_ROW = 150   # same as course_nav.py --min-row
STOP_ROW = 400  # same as course_nav.py --stop-row
REACH_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'course_nav_reach.json')
PAGE = b"""<!doctype html><html><head><meta charset="utf-8"><title>Spark camera</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>body{margin:0;background:#111;color:#ddd;font:14px sans-serif;text-align:center}
img{max-width:100%;height:auto;margin-top:8px}</style></head>
<body><div>Spark camera &middot; /camera/rgb/image_raw</div><img src="/stream"></body></html>"""


class Streamer:
    def __init__(self, args):
        self.args = args
        self.bridge = CvBridge()
        self.jpeg = None
        self.cond = threading.Condition()
        self.last = 0.0
        rospy.Subscriber(args.topic, Image, self.on_image, queue_size=1, buff_size=2 ** 24)

    def on_image(self, msg):
        now = time.time()
        if now - self.last < 1.0 / self.args.fps:
            return
        self.last = now
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError:
            return
        if not self.args.no_overlay:
            self.overlay(image)
        ok, buf = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, self.args.quality])
        if ok:
            with self.cond:
                self.jpeg = buf.tobytes()
                self.cond.notify_all()

    def overlay(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        for name, ((lo, hi), bgr) in COLORS.items():
            mask = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                if cv2.contourArea(c) >= 150:
                    x, y, w, h = cv2.boundingRect(c)
                    cv2.rectangle(image, (x, y), (x + w, y + h), bgr, 2)
                    cv2.putText(image, name, (x, max(12, y - 4)), cv2.FONT_HERSHEY_SIMPLEX,
                                0.45, bgr, 1)
        try:
            with open(REACH_FILE) as stream:
                reach = json.load(stream)
        except (IOError, ValueError):
            reach = {}
        # stop lines: taught per colour, else course_nav's default --stop-row
        lines = {name: (int(ref['bottom']), int(ref['u'])) for name, ref in reach.items()
                 if 'bottom' in ref} or {'default': (STOP_ROW, None)}
        for name, (row, col) in lines.items():
            bgr = COLORS.get(name, ((None, None), (0, 0, 255)))[1]
            for x0 in range(0, image.shape[1], 16):
                cv2.line(image, (x0, row), (x0 + 8, row), bgr, 1)
            if col is not None:
                for y0 in range(0, image.shape[0], 16):
                    cv2.line(image, (col, y0), (col, y0 + 8), bgr, 1)
            cv2.putText(image, '%s stop line' % name, (6, row - 5), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, bgr, 1)
        for x0 in range(0, image.shape[1], 24):          # course_nav ignores blobs above this
            cv2.line(image, (x0, MIN_ROW), (x0 + 6, MIN_ROW), (160, 160, 160), 1)
        cv2.putText(image, time.strftime('%H:%M:%S'), (image.shape[1] - 80, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def frames(self):
        while not rospy.is_shutdown():
            with self.cond:
                self.cond.wait(timeout=2.0)
                frame = self.jpeg
            if frame is not None:
                yield frame


def make_handler(streamer):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.end_headers()
                self.wfile.write(PAGE)
            elif self.path == '/snapshot.jpg' and streamer.jpeg is not None:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.end_headers()
                self.wfile.write(streamer.jpeg)
            elif self.path == '/stream':
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                try:
                    for frame in streamer.frames():
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n')
                        self.wfile.write(b'Content-Length: %d\r\n\r\n' % len(frame))
                        self.wfile.write(frame + b'\r\n')
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_error(404)
    return Handler


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--port', type=int, default=8080)
    p.add_argument('--topic', default='/camera/rgb/image_raw')
    p.add_argument('--fps', type=float, default=10.0)
    p.add_argument('--quality', type=int, default=70)
    p.add_argument('--no-overlay', action='store_true')
    args = p.parse_args(rospy.myargv()[1:])
    rospy.init_node('camera_stream', anonymous=True)
    streamer = Streamer(args)
    server = ThreadingHTTPServer(('0.0.0.0', args.port), make_handler(streamer))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    rospy.loginfo('Camera stream on http://0.0.0.0:%d/', args.port)
    rospy.spin()
    server.shutdown()


if __name__ == '__main__':
    main()
