#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import os
import glob
import struct
import select
import termios
import tty

try:
    import Tkinter as tk
except ImportError:
    import tkinter as tk

import rospy
import rospkg

from bodyhub.srv import SrvString
from bodyhub.srv import SrvTLSstring

sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
import motion.bodyhub_client as bodycli

CONTROL_ID = 2
WINDOW_WIDTH = 640
WINDOW_HEIGHT = 400
PAN_LIMIT = 75.0
TILT_LIMIT = 25.0
PUBLISH_INTERVAL_MS = 40
RAW_INPUT_EVENT_FMT = 'llHHI'
EV_REL = 0x02
EV_KEY = 0x01
REL_X = 0x00
REL_Y = 0x01
BTN_LEFT = 0x110
RAW_SCALE_X = 0.15
RAW_SCALE_Y = 0.10
RAW_MOUSE_CANDIDATES = [
    '/dev/input/by-id/*event-mouse',
    '/dev/input/event15',
    '/dev/input/mice',
    '/dev/input/mouse0',
]


class MouseHeadTele(object):
    def __init__(self):
        rospy.init_node('mouse_head_tele', anonymous=True)

        self.bodyhub = bodycli.BodyhubClient(CONTROL_ID)
        self.pan = 0.0
        self.tilt = 0.0
        self.last_publish_time = 0.0

        self.master_client = rospy.ServiceProxy('MediumSize/BodyHub/GetMasterID', SrvTLSstring)
        self.status_client = rospy.ServiceProxy('MediumSize/BodyHub/GetStatus', SrvString)

        self.ensure_head_control_ready()
        self.root = None
        self.canvas = None
        self.cursor_text = None
        self.raw_mouse_path = None

        if os.environ.get('DISPLAY'):
            self.init_ui()
        else:
            self.raw_mouse_path = self.find_raw_mouse_device()

        self.publish_head_position(force=True)

    def ensure_head_control_ready(self):
        rospy.wait_for_service('MediumSize/BodyHub/GetMasterID', 3)
        rospy.wait_for_service('MediumSize/BodyHub/GetStatus', 3)

        master_id = self.master_client('get').data
        status = self.status_client('get').data

        if master_id not in (0, CONTROL_ID):
            raise RuntimeError('bodyhub busy, current master id is %s' % master_id)

        # If no one owns bodyhub yet, switch to ready so head commands are accepted.
        if status == 'preReady':
            if self.bodyhub.ready() is not True:
                raise RuntimeError('bodyhub to ready failed')

    def init_ui(self):
        self.root = tk.Tk()
        self.root.title('Mouse Head Teleop')
        self.root.geometry('%dx%d' % (WINDOW_WIDTH, WINDOW_HEIGHT))
        self.root.resizable(False, False)

        self.label = tk.Label(
            self.root,
            text=(
                '鼠标在窗口内移动: 控制头部和摄像头\n'
                '空格: 回中    q / Esc: 退出'
            ),
            justify='center'
        )
        self.label.pack(pady=10)

        self.canvas = tk.Canvas(self.root, width=WINDOW_WIDTH, height=WINDOW_HEIGHT - 80, bg='black')
        self.canvas.pack()
        self.draw_crosshair()

        self.canvas.bind('<Motion>', self.on_mouse_move)
        self.root.bind('<space>', self.on_reset)
        self.root.bind('<Key-q>', self.on_quit)
        self.root.bind('<Escape>', self.on_quit)
        self.root.protocol('WM_DELETE_WINDOW', self.on_quit)

        self.canvas.focus_set()

    def find_raw_mouse_device(self):
        for pattern in RAW_MOUSE_CANDIDATES:
            for path in sorted(glob.glob(pattern)):
                if os.path.exists(path) and os.access(path, os.R_OK):
                    return path

        checked = []
        for pattern in RAW_MOUSE_CANDIDATES:
            checked.extend(sorted(glob.glob(pattern)))
        checked = checked or ['/dev/input/by-id/*event-mouse', '/dev/input/event*']

        raise RuntimeError(
            'no DISPLAY, and no readable mouse device found. '
            'Try running in desktop, or grant read permission to your mouse event device, '
            'for example: sudo chmod a+r /dev/input/event15'
        )

    def draw_crosshair(self):
        cx = WINDOW_WIDTH / 2
        cy = (WINDOW_HEIGHT - 80) / 2
        self.canvas.create_line(cx - 20, cy, cx + 20, cy, fill='green', width=2)
        self.canvas.create_line(cx, cy - 20, cx, cy + 20, fill='green', width=2)
        self.cursor_text = self.canvas.create_text(
            10, 10, anchor='nw', fill='white', text='pan=0.0 tilt=0.0'
        )

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def on_mouse_move(self, event):
        usable_height = WINDOW_HEIGHT - 80.0
        x_ratio = (float(event.x) / float(WINDOW_WIDTH)) * 2.0 - 1.0
        y_ratio = (float(event.y) / float(usable_height)) * 2.0 - 1.0

        self.pan = self.clamp(x_ratio * PAN_LIMIT, -PAN_LIMIT, PAN_LIMIT)
        self.tilt = self.clamp(-y_ratio * TILT_LIMIT, -TILT_LIMIT, TILT_LIMIT)

        self.publish_head_position()

    def on_reset(self, _event=None):
        self.pan = 0.0
        self.tilt = 0.0
        self.publish_head_position(force=True)

    def on_quit(self, _event=None):
        try:
            self.on_reset()
        except Exception:
            pass
        self.root.quit()
        self.root.destroy()

    def publish_head_position(self, force=False):
        now = time.time()
        if not force and (now - self.last_publish_time) * 1000.0 < PUBLISH_INTERVAL_MS:
            return

        self.last_publish_time = now
        self.bodyhub.set_head_position([self.pan, self.tilt])
        if self.canvas is not None and self.cursor_text is not None:
            self.canvas.itemconfig(
                self.cursor_text,
                text='pan=%.1f tilt=%.1f' % (self.pan, self.tilt)
            )
        rospy.loginfo_throttle(0.5, 'head pan=%.1f tilt=%.1f' % (self.pan, self.tilt))

    def run_raw_mode(self):
        print('')
        print('Raw mouse mode: %s' % self.raw_mouse_path)
        print('move mouse : control head')
        print('space      : center head')
        print('q          : quit')
        print('left click : center head')
        print('')

        event_size = struct.calcsize(RAW_INPUT_EVENT_FMT)
        mouse_fd = os.open(self.raw_mouse_path, os.O_RDONLY | os.O_NONBLOCK)

        stdin_fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(stdin_fd)
        tty.setcbreak(stdin_fd)

        try:
            while not rospy.is_shutdown():
                read_list, _, _ = select.select([mouse_fd, stdin_fd], [], [], 0.1)

                if stdin_fd in read_list:
                    key = os.read(stdin_fd, 1)
                    if key in ('q', 'Q'):
                        break
                    if key == ' ':
                        self.on_reset()

                if mouse_fd in read_list:
                    data = os.read(mouse_fd, event_size)
                    if len(data) != event_size:
                        continue

                    _, _, ev_type, ev_code, ev_value = struct.unpack(RAW_INPUT_EVENT_FMT, data)

                    if ev_type == EV_REL:
                        if ev_code == REL_X:
                            self.pan = self.clamp(self.pan + ev_value * RAW_SCALE_X, -PAN_LIMIT, PAN_LIMIT)
                            self.publish_head_position()
                        elif ev_code == REL_Y:
                            self.tilt = self.clamp(self.tilt - ev_value * RAW_SCALE_Y, -TILT_LIMIT, TILT_LIMIT)
                            self.publish_head_position()
                    elif ev_type == EV_KEY and ev_code == BTN_LEFT and ev_value == 1:
                        self.on_reset()
        finally:
            termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_settings)
            os.close(mouse_fd)

    def run(self):
        if self.root is not None:
            self.root.mainloop()
        else:
            self.run_raw_mode()


if __name__ == '__main__':
    try:
        MouseHeadTele().run()
    except Exception as err:
        rospy.logerr(str(err))
        sys.exit(1)
