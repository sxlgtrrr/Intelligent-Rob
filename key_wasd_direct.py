#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import tty
import termios
import select
import threading

import rospy
import rospkg

from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray

sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
import motion.bodyhub_client as bodycli

STEP_LEN = [0.10, 0.05, 0.0]
PUBLISH_HZ = 8.0


class WasdDirectTele(object):
    def __init__(self):
        rospy.init_node('key_wasd_direct', anonymous=True)
        time.sleep(0.2)
        rospy.on_shutdown(self.on_shutdown)

        self.bodyhub = bodycli.BodyhubClient(2)
        self.gait_cmd_pub = rospy.Publisher('/gaitCommand', Float64MultiArray, queue_size=2)
        self.gait_req_sub = rospy.Subscriber('/requestGaitCommand', Bool, self.on_gait_request)

        self.current_cmd = None
        self.walking_enabled = False
        self.lock = threading.Lock()

    def on_shutdown(self):
        try:
            self.stop_motion()
        except Exception:
            pass

    def enter_walking_mode(self):
        if self.bodyhub.walk() is False:
            rospy.logerr('bodyhub to walk failed!')
            rospy.signal_shutdown('error')
            sys.exit(1)
        self.walking_enabled = True

    def ensure_walking_mode(self):
        if not self.walking_enabled:
            self.enter_walking_mode()

    def stop_motion(self):
        with self.lock:
            self.current_cmd = [0.0, 0.0, 0.0]

        for _ in range(3):
            self.gait_cmd_pub.publish(data=[0.0, 0.0, 0.0])
            time.sleep(0.05)

        try:
            self.bodyhub.reset()
        finally:
            with self.lock:
                self.current_cmd = None
            self.walking_enabled = False

    def publish_current_command(self):
        with self.lock:
            cmd = list(self.current_cmd) if self.current_cmd is not None else None

        if self.walking_enabled and cmd is not None:
            self.gait_cmd_pub.publish(data=cmd)

    def on_gait_request(self, msg):
        if msg.data:
            self.publish_current_command()

    def print_help(self):
        print('')
        print('w a s d : continuous move')
        print('space   : stop')
        print('q       : quit')
        print('')
        print('Press once to change direction; movement continues until next command.')
        print('')

    def get_key(self, timeout):
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        try:
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                return sys.stdin.read(1)
            return None
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def key_to_command(self, key):
        key = key.lower()
        if key == 'w':
            return [STEP_LEN[0], 0.0, 0.0]
        if key == 's':
            return [-STEP_LEN[0] * 0.8, 0.0, 0.0]
        if key == 'a':
            return [0.0, STEP_LEN[1], 0.0]
        if key == 'd':
            return [0.0, -STEP_LEN[1], 0.0]
        if key == ' ':
            return [0.0, 0.0, 0.0]
        return None

    def keyboard_thread(self):
        while not rospy.is_shutdown():
            key = self.get_key(0.1)
            if key is None:
                continue
            if key.lower() == 'q':
                rospy.signal_shutdown('exit')
                return

            if key == ' ':
                self.stop_motion()
                rospy.loginfo('key=%s stop', repr(key))
                continue

            cmd = self.key_to_command(key)
            if cmd is None:
                continue

            self.ensure_walking_mode()

            with self.lock:
                self.current_cmd = cmd

            self.gait_cmd_pub.publish(data=cmd)
            rospy.loginfo('key=%s cmd=%s', repr(key), cmd)

    def start(self):
        self.stop_motion()
        self.print_help()

        key_worker = threading.Thread(target=self.keyboard_thread)
        key_worker.daemon = True
        key_worker.start()

        rospy.spin()


if __name__ == '__main__':
    WasdDirectTele().start()
