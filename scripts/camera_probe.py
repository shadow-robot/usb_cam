#!/usr/bin/env python

# Copyright (C) 2021 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.


import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from rh_msgs.srv import RhCameraReset
from std_srvs.srv import Trigger, TriggerResponse
from time import ctime
import os
import serial
from threading import Lock

class CX3HubResetter:
    def __init__(self, port='/dev/ttyUSB0' ):
        #self._serial = serial.Serial(port)
        self._mutex = Lock()
        self._run = ['a', 's', 'd']
        self._rst = ['q', 'w', 'd']


    def reset(self, which):
        self._mutex.acquire()
        #self._serial.write(self._rst[which])
        rospy.sleep(1)
        #self._serial.write(self._run[which])
        self._mutex.release()

class Test(object):
    def __init__(self, finger, hub_reset=None, try_reset=True, die_on_fail=True):

        self._reset_count = 0
        self._hub_reset = hub_reset

        self._try_reset = try_reset
        self._die_on_fail = die_on_fail

        self._last_time = rospy.Time.now()
        self._last_msg = rospy.Time.now()

        self._timedout = False
        self._dead = False
        self._finger = finger
        self._namespace = "usb_cam_%d" % finger

        if try_reset:
            self._reset = rospy.ServiceProxy(
                "/rh_finger/F%d/tactiles/F%d_SPARKLE/reset_camera" % (finger, finger) , RhCameraReset)

        self._pub = rospy.Publisher("/%s/monitor" % self._namespace, Int16, queue_size=5)

        self._resetting = False
        self._given_up = False

    def start(self):
        self._t = rospy.Timer(rospy.Duration(0.1), self.timer_cb)
        self._sub = rospy.Subscriber("/%s/image_raw" % self._namespace, Image, self.cb, queue_size=3)
        self.message("Camera %d monitor started." % self._finger, 1)

    def _trigger_cb(self, request):
        self._resetting = True
        self._reset(20000, 3)
        self._last_time = rospy.Time.now()
        self._resetting = False
        return TriggerResponse(True, "Reset complete")


    def message(self, message, int_out):
        rospy.logerr("%s %s" % (message, ctime(rospy.Time.now().secs)))
        self._pub.publish(int_out)
        self._pub.publish(1 - int_out)
        self._pub.publish(int_out)

    def cb(self, msg):
        self._last_time = rospy.Time.now()

        if self._resetting:
            self._last_time = rospy.Time.now()
            return

        if self._timedout:
            self.message("Camera %d topic resumed." % self._finger, 1)
            self._timedout = False
            self._dead = False
            self._given_up = False
            self._reset_count = 0


    def timer_cb(self, event):
        if not self._given_up and rospy.Time.now() - self._last_time > rospy.Duration(10):
            self._given_up = True
            self.message("Camera %d didn't recover." % self._finger, 0)
            if self._die_on_fail:
                os.system("killall roslaunch")
                os.system("killall python")

        elif self._reset_count == 1 and rospy.Time.now() - self._last_time > rospy.Duration(3):
            self._reset_count = 2
            self.message("Camera %d, resetting" % self._finger, 0)
            self._reset(20000, 2)

        elif not self._dead and rospy.Time.now() - self._last_time > rospy.Duration(1):
            self._dead = True
            self.message("Camera %d died" % self._finger, 0)
            if self._try_reset and self._reset_count == 0:
                self._reset_count = 1
                self.message("Camera %d, resetting" % self._finger, 0)
                self._reset(20000, 2)

        elif rospy.Time.now() - self._last_time > rospy.Duration(1.0/40.0):
            if not self._timedout:
                self._timedout = True
                self.message("Camera %d timer timeout" % self._finger, 0)



if __name__ == "__main__":
    rospy.init_node("camtest")
    rospy.sleep(1)

    n_sensors = rospy.get_param("~sensor_count", 3)
    die_on_fail = rospy.get_param("~die_on_fail", True)
    try_reset = rospy.get_param("~try_reset", True)

    hub_reset = CX3HubResetter()


    sensor_testers = [Test(x, hub_reset, try_reset, die_on_fail) for x in range(n_sensors)]

    rospy.sleep(1)



    for t in sensor_testers:
        t.start()

    rospy.spin()
