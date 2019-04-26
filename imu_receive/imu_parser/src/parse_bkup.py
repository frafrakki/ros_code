#!/usr/bin/env python

import roslib
import rospy

from std_msgs.msg import Header, Float32, Float64
from sensor_msgs.msg import Imu
from xsens_msgs.msg import orientationEstimate

import numpy
import sys
# from __future__ import print_function

def callback_euler(data):
    roll = data.roll
    pitch = data.pitch
    yaw = data.yaw
    print("roll:%7.2f | pitch:%7.2f | yaw:%7.2f" % (roll, pitch, yaw))
    sys.stdout.flush()

def imu_parser():
    rospy.init_node('imu_parser', anonymous=True)
    rospy.Subscriber("mti/filter/orientation", orientationEstimate, callback_euler)

    rospy.spin()

if __name__ == '__main__':
    imu_parser()
