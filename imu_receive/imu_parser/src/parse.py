#!/usr/bin/env python

import roslib
import rospy

from std_msgs.msg import Header, Float32, Float64
from sensor_msgs.msg import Imu
from xsens_msgs.msg import orientationEstimate

import numpy
import sys

roll = 0.0
pitch = 0.0
yaw = 0.0

def callback_euler(data):
    global roll, pitch, yaw

    roll = data.roll
    pitch = data.pitch
    yaw = data.yaw
    sys.stdout.write("\rroll:%7.2f | pitch:%7.2f | yaw:%7.2f" % (roll, pitch, yaw))
    sys.stdout.flush()

def imu_parser():
    rospy.init_node('imu_parser', anonymous=True)

    pub0 = rospy.Publisher('imu/euler/roll', Float64, queue_size=10)
    pub1 = rospy.Publisher('imu/euler/pitch', Float64, queue_size=10)
    pub2 = rospy.Publisher('imu/euler/yaw', Float64, queue_size=10)

    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.Subscriber("mti/filter/orientation", orientationEstimate, callback_euler)

        pub0.publish(roll)
        pub1.publish(pitch)
        pub2.publish(yaw)

        r.sleep()
    #rospy.spin()

if __name__ == '__main__':
    try:
        imu_parser()
    except rospy.ROSInterruptException: pass
