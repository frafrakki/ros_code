#!/usr/bin/env python
# motion_controller.py

import rospy
import roslib

# Dynamixel connection
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import MultiArrayLayout

# encoder message
from std_msgs.msg import Float64

# IMU message
from xsens_msgs.msg import orientationEstimate

# Debug message
from std_msgs.msg import Int32,String

import numpy as np
import sys

encoder_angle = 0.0
start_angle = -179.5

def encoder_callback(data):
    global encoder_angle
    encoder_angle = data.data

    rospy.loginfo("ENC :%f",encoder_angle)

def main():
    offset = np.array([41,-21])

    rospy.init_node('motion_controller',anonymous=True)

    pub0 = rospy.Publisher('/program_state', Int32, queue_size=1)
    pub1 = rospy.Publisher('/dxl_target/Target_position', Int32MultiArray, queue_size=1)

    gear_ratio = 18/40

    dynamixel_goal = np.array([-808-offset[0],-790-offset[1]])
    publish_array = Int32MultiArray(data = dynamixel_goal)

    state = input()

    while not rospy.is_shutdown():

        # print("RUNNING")

        pub0.publish(state)
        rospy.Subscriber("Angle", Float64, encoder_callback)

        if(encoder_angle >= start_angle):
            pub1.publish(publish_array)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException: pass