#!/usr/bin/env python
# motion_controller.py

import rospy
import roslib

# Dynamixel connection
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import MultiArrayLayout

# IMU message
from xsens_msgs.msg import orientationEstimate

# Debug message
from std_msgs.msg import Int32,String

import numpy as np
import sys

def main():
    offset = np.array([38,-20])

    rospy.init_node('motion_controller',anonymous=True)

    pub0 = rospy.Publisher('/program_state', Int32, queue_size=1)
    pub1 = rospy.Publisher('/dxl_target/Target_position', Int32MultiArray, queue_size=1)

    gear_ratio = 18/40

    dynamixel_goal = np.array([-808-offset[0],-790-offset[1]])
    publish_array = Int32MultiArray(data = dynamixel_goal)

    state = input()

    while not rospy.is_shutdown():

        print("RUNNING")

        pub0.publish(state)

        if(state == 1):
            pub1.publish(publish_array)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException: pass