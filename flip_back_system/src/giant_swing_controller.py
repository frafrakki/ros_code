#!/usr/bin/env python
# giant_swing_controller.py

import rospy
import roslib

# Dynamixel connection
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import MultiArrayLayout

from std_msgs.msg import Float64

# IMU message
from xsens_msgs.msg import orientationEstimate

# Debug message
from std_msgs.msg import Int32,String

import numpy as np
import sys

# global variables
present_dynamixel_position = np.array([0.,0.])
present_dynamixel_velocity = np.array([0.,0.])
present_robot_position = 0.

# constant dynamixel variables
gear_ratio = 18/40
position_scaling_factor = 360/4096 # deg/Dynamixel VALUE
velocity_scaling_factor = 0.299 # rpm/Dynamixel VALUE
current_scaling_factor =  2.69 # mA/Dynamixel VALUE
torque_constant = 0.79 # Nm/A

# mechanical variables
l1 = 0.363
d1 = 0.240
m1 = 1.234

I1 = 0.080042027

l2 = 0.273
d2 = 0.115
m2 = 0.645
I2 = 0.014397165

g = 9.81

# control variables
kd = 200
kp = 2500

alpha = np.pi/6

position_offset = np.array([37,4081])

def control_signal_definition(th1,th2,th2d,thdot1,thdot2):

    t2 = d2**2
    t3 = m2*t2
    t5 = np.cos(th2)
    t9 = d2*l1*m2*t5
    t4 = I2+t3+t9
    t6 = np.cos(th1)
    t7 = th1+th2
    t8 = np.cos(t7)
    t10 = d1**2
    t11 = m1*t10
    t12 = l1**2
    t13 = m2*t12
    t14 = d2*l1*m2*t5*2.0
    t15 = I1+I2+t3+t11+t13+t14
    t16 = 1.0/t15
    t17 = np.sin(th2)
    Tc = -(kd*thdot2+kp*(th2-th2d))*(I2+t3-t4**2*t16)-g*t4*t16*(d1*m1*t6+d2*m2*t8+l1*m2*t6)+d2*g*m2*t8+d2*l1*m2*t17*thdot1**2+d2*l1*m2*t4*t16*t17*thdot2*(thdot1*2.0+thdot2)

    return Tc

def dynamixel_position_callback(data):
    global present_dynamixel_position
    pos1 = data.data[0] - position_offset[0]
    pos2 = data.data[0] - position_offset[1]
    # if (pos1 > 32768):
    #     pos1 = ~pos1
    #     pos1 = pos1 + 1

    # if (pos2 > 32768):
    #     pos2 = ~pos2
    #     pos2 = pos2 + 1

    present_dynamixel_position[0] = (np.pi/180)*position_scaling_factor*pos1
    present_dynamixel_position[1] = (np.pi/180)*position_scaling_factor*pos2

def dynamixel_velocity_callback(data):
    global present_dynamixel_velocity

    present_dynamixel_velocity[0] = (np.pi/30)*velocity_scaling_factor*data.data[0]
    present_dynamixel_velocity[1] = (np.pi/30)*velocity_scaling_factor*data.data[1]

def robot_position_callback(data):
    global present_robot_position

    present_robot_position = (np.pi/180)*data.data

def main():
    rospy.init_node('motion_controller',anonymous=True)

    rospy.Subscriber("/dxl_data/present_position_array", Int32MultiArray, dynamixel_position_callback)
    rospy.Subscriber("/dxl_data/present_velocity_array", Int32MultiArray, dynamixel_velocity_callback)
    rospy.Subscriber("/Angle", Float64, robot_position_callback)

    pub0 = rospy.Publisher('/program_state', Int32, queue_size=1)
    pub1 = rospy.Publisher('/dxl_target/Target_position', Int32MultiArray, queue_size=1)
    pub2 = rospy.Publisher('/dxl_target/Target_current', Int32, queue_size=1)

    sleep_rate = 100 # 100Hz
    rate = rospy.Rate(sleep_rate) 

    dynamixel_goal = np.array([35,0])
    publish_target_position_array = Int32MultiArray(data = dynamixel_goal)

    previous_robot_position = 0.

    state = input()

    print("RUNNING")
    pub0.publish(state)

    while not rospy.is_shutdown():
                    
        robot_velocity = (previous_robot_position-present_robot_position)*sleep_rate
        #print(robot_velocity)

        desired_position = alpha*np.arctan(robot_velocity)
        
        publish_target_current = int( (control_signal_definition(present_robot_position,-1*present_dynamixel_position[1],desired_position,robot_velocity,-1*present_dynamixel_velocity[1])/torque_constant)*10/(current_scaling_factor) )

        pub1.publish(publish_target_position_array)
        pub2.publish(publish_target_current)

        previous_robot_position = present_robot_position
    
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException: pass
