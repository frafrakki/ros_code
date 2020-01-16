#!/usr/bin/env python
##motion_logger.py

import rospy
import roslib

# Dynamixel connection
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import MultiArrayLayout

# IMU message
from xsens_msgs.msg import orientationEstimate

# Debug message
from std_msgs.msg import Int32,String,Float64

import numpy as np
import sys,os

# global variables for IMU/Dynamixel data
imu_euler_temp = np.array([0.0,0.0,0.0,0.0])
dynamixel_position_temp = np.array([0.0,0.0,0.0])
dynamixel_velocity_temp = np.array([0.0,0.0,0.0])
dynamixel_current_temp = np.array([0.0,0.0,0.0])
body_angle_temp = np.array([0.0,0.0])

program_state = 0

# constant values
gear_ratio = 18/40
position_scaling_factor = 360/4096 # deg/Dynamixel VALUE
velocity_scaling_factor = 0.299 # rpm/Dynamixel VALUE
current_scaling_factor =  2.69 # mA/Dynamixel VALUE

#position_offset = np.array([140,-4080])
position_offset = np.array([140,-20])

def callback_euler(data):
    global imu_euler_temp

    imu_euler_temp[1] = data.roll
    imu_euler_temp[2] = data.pitch
    imu_euler_temp[3] = data.yaw

    #rospy.loginfo("IMU R,P,Y: %f,%f,%f", imu_euler_temp[1],imu_euler_temp[2],imu_euler_temp[3])

def callback_dynamixel_position(data):
    global dynamixel_position_temp

    dynamixel_position_temp[1] = -1*(data.data[0]-position_offset[0])*position_scaling_factor
    dynamixel_position_temp[2] = -1*(data.data[1]-position_offset[1])*position_scaling_factor

    #rospy.loginfo("DYNA 1,2: %f,%f", dynamixel_position_temp[1],dynamixel_position_temp[2])

def callback_dynamixel_velocity(data):
    global dynamixel_velocity_temp

    dynamixel_velocity_temp[1] = -1*data.data[0]*velocity_scaling_factor
    dynamixel_velocity_temp[2] = -1*data.data[1]*velocity_scaling_factor

    #rospy.loginfo("DYNA 1,2: %f,%f", dynamixel_velocity_temp[1],dynamixel_velocity_temp[2])

def callback_dynamixel_current(data):
    global dynamixel_current_temp

    dynamixel_current_temp[1] = -1*data.data[0]*current_scaling_factor
    dynamixel_current_temp[2] = -1*data.data[1]*current_scaling_factor

    #rospy.loginfo("DYNA 1,2: %f,%f", dynamixel_current_temp[1],dynamixel_current_temp[2])

def callback_body_angle(data):
    global body_angle_temp

    body_angle_temp[1] = data.data

    rospy.loginfo("ENC: %f", body_angle_temp[1])

def callback_program_state(data):
    global program_state

    program_state = data.data

def main():
    rospy.init_node('motion_logger',anonymous=True)
    Rate = rospy.Rate(10)

    loop_count = 0

    imu_euler = np.array([np.array([0.0,0.0,0.0,0.0])])
    dynamixel_position = np.array([np.array([0.0,0.0,0.0])])
    dynamixel_velocity = np.array([np.array([0.0,0.0,0.0])])
    dynamixel_current = np.array([np.array([0.0,0.0,0.0])])
    body_angle = np.array([np.array([0.0,0.0])])

    while not rospy.is_shutdown():
        rospy.Subscriber("/program_state", Int32, callback_program_state)
        rospy.Subscriber("mti/filter/orientation", orientationEstimate, callback_euler)
        rospy.Subscriber("dxl_data/present_position_array", Int32MultiArray, callback_dynamixel_position)
        rospy.Subscriber("dxl_data/present_velocity_array", Int32MultiArray, callback_dynamixel_velocity)
        rospy.Subscriber("dxl_data/present_current_array", Int32MultiArray, callback_dynamixel_current)
        rospy.Subscriber("Angle", Float64, callback_body_angle)
        if(program_state == 1):
            loop_count=loop_count+1

            # create log data array
            imu_euler_temp[0] = loop_count
            body_angle_temp[0] = loop_count
            dynamixel_position_temp[0] = loop_count
            dynamixel_velocity_temp[0] = loop_count
            dynamixel_current_temp[0] = loop_count

            imu_euler = np.concatenate([imu_euler,[imu_euler_temp]],axis=0)
            body_angle = np.concatenate([body_angle,[body_angle_temp]],axis=0)
            dynamixel_position = np.concatenate([dynamixel_position,[dynamixel_position_temp]],axis=0)
            dynamixel_velocity = np.concatenate([dynamixel_velocity,[dynamixel_velocity_temp]],axis=0)
            dynamixel_current = np.concatenate([dynamixel_current,[dynamixel_current_temp]],axis=0)

        Rate.sleep()
    
    # save data
    np.savetxt("/home/irl/ROS_data/IMU_EULER_DATA.csv",X=imu_euler,delimiter=',')
    np.savetxt("/home/irl/ROS_data/DYNAMIXEL_POSITION_DATA.csv",X=dynamixel_position,delimiter=',')
    np.savetxt("/home/irl/ROS_data/DYNAMIXEL_VELOCITY_DATA.csv",X=dynamixel_velocity,delimiter=',')
    np.savetxt("/home/irl/ROS_data/DYNAMIXEL_CURRENT_DATA.csv",X=dynamixel_current,delimiter=',')
    np.savetxt("/home/irl/ROS_data/ENCODER_ANGLE_DATA.csv",X=body_angle,delimiter=',')
    print("SHUTDOWN")

if __name__ == "__main__":
    try:
        main()

    except rospy.ROSInterruptException:

        pass