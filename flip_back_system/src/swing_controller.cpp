// c++ lib
#include <iostream>
#include <string>
#include <fstream>
#include <math.h>
#include <bits/stdc++.h>
// ros lib
#include <ros/ros.h>
// msg lib
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <xsens_msgs/orientationEstimate.h>
// private include header(s)
#include "global_definitions.h"
// prototype of private function(s)
double control_signal_definition(float th1,float th2,float th2d,float thdot1,float thdot2);
double t2,t3,t5,t9,t4,t6,t7,t8,t10,t11,t12,t13,t14,t15,t16,t17,Tc;

// prototype of ROS callback function(s)
void dynamixel_position_callback(const std_msgs::Int32MultiArray &msg);
void dynamixel_velocity_callback(const std_msgs::Int32MultiArray &msg);
void encoder_angle_callback(const std_msgs::Float64 &msg);

// constant dynamixel variables
float gear_ratio = 18.0/40.0;
float position_scaling_factor = 360.0/4095.0; // deg/Dynamixel VALUE
float velocity_scaling_factor = 0.299; // rpm/Dynamixel VALUE
float current_scaling_factor =  2.69; // mA/Dynamixel VALUE
float torque_constant = 0.79; // Nm/A

// mechanical variables
float l1 = 0.363;
float d1 = 0.240;
float m1 = 1.234;
float I1 = 0.080042027;

float l2 = 0.273;
float d2 = 0.115;
float m2 = 0.645;
float I2 = 0.014397165;

float g = 9.81;

// control variables
float kd = 200.0;
float kp = 2500.0;

float alpha = M_PI/6.0;

// data storage variable(s)
float dxl_position_data[2];
float dxl_velocity_data[2];
float encoder_position_data;

int main(int argc, char **argv){
    // setup as ROS node
    ros::init(argc,argv,"swing_controller");
    ros::NodeHandle nh;
    // setup Publisher
    ros::Publisher pub0 = nh.advertise<std_msgs::Int32>("program_state",1);
    ros::Publisher pub1 = nh.advertise<std_msgs::Int32MultiArray>("dxl_target/Target_position",1);
    ros::Publisher pub2 = nh.advertise<std_msgs::Int32>("dxl_target/Target_current",1);

    // setup Subscriber
    ros::Subscriber encoder_data = nh.subscribe("Angle",1,encoder_angle_callback);
    ros::Subscriber dynamixel_position = nh.subscribe("dxl_data/present_position_array",1,dynamixel_position_callback);
    ros::Subscriber dynamixel_velocity = nh.subscribe("dxl_data/present_velocity_array",1,dynamixel_velocity_callback);

    // ROS loop rate
    ros::Rate rate = LOOP_RATE;

    // ROS message(s)
    std_msgs::Int32 program_state;
    std_msgs::Int32MultiArray dynamixel_target_position;
    std_msgs::Int32 dynamixel_target_current;

    // initialize message(s)
    program_state.data = 0;

    dynamixel_target_position.data.resize(2);
    dynamixel_target_position.data[0] = SHOULDER_OFFSET;
    dynamixel_target_position.data[1] = 0;

    dynamixel_target_current.data=0;

    // private variable(s)
    float previous_robot_position = 0.0;
    float robot_velocity = 0.0;
    float desired_position = 0.0;

    ROS_INFO("Waiting for input : 1");
    std::cin >> program_state.data;

    pub0.publish(program_state);
    pub1.publish(dynamixel_target_position);

    // main loop
    while(ros::ok()){
        // get data from queue
        ros::spinOnce();
        robot_velocity = (encoder_position_data - previous_robot_position)*LOOP_RATE;

        desired_position = alpha*atan(robot_velocity);
        dynamixel_target_current.data = int( (control_signal_definition(encoder_position_data,-1*dxl_position_data[1],desired_position,robot_velocity,-1*dxl_velocity_data[1])/torque_constant)*10/(current_scaling_factor) );
        pub2.publish(dynamixel_target_current);

        previous_robot_position = encoder_position_data;

        std::cout << "current velocity :" << robot_velocity << std::endl;

        rate.sleep();
    } 
}

void dynamixel_position_callback(const std_msgs::Int32MultiArray &msg){
    dxl_position_data[0] = msg.data[0] - SHOULDER_OFFSET;
    dxl_position_data[1] = msg.data[1] - WAIST_OFFSET_SWING;

    dxl_position_data[0] =(M_PI/180.0)*position_scaling_factor*dxl_position_data[0];
    dxl_position_data[1] =(M_PI/180.0)*position_scaling_factor*dxl_position_data[1];
}

void dynamixel_velocity_callback(const std_msgs::Int32MultiArray &msg){
    dxl_velocity_data[1] = msg.data[1];

    dxl_velocity_data[1] = (M_PI/30.0)*velocity_scaling_factor*dxl_velocity_data[1];
}

void encoder_angle_callback(const std_msgs::Float64 &msg){
    encoder_position_data = (M_PI/180.0)*msg.data;
}

double control_signal_definition(float th1,float th2,float th2d,float thdot1,float thdot2){
    
    t2 = d2*d2;
    t3 = m2*t2;
    t5 = cos(th2);
    t9 = d2*l1*m2*t5;
    t4 = I2+t3+t9;
    t6 = cos(th1);
    t7 = th1+th2;
    t8 = cos(t7);
    t10 = d1*d1;
    t11 = m1*t10;
    t12 = l1*l1;
    t13 = m2*t12;
    t14 = d2*l1*m2*t5*2.0;
    t15 = I1+I2+t3+t11+t13+t14;
    t16 = 1.0/t15;
    t17 = sin(th2);
    Tc = -1*(kd*thdot2+kp*(th2-th2d))*(I2+t3-t4*t4*t16)-g*t4*t16*(d1*m1*t6+d2*m2*t8+l1*m2*t6)+d2*g*m2*t8+d2*l1*m2*t17*thdot1*thdot1+d2*l1*m2*t4*t16*t17*thdot2*(thdot1*2.0+thdot2);

    return Tc;
}