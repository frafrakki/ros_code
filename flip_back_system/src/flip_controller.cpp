// c++ lib
#include <iostream>
#include <string>
#include <fstream>

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
// definition
#define SIM_TIME            30
#define LOOP_RATE           100
#define SHOULDER_OFFSET     150
#define WAIST_OFFSET        -20
#define START_ANGLE         -179.5
// prototype of callback function(s)
void encoder_callback(const std_msgs::Float64 data);

// global variable(s)
float encoder_angle = 0.0;

int main(int argc, char **argv){
    // setup as ROS node
    ros::init(argc,argv, "flip_controller");
    ros::NodeHandle nh;
    // setup Publisher
    ros::Publisher pub0 = nh.advertise<std_msgs::Int32>("program_state",10);
    ros::Publisher pub1 = nh.advertise<std_msgs::Int32MultiArray>("dxl_target/Target_position_array",10);
    // setup Subscriber
    ros::Subscriber encoder_data  = nh.subscribe("Angle", 10, encoder_callback);
    //setup loop rate
    ros::Rate rate=LOOP_RATE;

    // ROS messages
    std_msgs::Int32 program_state;
    std_msgs::Int32MultiArray dynamixel_goal;
    // massage datum
    program_state.data = 0;

    dynamixel_goal.data.resize(2);
    dynamixel_goal.data[0] = -808-SHOULDER_OFFSET;
    dynamixel_goal.data[1] = -790-WAIST_OFFSET;

    ROS_INFO("Waiting for input");

    std::cin >> program_state.data;

    while(ros::ok()){
        ros::spinOnce();

        pub0.publish(program_state);

        if(encoder_angle >= START_ANGLE){
            pub1.publish(dynamixel_goal);
        }

        rate.sleep();
    }
}

void encoder_callback(const std_msgs::Float64 data){
    encoder_angle = data.data;
    ROS_INFO("ENC :%f",encoder_angle);
}