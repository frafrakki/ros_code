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
#define SHOULDER_OFFSET     -63
#define WAIST_OFFSET        -21
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
    ros::Publisher pub0 = nh.advertise<std_msgs::Int32>("program_state",1);
    ros::Publisher pub1 = nh.advertise<std_msgs::Int32MultiArray>("dxl_target/Target_position",1);
    // setup Subscriber
    ros::Subscriber encoder_data  = nh.subscribe("Angle", 1, encoder_callback);
    //setup loop rate
    ros::Rate rate=LOOP_RATE;

    // ROS messages
    std_msgs::Int32 program_state;
    std_msgs::Int32MultiArray dynamixel_goal;
    dynamixel_goal.data.resize(2);

    // message datum
    program_state.data = 0;

    int goal_position[2] = {808,790};
    // int goal_position[2] = {600,790};
    dynamixel_goal.data[0] = -goal_position[0]-SHOULDER_OFFSET;
    dynamixel_goal.data[1] = -goal_position[1]-WAIST_OFFSET;

    ROS_INFO("Waiting for input");

    std::cin >> program_state.data;

    while(ros::ok()){
        ros::spinOnce();

        pub0.publish(program_state);

        if(encoder_angle >= START_ANGLE){
            pub1.publish(dynamixel_goal);
            ROS_INFO("TGT 1,2 :%d,%d",dynamixel_goal.data[0],dynamixel_goal.data[1]);
        }

        rate.sleep();
    }
}

void encoder_callback(const std_msgs::Float64 data){
    encoder_angle = data.data;
    // ROS_INFO("ENC :%f",encoder_angle);
}