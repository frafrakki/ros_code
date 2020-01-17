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
#define SIM_TIME    30
#define LOOP_RATE   100
// prototype of callback function(s)
void dxl_Position_callback(const std_msgs::Int32MultiArray &msg);
void dxl_Velocity_callback(const std_msgs::Int32MultiArray &msg);
void dxl_Current_callback(const std_msgs::Int32MultiArray &msg);
void IMU_Euler_callback(const xsens_msgs::orientationEstimate &msg);
void Encoder_Angle_callback(const std_msgs::Float64 &msg);
void Program_State_callback(const std_msgs::Int32 &msg);
// setup global Var
float dxl_position_data[2];
float dxl_velocity_data[2];
float dxl_current_data[2];
float IMU_euler_data[3];
float encoder_position_data;
int program_state = 0;

int main(int argc, char **argv){
    // setup as ROS node
    ros::init(argc,argv, "data_logger");
    ros::NodeHandle nh;
    // setup Subscriber
    ros::Subscriber dxl_position  = nh.subscribe("dxl_data/present_position_array", 10, dxl_Position_callback);
    ros::Subscriber dxl_velocity  = nh.subscribe("dxl_data/present_velocity_array", 10, dxl_Velocity_callback);
    ros::Subscriber dxl_current  = nh.subscribe("dxl_data/present_current_array", 10, dxl_Current_callback);
    ros::Subscriber IMU_euler  = nh.subscribe("mti/filter/orientation", 10, IMU_Euler_callback);
    ros::Subscriber Encoder_position  = nh.subscribe("Angle", 10, Encoder_Angle_callback);
    ros::Subscriber program_state_sub  = nh.subscribe("program_state", 10, Program_State_callback);
    //setup loop rate
    ros::Rate rate=LOOP_RATE;

    // setup file export
    // declare file name(s)
    std::string action_data_file="/home/irl/ROS_data/ROBOT_ACTION_DATA.csv";
    // setup writing method
    std::ofstream write_data;
    // open file(s)
    write_data.open("/home/irl/ROS_data/ROBOT_ACTION_DATA.csv", std::ios::trunc);
    // set label
    write_data  << "DATA" <<","
                << "encoder" <<","
                << "dxl_pos_1" <<","
                << "dxl_pos_2" <<","
                << "dxl_vel_1" <<","
                << "dxl_vel_2" <<","
                << "dxl_current_1" <<","
                << "dxl_current_2" <<","
                << "IMU_roll" <<","
                << "IMU_pitch" <<","
                << "IMU_yaw" << std::endl;
    // set private var(s)
    int data_count = 0;
    // loop
    while(ros::ok()){
        ros::spinOnce(); // recieve data from ROS queue
        if(program_state == 1){
            write_data  << data_count <<","
                        << encoder_position_data <<","
                        << dxl_position_data[0] <<","
                        << dxl_position_data[1] <<","
                        << dxl_velocity_data[0] <<","
                        << dxl_velocity_data[1] <<","
                        << dxl_current_data[0] <<","
                        << dxl_current_data[1] <<","
                        << IMU_euler_data[0] <<","
                        << IMU_euler_data[1] <<","
                        << IMU_euler_data[2] << std::endl;
            data_count ++;
        }
        rate.sleep();
    }
    write_data.close();

}

// private functions
void dxl_Position_callback(const std_msgs::Int32MultiArray &msg){
    dxl_position_data[0] = msg.data[0];
    dxl_position_data[1] = msg.data[1];

    // ROS_INFO("DXL POS 1,2 :%d, %d",msg.data[0],msg.data[1]);
}

void dxl_Velocity_callback(const std_msgs::Int32MultiArray &msg){
    dxl_velocity_data[0] = msg.data[0];
    dxl_velocity_data[1] = msg.data[1];

    // ROS_INFO("DXL VEL 1,2 :%d, %d",msg.data[0],msg.data[1]);
}

void dxl_Current_callback(const std_msgs::Int32MultiArray &msg){
    dxl_current_data[0] = msg.data[0];
    dxl_current_data[1] = msg.data[1];

    // ROS_INFO("DXL CRNT 1,2 :%d, %d",msg.data[0],msg.data[1]);
}

void IMU_Euler_callback(const xsens_msgs::orientationEstimate &msg){
    IMU_euler_data[0] = msg.roll;
    IMU_euler_data[1] = msg.pitch;
    IMU_euler_data[2] = msg.yaw;

    // ROS_INFO("IMU ELR R,P,Y :%lf, %lf, %lf",msg.roll,msg.pitch,msg.yaw);
}

void Encoder_Angle_callback(const std_msgs::Float64 &msg){
    encoder_position_data = msg.data;

    ROS_INFO("ENC POS :%f",msg.data);
}

void Program_State_callback(const std_msgs::Int32 &msg){
    program_state = msg.data;
}