// c++ lib
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
using namespace std;

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

// prototype of callback/private function(s)
void encoder_callback(const std_msgs::Float64 data);
vector<string> split(string& input, char delimiter);

// global variable(s)
float encoder_angle = 0.0;

int main(int argc, char **argv){
    // setup as ROS node
    ros::init(argc,argv, "trilink_swing");
    ros::NodeHandle nh;
    // setup Publisher
    ros::Publisher pub0 = nh.advertise<std_msgs::Int32>("program_state",1);
    ros::Publisher pub1 = nh.advertise<std_msgs::Int32MultiArray>("dxl_target/Target_current",1);
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
    /*
    int goal_position[2] = {SHOULDER_FLIP_GOAL,WAIST_FLIP_GOAL};

    dynamixel_goal.data[0] = -goal_position[0]+SHOULDER_OFFSET;
    dynamixel_goal.data[1] = -goal_position[1]+WAIST_OFFSET_FLIP;
    */
    //counter
    int i=0;
    int j=0;

    //dynamixel variables
    float gear_ratio = 18.0/40.0;
    float position_scaling_factor = 360.0/4095.0; // deg/Dynamixel VALUE
    float velocity_scaling_factor = 0.299; // rpm/Dynamixel VALUE
    float current_scaling_factor =  2.69; // mA/Dynamixel VALUE
    float torque_constant_shoulder = 0.79; // Nm/A
    float torque_constant_hip = 0.79; // Nm/A

    // read CSV data
    ifstream ifs(CSV_NAME); //set motion data source

    string line;
    vector<vector<string> > strvec;

    while (getline(ifs, line)) {
        
        strvec.push_back(split(line, ','));
        
        for (i=0; i<strvec.size();i++){
            for(j=0; j<strvec.at(i).size();j++){
                printf("%5d", stoi(strvec.at(i).at(j)));
            }
            printf("\n");
        }
        printf("---\n");
        i=0;
    }

    ROS_INFO("Waiting for input");

    do{
        cin >> program_state.data;
    }
    while(program_state.data != 1);

    while(ros::ok() && i<strvec.size()){
        ros::spinOnce();

        pub0.publish(program_state);
        dynamixel_goal.data[0] = (int)(stof(strvec.at(i).at(0))/gear_ratio/torque_constant_shoulder)*1000/current_scaling_factor;
        dynamixel_goal.data[1] = (int)(stof(strvec.at(i).at(1))/torque_constant_hip)*1000/current_scaling_factor;
        pub1.publish(dynamixel_goal);
        ROS_INFO("TGT 1,2 :%d,%d",dynamixel_goal.data[0],dynamixel_goal.data[1]);
        i++;

        rate.sleep();
    }
}

void encoder_callback(const std_msgs::Float64 data){
    encoder_angle = data.data;
    // ROS_INFO("ENC :%f",encoder_angle);
}

vector<string> split(string& input, char delimiter){
    istringstream stream(input);
    string field;
    vector<string> result;
    while (getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}