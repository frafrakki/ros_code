#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

int main(int argc,char **argv){
    ros::init(argc, argv, "torque_controller");
    ros::NodeHandle nh;
    ros::Publisher tc_pub = nh.advertise<std_msgs::Float64>("torque_order", 10);
    ros::Rate loop_rate(10);

    while(ros::ok()){
        std_msgs::Float64 tc;

        tc.data = torque_calc_callback();
    }
}

