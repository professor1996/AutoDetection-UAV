#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "PID.h"
using namespace std;

sensor_msgs::Imu msg_imu;
geometry_msgs::Vector3Stamped msg_barometer;


void cb_imu(const sensor_msgs::Imu &msg)
{
    msg_imu = msg;
}

void cb_barometer(const geometry_msgs::Vector3Stamped& msg){
    msg_barometer = msg;
    cout<<msg_barometer.vector.x<<endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "airsim_control");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Subscriber sub_imu = nh.subscribe("airsim/imu/fusion", 1, cb_imu);
    ros::Subscriber sub_baromrter=nh.subscribe("airsim/barometer", 1, cb_barometer);
    ros::Rate rate(30);


    PIDctrl pid_height,pid_height_vel;
    pid_height.init(1.3, 0.0005, 150, 5); 
    // pid_height_vel.init(0.08, 0.0003, 1.5, 5); 
    float target_height=20;

    msr::airlib::MultirotorRpcLibClient* control_client = new msr::airlib::MultirotorRpcLibClient("192.168.1.100");
    control_client->confirmConnection();
    control_client->enableApiControl(true);
    control_client->armDisarm(true);

    while (ros::ok()){
        double d_height = target_height - msg_barometer.vector.x;
        float control_throttle = pid_height.calc(d_height);
        
        control_client->moveByAngleThrottle(0, 0, control_throttle, 0, 5); 
        ros::spinOnce();
        rate.sleep();
    }
}