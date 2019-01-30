
#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/MagneticField.h>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
using namespace std;
msr::airlib::MultirotorRpcLibClient *control_client;

void getMagneticData(sensor_msgs::MagneticField& magnetic_data)
{
     msr::airlib::MagnetometerData airsim_magnetic_data;
    airsim_magnetic_data = control_client->getMagnetometerdata();
    magnetic_data.header.frame_id = "base_link";
    magnetic_data.header.stamp = ros::Time::now();
    magnetic_data.magnetic_field.x =
        airsim_magnetic_data.magnetic_field_body(0);
    magnetic_data.magnetic_field.y =
        airsim_magnetic_data.magnetic_field_body(1);
    magnetic_data.magnetic_field.z =
        airsim_magnetic_data.magnetic_field_body(2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "airsim_magnetic");
    ros::NodeHandle nh;
    string ip("192.168.1.100");
    control_client = new msr::airlib::MultirotorRpcLibClient(ip);
    control_client->confirmConnection();
    ros::Publisher  pub_magnetic =
        nh.advertise<sensor_msgs::MagneticField>("airsim/magnetic", 1);
            ros::Rate rate(32);
    while (ros::ok())
    {
           sensor_msgs::MagneticField magnetic_data;
        getMagneticData(magnetic_data);
        pub_magnetic.publish(magnetic_data);
        ros::spinOnce();
        rate.sleep();
    }
}