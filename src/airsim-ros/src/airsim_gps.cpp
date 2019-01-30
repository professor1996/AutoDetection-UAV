
#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/NavSatFix.h>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
using namespace std;
msr::airlib::MultirotorRpcLibClient *control_client;

void getGPSData(sensor_msgs::NavSatFix& gps_data)
{
    msr::airlib::GpsData airsim_aps_data;
    airsim_aps_data = control_client->getGpsLocation();
    gps_data.header.frame_id = "base_link";
    gps_data.header.stamp = ros::Time::now();
    gps_data.latitude = airsim_aps_data.latitude;
    gps_data.longitude = airsim_aps_data.longitude;
    gps_data.altitude = airsim_aps_data.altitude;


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "airsim_gps");
    ros::NodeHandle nh;
    string ip("192.168.1.100");
    control_client = new msr::airlib::MultirotorRpcLibClient(ip);
    control_client->confirmConnection();
    ros::Publisher pub_gps = nh.advertise<sensor_msgs::NavSatFix>("airsim/gps", 1);
    while (ros::ok())
    {
        sensor_msgs::NavSatFix gps_data;
        getGPSData(gps_data);
        pub_gps.publish(gps_data);
        ros::spinOnce();
    }
}