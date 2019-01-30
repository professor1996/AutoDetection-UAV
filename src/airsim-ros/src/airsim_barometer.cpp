
#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Vector3Stamped.h>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
using namespace std;
msr::airlib::MultirotorRpcLibClient *control_client;

void getBarometerData(geometry_msgs::Vector3Stamped& barometer_data)
{
    msr::airlib::BarometerData airsim_barometer_data;
    airsim_barometer_data = control_client->getBarometerdata();
    barometer_data.header.frame_id = "base_link";
    barometer_data.header.stamp = ros::Time::now();
    barometer_data.vector.x = airsim_barometer_data.altitude; // altitude meter
    barometer_data.vector.y = airsim_barometer_data.pressure; // pressure
                                                              // Pascal
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "airsim_image_front_depth");
    ros::NodeHandle nh;
    string ip("192.168.1.100");
    control_client = new msr::airlib::MultirotorRpcLibClient(ip);
    control_client->confirmConnection();
    ros::Publisher pub_barometer =
        nh.advertise<geometry_msgs::Vector3Stamped>("airsim/barometer", 1);
                    ros::Rate rate(32);
    while (ros::ok())
    {
        geometry_msgs::Vector3Stamped barometer_data;
        getBarometerData(barometer_data);
        pub_barometer.publish(barometer_data);
        ros::spinOnce();
        rate.sleep();
    }
}