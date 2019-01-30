
#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
using namespace std;
msr::airlib::MultirotorRpcLibClient *control_client;

void getImuData(sensor_msgs::Imu& imu_data)
{
msr::airlib::ImuData airsim_img_data;
    airsim_img_data = control_client->getImudata();
    imu_data.header.frame_id = "base_link";
    imu_data.header.stamp = ros::Time::now();
    imu_data.angular_velocity.x = airsim_img_data.angular_velocity(0);
    imu_data.angular_velocity.y = airsim_img_data.angular_velocity(1);
    imu_data.angular_velocity.z = airsim_img_data.angular_velocity(2);
    imu_data.linear_acceleration.x = airsim_img_data.linear_acceleration(0);
    imu_data.linear_acceleration.y = airsim_img_data.linear_acceleration(1);
    imu_data.linear_acceleration.z = airsim_img_data.linear_acceleration(2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "airsim_imu");
    ros::NodeHandle nh;
    string ip("192.168.1.100");
    control_client = new msr::airlib::MultirotorRpcLibClient(ip);
    control_client->confirmConnection();
    ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("airsim/imu", 1);
    ros::Rate rate(35);
    while (ros::ok())
    {
        sensor_msgs::Imu imu_data;
        getImuData(imu_data);
        pub_imu.publish(imu_data);
        ros::spinOnce();
        rate.sleep();
    }
}