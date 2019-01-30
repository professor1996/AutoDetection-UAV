#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Polygon.h>

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "PID.h"
#include <iostream>
#include <string>
#include <gf_perception/Object.h>
#include <gf_perception/ObjectList.h>

class AirsimControl{
    public:
    ros::NodeHandle nh;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_gps;
    ros::Subscriber sub_magnetic;
    ros::Subscriber sub_baromrter;
    ros::Subscriber sub_object_front;
    ros::Subscriber sub_object_down;
    ros::Subscriber sub_object_aruco_down;
    ros::Subscriber sub_object_aruco_front;
    ros::Subscriber sub_object_circle;
    ros::Subscriber sub_depth_count,sub_depth_count_full;
    ros::Subscriber sub_line;

    ros::Publisher pub_front_camera_pose;
    ros::Publisher pub_height_filter;

    void cb_imu(const sensor_msgs::Imu& msg);
    void cb_gps(const sensor_msgs::NavSatFix& msg);
    void cb_magnetic(const sensor_msgs::MagneticField& msg);
    void cb_barometer(const geometry_msgs::Vector3Stamped& msg);
    void cb_object_front(const gf_perception::ObjectList& msg);
    void cb_object_down(const gf_perception::ObjectList& msg);
    void cb_object_aruco_down(const gf_perception::ObjectList& msg);
    void cb_object_aruco_front(const gf_perception::ObjectList& msg);
    void cb_object_circle(const geometry_msgs::Vector3Stamped& msg);
    void cb_depth_count(const geometry_msgs::Vector3Stamped& msg);
    void cb_depth_count_full(const geometry_msgs::Vector3Stamped& msg);
    void cb_line(const geometry_msgs::Polygon& msg);

    

    
    sensor_msgs::Imu msg_imu;
    sensor_msgs::NavSatFix msg_gps;
    sensor_msgs::MagneticField msg_magnetic;
    geometry_msgs::Vector3Stamped msg_barometer;
    geometry_msgs::Vector3Stamped msg_circle;
    geometry_msgs::Vector3Stamped msg_depth_count,msg_depth_count_full;
    geometry_msgs::Vector3Stamped msg_pre_depth_count,msg_pre_depth_count_full;
    gf_perception::ObjectList msg_objects_front;
    gf_perception::ObjectList msg_objects_down;
    gf_perception::ObjectList msg_objects_aruco_down;
    gf_perception::ObjectList msg_objects_aruco_front;
    gf_perception::Object object_front;
    gf_perception::Object object_down;
    geometry_msgs::Polygon msg_line;
    int error_code,pre_error_code;
    std::vector<int> num_queue;
    
    void run();
    int target_mode_count[12][2];
    int target_mode[12]; //0-未知,1-板子.2-停机坪
    int detect_num;       //1-10 ,11-二维码

    AirsimControl();
    ~AirsimControl(){;};
    
    msr::airlib::MultirotorRpcLibClient* control_client;
    bool takeoff(PIDctrl* pid);
    float initial_height;
    bool land();
    bool move(float pitch,float roll,float throttle,float yaw,float duration);
    void gostraight();

    int running_count,leftright_count,forward_count;
    int search_flag;
    float initial_yaw;
    int aruco_lost_flag;

    void search(double& pitch,double& roll,double& yaw);
    bool go_forward(double& pitch,double count);
    double current_yaw;
};