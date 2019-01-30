#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "num_detect.hpp"
using namespace std;
using namespace cv;

Mat img_rgb, img_depth;
void cb_front_rgb(const sensor_msgs::ImageConstPtr &ptr_msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr =
            cv_bridge::toCvCopy(ptr_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    img_rgb = cv_ptr->image.clone();
    // cur_seq = msg->header.seq;
    char filename[100];
    if (ptr_msg->header.seq % 3 == 0)
    {
        sprintf(filename, "/root/GaoFen_Drone/resource/img/f1-%d.png", ptr_msg->header.seq);
        // cv::imwrite(filename,img_rgb);
    }
}

void cb_front_depth(const sensor_msgs::ImageConstPtr &ptr_msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(ptr_msg,
                                     sensor_msgs::image_encodings::BGR8); //TYPE_32FC1
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    img_depth = cv_ptr->image.clone();
    // cur_seq = msg->header.seq;
    char filename[100];
    if (ptr_msg->header.seq % 3 == 0)
    {
        sprintf(filename, "/root/GaoFen_Drone/resource/img_test/d1-%d.png", ptr_msg->header.seq);
        // cv::imwrite(filename,img_depth);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "img_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_front_rgb =
        it.subscribe("airsim/image/front/rgb", 1, &cb_front_rgb);
    image_transport::Subscriber sub_front_depth =
        it.subscribe("airsim/image/down/rgb", 1, &cb_front_depth);

    ros::Rate rate(80);
    ros::Publisher cloud_pub =
        nh.advertise<sensor_msgs::PointCloud>("cloud", 10);

    NumberDetect num_detect(&nh);

    double camD[9] = {320, 0, 320, 0, 320, 240, 0, 0, 1};
    double distCoeffD[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    Mat camera_matrix = Mat(3, 3, CV_64FC1, camD);
    Mat distortion_coefficients = Mat(5, 1, CV_64FC1, distCoeffD);
    num_detect.setCamera(camera_matrix, distortion_coefficients);
    int FN = 0;
    while (ros::ok())
    {
        if (img_rgb.empty() || img_depth.empty())
        {
            rate.sleep();
            ros::spinOnce();
            continue;
        }
        cv::imshow("rgb", img_rgb);
        cv::imshow("down", img_depth);
        char c = cv::waitKey(1);

        if (c == 'q')
        {
            char filename[200];
            sprintf(filename, "/home/ubuntu/GaoFen_Drone/resource/img2/9-%d.png", FN++);
            cv::imwrite(filename, img_rgb);
        }
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}