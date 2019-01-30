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
void cb_front_rgb(const sensor_msgs::ImageConstPtr& ptr_msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr =
            cv_bridge::toCvCopy(ptr_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    img_rgb = cv_ptr->image.clone();
    // cur_seq = msg->header.seq;

}

void cb_front_depth(const sensor_msgs::ImageConstPtr& ptr_msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(ptr_msg,
                                     sensor_msgs::image_encodings::TYPE_32FC1);//TYPE_32FC1
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    img_depth = cv_ptr->image.clone();
    // cur_seq = msg->header.seq;
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "img_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_front_rgb =
        it.subscribe("airsim/image/front/rgb", 1, &cb_front_rgb);
    image_transport::Subscriber sub_front_depth =
        it.subscribe("airsim/image/front/depth", 1, &cb_front_depth); 

    ros::Rate rate(80);
    ros::Publisher cloud_pub =
        nh.advertise<sensor_msgs::PointCloud>("cloud", 10);

    NumberDetect num_detect(&nh);

    double camD[9] = {268.3556,0,320.6049,0,268.2210,240.4329,0,0,1};
    double distCoeffD[5] = {0.0013,0.0,0.0,0.0,0.0};
    Mat camera_matrix = Mat(3, 3, CV_64FC1, camD);
    Mat distortion_coefficients = Mat(5, 1, CV_64FC1, distCoeffD);
    num_detect.setCamera(camera_matrix,distortion_coefficients);
int FN=21;
    while (ros::ok()) {
        if (img_rgb.empty() || img_depth.empty()) {
            rate.sleep();
            ros::spinOnce();
            continue;
        }
    
        ros::Time start = ros::Time::now();
        num_detect.detect(img_rgb, img_depth);
        ros::Time end = ros::Time::now();
        ROS_INFO("Time : %f", (end - start).toSec());

        sensor_msgs::PointCloud cloud;
        cloud.header.stamp = ros::Time::now();
        cloud.header.frame_id = "map";

        cloud.points.resize(img_depth.cols * img_depth.rows);

        // generate some fake data for our point cloud
        Mat ii(img_depth);

        for (int i = 0; i < img_depth.rows; i++) {
            for (int j = 0; j < img_depth.cols; j++) {
                float u = j - 320;
                float v = i - 240;
                float d = img_depth.at<float>(i, j);
                double k = sqrt(1 + pow((u / 320), 2) + pow((v / 320), 2));

                cloud.points[i * 640 + j].y = d / k;
                cloud.points[i * 640 + j].x =
                    1.0 * cloud.points[i * 640 + j].y * u / 320;
                cloud.points[i * 640 + j].z =
                    -1.0 * cloud.points[i * 640 + j].y * v / 320;
            }
        }

        cloud_pub.publish(cloud);

        // imshow("a", img_rgb);
        // waitKey(1);
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}