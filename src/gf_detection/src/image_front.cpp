#include <iostream>
#include <string>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <gf_perception/Object.h>
#include <gf_perception/ObjectList.h>
using namespace std;
using namespace cv;

void cb_front_object(){

}


int main (int argc, char** argv){
    ros::init(argc, argv, "img_front");
    ros::NodeHandle nh;
    ros::Subcriber sub_magnetic = nh.subscribe("gaofen_detection/number", 1, &AirsimControl::cb_magnetic, this);
    
}