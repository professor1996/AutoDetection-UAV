#pragma once
#include <iostream>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;


class NumberDetect{
    public:
    ros::NodeHandle* pnh;
    ros::Publisher cloud_pub; 
    Mat cam_matrix,cam_distortion;

    NumberDetect(ros::NodeHandle* _pnh);
    void detect(Mat&,Mat&);
    void colorExteact(Mat&,Mat&);
    double getPointDistance(Point& pt1,Point& pt2);
    float getDepthValue(Mat& img,Point& pt );
    void reprojectDepthImage(const Point& pt,float depth,Point3f& position);
    void setCamera(Mat& _matrix,Mat& _dist);
    void calcRotationTranslation(vector<Point3f>& point,float& yaw,Mat& tran);
};


