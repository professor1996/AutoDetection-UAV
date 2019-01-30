#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <gf_perception/Object.h>
#include <gf_perception/ObjectList.h>
#include "std_msgs/Int16.h"
class ImageMatching
{
  public:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher object_pub;  
    ros::Subscriber campose_sub;

    ImageMatching();
    void imageCb(const sensor_msgs::ImageConstPtr &msg);
    void imageProcess(cv::Mat &,gf_perception::ObjectList &);
    void camposeCallback(const std_msgs::Int16::ConstPtr& msg);
    int campose;

  private:
    void digitSquares(IplImage *img, int minarea, int maxarea,gf_perception::ObjectList &rect_list);
    double angle(CvPoint *pt1, CvPoint *pt2, CvPoint *pt0);
    int lentowid(CvPoint *pt0, CvPoint *pt1, CvPoint *pt2);
    int judgecenter(double x1, double y1, double x2, double y2);
    int judgerect(CvPoint *pt0, CvPoint *pt1, CvPoint *pt2, CvPoint *pt3);
    double getx(CvPoint *pt);
    double gety(CvPoint *pt);
    int t_rect(IplImage *img, CvPoint *pt0, CvPoint *pt1, CvPoint *pt2, CvPoint *pt3);
    int getDigit(IplImage *img, IplImage *imgsrc, CvPoint *pt0, CvPoint *pt2);
};