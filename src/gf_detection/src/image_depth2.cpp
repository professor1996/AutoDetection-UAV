#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Vector3Stamped.h>
#include <cstring>
#include <ros/package.h>

using namespace cv;
using namespace std;

void depth_processing(Mat &depthRaw);

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber depth_sub_;
  image_transport::Publisher image_pub_;

  ros::Publisher pub_depth_count,pub_depth_count_full;
  ros::Publisher pub_circle;
  Mat mask_down,mask_left,mask_right,mask_up;

public:
  ImageConverter()
      : it_(nh_)
  {

    depth_sub_ = it_.subscribe("/airsim/image/front/depth", 1,
                               &ImageConverter::imageDepth, this);
    pub_depth_count = nh_.advertise<geometry_msgs::Vector3Stamped>("/airsim/depth/count", 1);
    pub_depth_count_full = nh_.advertise<geometry_msgs::Vector3Stamped>("/airsim/depth/count_full", 1);
    
    pub_circle = nh_.advertise<geometry_msgs::Vector3Stamped>("/airsim/depth/circle", 1);
    std::string resource_dir=ros::package::getPath("gf_detection");
    char name[128];
    sprintf(name, "%s/resource/mask_down.png",resource_dir.c_str());
    mask_down=imread(name,0);
        sprintf(name, "%s/resource/mask_left.png",resource_dir.c_str());
    mask_left=imread(name,0);
        sprintf(name, "%s/resource/mask_right.png",resource_dir.c_str());
    mask_right=imread(name,0);
    threshold(mask_down,mask_up,100,255,CV_THRESH_BINARY_INV);
    // namedWindow("depth");
    // namedWindow("depth1");
  }

  ~ImageConverter()
  {
    // destroyWindow("depth");
    // destroyWindow("depth1");
  }

  void imageDepth(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    Mat image_DEPTH;

    try
    {
      //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      image_DEPTH = cv_ptr->image;
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    depth_processing(image_DEPTH);
    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
  }

  void depth_processing(Mat &depthRaw)
  {
    int nr = depthRaw.rows; // number of rows
    int nc = depthRaw.cols; // total number of elements per line
    if (depthRaw.empty())
    {
      ROS_INFO("NO image input");
      return;
    }
    int max = 10;
    int min = 0;

    double minVal;
    minMaxLoc(depthRaw,&minVal,NULL,NULL,NULL);
    Mat Gray = Mat(nr, nc, CV_8UC1);

    for (int i = 0; i < nr; i++)
    {
      uchar *data_gray = Gray.ptr<uchar>(i);
      float *data_src = depthRaw.ptr<float>(i);

      for (int j = 0; j < nc; j++)
      {
        if (data_src[j] < max && data_src[j] > min) //max,min表示需要的深度值的最大/小值
        {
          data_gray[j] = 255 - (data_src[j] - min) / (max - min) * 255.0f;
        }
        else
        {
          data_gray[j] = 0;
        }
      }
    }


    // Mat thres;
    // threshold(Gray, thres, 0, 255, CV_THRESH_OTSU);

    Mat edge;
    Canny(Gray, edge, 100, 300, 3);
    // imshow("gg", edge);

    // imshow("depth", edge);
    waitKey(5);

    //寻找最外层轮廓
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(edge, contours, hierarchy, CV_RETR_LIST, CHAIN_APPROX_NONE, Point());

    float p0x, p0y, p1x, p1y, p2x, p2y;
    float l1, l2, l12;
    float cx, cy;
    float l1t;
    Point2f Pt[4];
    int n_circle = 0;

    // cout<<contours.size()<<endl;

    for (int i = 0; i < contours.size(); i++)
    {
      RotatedRect rect = minAreaRect(contours[i]);
      Point2f P[4];
      rect.points(P);
      p0x = P[0].x;
      p0y = P[0].y;
      p1x = P[1].x;
      p1y = P[1].y;
      p2x = P[2].x;
      p2y = P[2].y;
      l1 = sqrt((p0x - p1x) * (p0x - p1x) + (p0y - p1y) * (p0y - p1y));
      l2 = sqrt((p2x - p1x) * (p2x - p1x) + (p2y - p1y) * (p2y - p1y));
      l12 = l1 / l2;

      // for (int j = 0; j <= 3; j++)
      // {
      //   line(Gray, P[j], P[(j + 1) % 4], Scalar(255), 8);
      // }

      if (fabs(l12 - 1) < 0.2 && l1 > 100 && l2 > 100)
      {
        if (n_circle == 0)
        {
          n_circle = 1;
          cx = (p0x + p2x) / 2;
          cy = (p0y + p2y) / 2;
          l1t = l1;
          memcpy(Pt, P, sizeof(P));
        }
        else if (n_circle == 1)
        {
          if (l1 < l1t)
          {
            cx = (p0x + p2x) / 2;
            cy = (p0y + p2y) / 2;
            memcpy(Pt, P, sizeof(P));
          }
        }
      }
    }
    // imshow("ggg", Gray);
    geometry_msgs::Vector3Stamped circle_msg;
    if (n_circle > 0)
    {
      
      circle_msg.header.stamp = ros::Time::now();
      circle_msg.vector.x = cx;
      circle_msg.vector.y = cy;
      circle_msg.vector.z=sqrt(pow(Pt[0].x - Pt[2].x,2) +pow(Pt[0].y - Pt[2].y,2));
      
      circle(Gray, Point2f(cx, cy), 2, Scalar(0));

      for (int j = 0; j <= 3; j++)
      {
        line(Gray, Pt[j], Pt[(j + 1) % 4], Scalar(0), 2);
      }

      cout << cx << ',' << cy << endl;
    }
    pub_circle.publish(circle_msg);

    Mat left,right,down,up;
    int len=100;
    // imshow("aa",mask_left);
    bitwise_and(Gray,mask_left,left);
    bitwise_and(Gray,mask_right,right);
    bitwise_and(Gray,mask_down,down);
    bitwise_and(Gray,mask_up,up);
  
  
    geometry_msgs::Vector3Stamped count_msg,count_full_msg;
    count_msg.header.stamp = ros::Time::now();
    count_full_msg.header.stamp = ros::Time::now();
    Mat gray_th;
    threshold(Gray,gray_th,100,255,CV_THRESH_BINARY);
    count_msg.vector.x= countNonZero(left)*1.0/(countNonZero(mask_left));
    count_msg.vector.y= countNonZero(right)*1.0/(countNonZero(mask_right));
    count_msg.vector.z= countNonZero(down)*1.0/(countNonZero(mask_down));
    count_full_msg.vector.x= countNonZero(up)*1.0/(countNonZero(mask_up));
    count_full_msg.vector.y=minVal;
    pub_depth_count.publish(count_msg);
    pub_depth_count_full.publish(count_full_msg);
    // imshow("depth1", up);
    waitKey(5);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_depth2");
  ImageConverter ic;
  ros::spin();
  return 0;
}
