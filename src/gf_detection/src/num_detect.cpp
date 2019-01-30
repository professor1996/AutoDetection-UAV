#include "num_detect.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <sensor_msgs/PointCloud.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
using namespace Eigen;
#define SHOW_IMAGE
#define CALC_TIME_START ros::Time start=ros::Time::now()
#define CALC_TIME_END ROS_INFO("TIME : %f FPS: %f",(ros::Time::now()-start).toSec(),1.0/(ros::Time::now()-start).toSec())

NumberDetect::NumberDetect(ros::NodeHandle *_pnh)
{
    int a = 0;
    pnh = _pnh;
    cloud_pub = pnh->advertise<sensor_msgs::PointCloud>("plane_cloud", 10);
}
void NumberDetect::detect(Mat &raw_img_rgb, Mat &raw_img_depth)
{
    // cout<<"detect"<<endl;
    
    Mat img_threshold, img_depth_8u, img_fusion;
    //深度图转换
    raw_img_depth.convertTo(img_depth_8u, CV_8U, 1);
    threshold(img_depth_8u, img_depth_8u, 200, 255, CV_THRESH_BINARY_INV);
    // RGB颜色提取
    colorExteact(raw_img_rgb, img_threshold);
    //求与
    bitwise_and(img_threshold, img_depth_8u, img_fusion);
  //  imshow("dep", img_threshold);
    //轮廓提取
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(img_fusion, contours, hierarchy, CV_RETR_EXTERNAL,
                 CV_CHAIN_APPROX_NONE, Point(0, 0));

    Mat img_draw = raw_img_rgb.clone();

    vector<vector<Point3f>> target_position;
    vector<vector<Point2i>>::const_iterator it = contours.begin();
    
    while (it != contours.end())
    {
        //轮廓筛选 
        if (it->size() < 60 || it->size() > 10000)
        {
            it++;
            continue;
        }
        //四边形拟合 approxPolyDP
        vector<Point> hull, poly;
        convexHull(Mat(*it), hull, false);
        approxPolyDP(Mat(hull), poly, it->size() / 10, true);
        //最小外接矩形
        Rect bouding_rect = boundingRect(Mat(poly));
        rectangle(img_draw, bouding_rect, Scalar(0, 0, 255));
        if (bouding_rect.width < 20 || bouding_rect.height < 20)
        {
            it++;
            continue;
        }
        //矩形所有点深度计算
        
        vector<Point3f> vec_position;
        for (int i = bouding_rect.x; i < bouding_rect.x + bouding_rect.width; i = i + bouding_rect.width / 15)
        {
            for (int j = bouding_rect.y; j < bouding_rect.y + bouding_rect.height; j = j + bouding_rect.height / 15)
            {
                float ptz = raw_img_depth.at<float>(j, i);
                if (ptz > 5)
                    continue;

                Point3f pt_position;
                //三维坐标点 jiesuan
                reprojectDepthImage(Point(i, j), ptz, pt_position);
                vec_position.push_back(pt_position);
            }
        }
        target_position.push_back(vec_position);
        
        //旋转矩阵计算
        float yaw;
        Mat tran;

        calcRotationTranslation(vec_position,yaw,tran);

        cout<<tran<<endl;
        it++;
    }

    // imshow("dd", img_draw);
    // imshow("ddd", img_fusion);
    waitKey(1);
    //对应位置深度图
    //深度解算
}
double NumberDetect::getPointDistance(Point &pt1, Point &pt2)
{
    return sqrt(pow((pt1.x - pt2.x), 2) + pow((pt1.y - pt2.y), 2));
}
float NumberDetect::getDepthValue(Mat &img, Point &pt)
{
    float ptz = img.at<float>(pt.x, pt.y);
    if (ptz == 255)
    {
        int sum = 0, count = 0;
        for (int m = 1; m < 5; m++)
        {
            for (int n = 1; n < 5; n++)
            {
                if (pt.x < m || pt.x > img.cols - m || pt.y < n ||
                    pt.y > img.rows - n)
                    continue;
                if (img.at<float>(pt.y + n, pt.x + m) != 255)
                {
                    sum += img.at<float>(pt.y + n, pt.x + m);
                    count++;
                }
                if (img.at<float>(pt.y - n, pt.x - m) != 255)
                {
                    sum += img.at<float>(pt.y - n, pt.x - m);
                    count++;
                }
            }
        }
        if (sum != 0)
            ptz = sum * 1.0 / count;
    }

    return ptz;
}
void NumberDetect::colorExteact(Mat &img, Mat &img_output)
{
    int img_size = img.cols * img.rows;
    const uchar *ptr_begin = img.data;
    const uchar *ptr_src = img.data;
    const uchar *ptr_src_end = img.data + img_size * 3;

    Mat img_gray(img.rows, img.cols, CV_8UC1);
    uchar *ptr_img_gray = img_gray.data;

    for (; ptr_src != ptr_src_end; ++ptr_src)
    {
        uchar max = 0, min = 255;
        uchar b = *ptr_src;
        if (b > max)
            max = b;
        if (b < min)
            min = b;

        uchar g = *(++ptr_src);
        if (g > max)
            max = g;
        if (g < min)
            min = g;

        uchar r = *(++ptr_src);
        if (r > max)
            max = r;
        if (r < min)
            min = r;

        uchar gray = (r * 30 + g * 59 + b * 11 + 50) / 100;
        if (gray > 120 && gray < 230 && (max - min) < 11)
            *ptr_img_gray = gray;
        else
            *ptr_img_gray = 0;

        ptr_img_gray++;
    }
    img_output = img_gray.clone();
    // imshow("gray", img_output);
    waitKey(1);
}

void NumberDetect::reprojectDepthImage(const Point &pt, float depth,
                                       Point3f &position)
{
    Point center(cam_matrix.at<double>(0, 2), cam_matrix.at<double>(1, 2));
    float focal_x = cam_matrix.at<double>(0, 0);
    float focal_y = cam_matrix.at<double>(1, 1);

    float u = pt.x - center.x;
    float v = pt.y - center.y;
    double k = sqrt(1 + pow((u / focal_x), 2) + pow((v / focal_y), 2));

    position.y = depth / k;
    position.x = 1.0 * position.y * u / focal_x;
    position.z = -1.0 * position.y * v / focal_y;
}

void NumberDetect::setCamera(Mat &_matrix, Mat &_dist)
{
    cam_matrix = _matrix.clone();
    cam_distortion = _dist.clone();
}

void NumberDetect::calcRotationTranslation(vector<Point3f> &point,float& _yaw,Mat& _tran)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->is_dense = 0;
    cloud->resize(point.size());
    for (int i = 0; i < point.size(); i++)
    {
        cloud->points[i].x = point[i].x;
        cloud->points[i].y = point[i].y;
        cloud->points[i].z = point[i].z;
    }
    std::vector<int> inliers;
    // 采样一致性模型对象
   
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
    ransac.setDistanceThreshold(0.01);
    ransac.computeModel();
    ransac.getInliers(inliers);
    Eigen::VectorXf coef = Eigen::VectorXf::Zero(4, 1);
    ransac.getModelCoefficients(coef);
       
    //ax+by+cz+d=0 =>ax+by+cz+1=0
    float a = coef(0) / coef(3), b = coef(1) / coef(3), c = coef(2) / coef(3);
    
    // sensor_msgs::PointCloud cloud2;
    // cloud2.header.stamp = ros::Time::now();
    // cloud2.header.frame_id = "map";
    // int SIZE = 500;
    // cloud2.points.resize(SIZE * SIZE);
    // for (int i = 0; i < SIZE; i++)
    // {
    //     for (int j = 0; j < SIZE; j++)
    //     {
    //         cloud2.points[i * SIZE + j].y = 5 * j * pow(-1, j) / SIZE;
    //         cloud2.points[i * SIZE + j].x =
    //             5 * i * pow(-1, j) / SIZE;
    //         cloud2.points[i * SIZE + j].z =
    //             (-1 - a * cloud2.points[i * SIZE + j].x - b * cloud2.points[i * SIZE + j].y) / c;
    //     }
    // }
    // cloud_pub.publish(cloud2);
    float yaw = -a / b;
    float pitch = c / b;
    float roll = c / a;

    Mat A(point);
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>> pt_eigen(A.ptr<float>(), A.rows, A.cols);
    Eigen::MatrixXf pt_mean = pt_eigen.colwise().mean();
    
    cout<<pt_mean<<endl;
    cout << "YAW: " << yaw << "  " << pitch << "  " << roll << endl;
    _yaw=yaw;
    _tran=Mat(1,3,CV_32FC1,pt_mean.data()).clone();
}