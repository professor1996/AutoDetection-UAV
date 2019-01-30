

#include <iostream>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
using namespace std;
msr::airlib::MultirotorRpcLibClient* control_client;

void getImageFrontRgbData(cv_bridge::CvImage& msg_front_depth)
{
    using namespace msr::airlib;
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;

    std::vector<ImageRequest> request = {
        ImageRequest(0, ImageType::DepthPerspective, true)};

    std::vector<ImageResponse> response = control_client->simGetImages(request);

    cv::Mat img_front_depth(response[0].height, response[0].width, CV_32FC1, response[0].image_data_float.data());
    if (img_front_depth.empty())
        return;
 
    msg_front_depth.header.stamp = ros::Time::now();
    msg_front_depth.header.frame_id = "image";
    msg_front_depth.image = img_front_depth;
    msg_front_depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "airsim_image_front_depth");
    ros::NodeHandle nh;
    string ip("192.168.1.100");
    control_client = new msr::airlib::MultirotorRpcLibClient(ip);
    control_client->confirmConnection();
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_image_front_rgb = it.advertise("airsim/image/front/depth", 1);

    while(ros::ok()){
        cv_bridge::CvImage msg_front_rgb;
        getImageFrontRgbData(msg_front_rgb);
        pub_image_front_rgb.publish(msg_front_rgb.toImageMsg());
        ros::spinOnce();
    }
}