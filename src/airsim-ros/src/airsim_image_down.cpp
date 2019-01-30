


#include <iostream>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
using namespace std;

msr::airlib::MultirotorRpcLibClient* client;
void getImageDownRgbData(cv_bridge::CvImage& msg_down_rgb)
{
    using namespace msr::airlib;
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;

    std::vector<ImageRequest> request = {
        ImageRequest(3, ImageType::Scene)};


    std::vector<ImageResponse> response = client->simGetImages(request);

    cv::Mat img_front_rgb = cv::imdecode(response[0].image_data_uint8, cv::IMREAD_COLOR);

    msg_down_rgb.header.stamp = ros::Time::now();
    msg_down_rgb.header.frame_id = "image";
    msg_down_rgb.image = img_front_rgb;
    msg_down_rgb.encoding = sensor_msgs::image_encodings::BGR8;


}

int main(int argc, char** argv) {
    ros::init(argc, argv, "airsim_image_down");
    ros::NodeHandle nh;
    string ip("192.168.1.100");
    client = new msr::airlib::MultirotorRpcLibClient(ip);
    client->confirmConnection();
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_image_down_rgb = it.advertise("airsim/image/down/rgb", 1);

    while(ros::ok()){
        cv_bridge::CvImage msg_down_rgb;
        getImageDownRgbData(msg_down_rgb);
        pub_image_down_rgb.publish(msg_down_rgb.toImageMsg());
        ros::spinOnce();
    }
}