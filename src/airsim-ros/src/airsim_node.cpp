#include "airsim_node.hpp"
#include "common/CommonStructs.hpp"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <thread>

using namespace std;

#define DATA_FREQ 100000
#define CALC_TIME_START ros::Time start = ros::Time::now();
#define CALC_TIME_END                                                                                            \
    ROS_INFO("TIME : %f FPS: %f", (ros::Time::now() - start).toSec(), 1.0 / (ros::Time::now() - start).toSec()); \
    start = ros::Time::now();

AirsimNode::AirsimNode(ros::NodeHandle *_pnh, const std::string &_ip)
{
    pnh = _pnh;
    pub_imu = nh.advertise<sensor_msgs::Imu>("airsim/imu", 1);
    pub_magnetic =
        nh.advertise<sensor_msgs::MagneticField>("airsim/magnetic", 1);
    pub_gps = nh.advertise<sensor_msgs::NavSatFix>("airsim/gps", 1);
    pub_barometer =
        nh.advertise<geometry_msgs::Vector3Stamped>("airsim/barometer", 1);

    image_transport::ImageTransport it(nh);
    pub_image_front_rgb = it.advertiseCamera("airsim/image/front/rgb", 1);
    pub_image_front_depth = it.advertiseCamera("airsim/image/front/depth", 1);
    pub_image_down_rgb = it.advertise("airsim/image/down/rgb", 1);

    sub = nh.subscribe("airsim/object/down", 100, &AirsimNode::callb, this);

    ip_adress = _ip;
    control_client = new msr::airlib::MultirotorRpcLibClient(_ip);
    control_client->confirmConnection();

    RUNNING_FLAG = 0;
    memset(data_ready_flag, 0, 7 * sizeof(int));
    memset(exit_ready_flag, 0, 7 * sizeof(int));
    FN = 0;
}

AirsimNode::~AirsimNode() { delete control_client; }

void AirsimNode::getImageFrontRgbData(msr::airlib::MultirotorRpcLibClient *client)
{
    using namespace msr::airlib;
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;

    std::vector<ImageRequest> request = {
        ImageRequest(0, ImageType::Scene)};

    std::vector<ImageResponse> response = client->simGetImages(request);

    cv::Mat img_front_rgb = cv::imdecode(response[0].image_data_uint8, cv::IMREAD_COLOR);

    cv_bridge::CvImage msg_front_rgb;
    msg_front_rgb.header.stamp = ros::Time::now();
    msg_front_rgb.header.frame_id = "image";
    msg_front_rgb.image = img_front_rgb;
    msg_front_rgb.encoding = sensor_msgs::image_encodings::BGR8;

    sensor_msgs::CameraInfoPtr cam_info(new sensor_msgs::CameraInfo());
    cam_info->header = msg_front_rgb.header;
    cam_info->height = response[0].height;
    cam_info->width = response[0].width;
    cam_info->K[0] = 320;
    cam_info->K[2] = 320;
    cam_info->K[4] = 320;
    cam_info->K[5] = 240;
    cam_info->K[8] = 1;
    pub_image_front_rgb.publish(msg_front_rgb.toImageMsg(), cam_info);
}

void AirsimNode::getImageFrontDepthData(msr::airlib::MultirotorRpcLibClient *client)
{
    using namespace msr::airlib;
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;

    std::vector<ImageRequest> request = {
        ImageRequest(0, ImageType::DepthPerspective, true)};

    std::vector<ImageResponse> response = client->simGetImages(request);

    cv::Mat img_front_depth(response[0].height, response[0].width, CV_32FC1, response[0].image_data_float.data());
    if (img_front_depth.empty())
        return;
    cv_bridge::CvImage msg_front_depth;
    msg_front_depth.header.stamp = ros::Time::now();
    msg_front_depth.header.frame_id = "image";
    msg_front_depth.image = img_front_depth;
    msg_front_depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

    sensor_msgs::CameraInfoPtr cam_info(new sensor_msgs::CameraInfo());
    cam_info->header = msg_front_depth.header;
    cam_info->height = response[0].height;
    cam_info->width = response[0].width;
    cam_info->K[0] = 320;
    cam_info->K[2] = 320;
    cam_info->K[4] = 320;
    cam_info->K[5] = 240;
    cam_info->K[8] = 1;
    cam_info->roi.height = response[0].height;
    cam_info->roi.width = response[0].width;
    pub_image_front_depth.publish(msg_front_depth.toImageMsg(), cam_info);
}

void AirsimNode::getImageDownRgbData(msr::airlib::MultirotorRpcLibClient *client)
{
    using namespace msr::airlib;
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;

    std::vector<ImageRequest> request = {
        ImageRequest(3, ImageType::Scene)};

    std::vector<ImageResponse> response = client->simGetImages(request);

    cv::Mat img_down_rgb = cv::imdecode(response[0].image_data_uint8, cv::IMREAD_COLOR);

    cv_bridge::CvImage msg_down_rgb;
    msg_down_rgb.header.stamp = ros::Time::now();
    msg_down_rgb.header.frame_id = "image";
    msg_down_rgb.image = img_down_rgb;
    msg_down_rgb.encoding = sensor_msgs::image_encodings::BGR8;
    pub_image_down_rgb.publish(msg_down_rgb.toImageMsg());
}

void AirsimNode::getAllImageData(msr::airlib::MultirotorRpcLibClient *client)
{
    using namespace msr::airlib;
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;

    std::vector<ImageRequest> request = {
        ImageRequest(0, ImageType::Scene),
        ImageRequest(3, ImageType::Scene),
        ImageRequest(0, ImageType::DepthPerspective, true)};

    std::vector<ImageResponse> response = client->simGetImages(request);

    cv::Mat img_front_rgb = cv::imdecode(response[0].image_data_uint8, cv::IMREAD_COLOR);
    cv::Mat img_down_rgb = cv::imdecode(response[1].image_data_uint8, cv::IMREAD_COLOR);
    cv::Mat img_front_depth(response[2].height, response[2].width, CV_32FC1, response[2].image_data_float.data());

    cv_bridge::CvImage msg_front_rgb;
    msg_front_rgb.header.stamp = ros::Time::now();
    msg_front_rgb.header.frame_id = "image";
    msg_front_rgb.image = img_front_rgb;
    msg_front_rgb.encoding = sensor_msgs::image_encodings::BGR8;

    sensor_msgs::CameraInfoPtr cam_info(new sensor_msgs::CameraInfo());
    cam_info->header = msg_front_rgb.header;
    cam_info->height = response[0].height;
    cam_info->width = response[0].width;
    cam_info->K[0] = 320;
    cam_info->K[2] = 320;
    cam_info->K[4] = 320;
    cam_info->K[5] = 240;
    cam_info->K[8] = 1;
    cam_info->roi.height = response[0].height;
    cam_info->roi.width = response[0].width;

    pub_image_front_rgb.publish(msg_front_rgb.toImageMsg(), cam_info);

    cv_bridge::CvImage msg_down_rgb;
    msg_down_rgb.header.stamp = ros::Time::now();
    msg_down_rgb.header.frame_id = "image";
    msg_down_rgb.image = img_down_rgb;
    msg_down_rgb.encoding = sensor_msgs::image_encodings::BGR8;
    pub_image_down_rgb.publish(msg_down_rgb.toImageMsg());

    cv_bridge::CvImage msg_front_depth;
    msg_front_depth.header.stamp = ros::Time::now();
    msg_front_depth.header.frame_id = "image";
    msg_front_depth.image = img_front_depth;
    msg_front_depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    pub_image_front_depth.publish(msg_front_depth.toImageMsg(), cam_info);
}

void AirsimNode::getImuData(msr::airlib::MultirotorRpcLibClient *client)
{
    msr::airlib::ImuData airsim_img_data;
    airsim_img_data = client->getImudata();
    imu_data.header.frame_id = "base_link";
    imu_data.header.stamp = ros::Time::now();
    imu_data.angular_velocity.x = airsim_img_data.angular_velocity(0);
    imu_data.angular_velocity.y = airsim_img_data.angular_velocity(1);
    imu_data.angular_velocity.z = airsim_img_data.angular_velocity(2);
    imu_data.linear_acceleration.x = airsim_img_data.linear_acceleration(0);
    imu_data.linear_acceleration.y = airsim_img_data.linear_acceleration(1);
    imu_data.linear_acceleration.z = airsim_img_data.linear_acceleration(2);
    pub_imu.publish(imu_data);
}

void AirsimNode::getMagneticData(msr::airlib::MultirotorRpcLibClient *client)
{
    msr::airlib::MagnetometerData airsim_magnetic_data;
    airsim_magnetic_data = client->getMagnetometerdata();
    magnetic_data.header.frame_id = "base_link";
    magnetic_data.header.stamp = ros::Time::now();
    magnetic_data.magnetic_field.x =
        airsim_magnetic_data.magnetic_field_body(0);
    magnetic_data.magnetic_field.y =
        airsim_magnetic_data.magnetic_field_body(1);
    magnetic_data.magnetic_field.z =
        airsim_magnetic_data.magnetic_field_body(2);
    pub_magnetic.publish(magnetic_data);
}
// void AirsimNode::getGPSData(msr::airlib::MultirotorRpcLibClient *client)
// {
//     msr::airlib::GpsData airsim_aps_data;
//     airsim_aps_data = client->getGpsLocation();
//     gps_data.header.frame_id = "base_link";
//     gps_data.header.stamp = ros::Time::now();
//     gps_data.latitude = airsim_aps_data.latitude;
//     gps_data.longitude = airsim_aps_data.longitude;
//     gps_data.altitude = airsim_aps_data.altitude;
//     pub_gps.publish(gps_data);
// }
void AirsimNode::getBarometerData(msr::airlib::MultirotorRpcLibClient *client)
{
    msr::airlib::BarometerData airsim_barometer_data;
    airsim_barometer_data = client->getBarometerdata();
    barometer_data.header.frame_id = "base_link";
    barometer_data.header.stamp = ros::Time::now();
    barometer_data.vector.x = airsim_barometer_data.altitude; // altitude meter
    barometer_data.vector.y = airsim_barometer_data.pressure; // pressure
                                                              // Pascal
    pub_barometer.publish(barometer_data);
}

bool AirsimNode::takeoff()
{
    if (control_client->isApiControlEnabled() == 0)
    {
        control_client->enableApiControl(true);
        ROS_INFO("Command : Enable Api Control");
    }
    control_client->armDisarm(true);
    ROS_INFO("Command : Take Off");
    if (barometer_data.vector.x == 0)
    {
        ROS_ERROR("Cannot get barometer data!");
        return 0;
    }

    initial_height = barometer_data.vector.x;
    PIDctrl pid_height;
    pid_height.init(0.12, 0.0018, 1.5, 5);
    int count = 0;
    while (1)
    {
        double d_height = initial_height + 5 - barometer_data.vector.x;
        double d_throttle = pid_height.calc(d_height);
        move(0, 0, d_throttle, 0, 5);
        if (abs(d_height) < 0.1)
            break;
        // count++;
        else
            count = 0;
        cout << d_height << "   " << count << endl;
        usleep(10000);
    }
    control_client->hover();
    ROS_INFO("Take Off Finish");
}
bool AirsimNode::land()
{
    PIDctrl pid_height;
    pid_height.init(0.06, 0.0003, 1.5, 5);
    double land_target_altitude = 0;
    int highest_altitude = 25;
    int get_to_highest_altitude_flag = 0, start_land_flag = 0;

    double last_altitude = 0;
    int land_complete_cnt = 0;
    while (1)
    {
        //上升中：
        if (get_to_highest_altitude_flag == 0)
        {
            land_target_altitude = highest_altitude;
        }
        if (get_to_highest_altitude_flag == 0 && barometer_data.vector.x > highest_altitude - 0.5)
        {
            get_to_highest_altitude_flag = 1;
            start_land_flag = 1;
        }
        //下降中：
        if (start_land_flag == 1)
        {
            land_target_altitude -= 0.03;
        }
        //降落完成：
        land_complete_cnt++;
        if (land_complete_cnt >= 300)
        {
            //如果3秒前后高度差大于1,则判断仍在下降
            if (abs(barometer_data.vector.x - last_altitude) > 0.8)
            {
                land_complete_cnt = 0;
                last_altitude = barometer_data.vector.x;
            }
            //否则降落成功：跳出循环
            else
            {
                break;
            }
        }
        //PID部分
        double d_height = land_target_altitude - barometer_data.vector.x;
        double d_throttle = pid_height.calc(d_height);
        move(0, 0, d_throttle, 0, 5);

        cout << "   target_altitude:" << land_target_altitude << "real_altitude :" << barometer_data.vector.x << endl;
        usleep(10000);
    }
    ROS_INFO("Land complete ");
    // if (control_client->isApiControlEnabled() == 0)
    // {
    //     control_client->enableApiControl(true);
    //     ROS_INFO("Command : Enable Api Control");
    // }
    // float takeoffTimeout = 100;
    // control_client->land(takeoffTimeout);
    // sleep(3);
    // control_client->armDisarm(false);
    // ROS_INFO("Command : Land");
}
bool AirsimNode::move(float pitch, float roll, float throttle, float yaw,
                      float duration)
{
    if (control_client->isApiControlEnabled() == 0)
    {
        control_client->enableApiControl(true);
        ROS_INFO("Command : Enable Api Control");
    }
    control_client->moveByAngleThrottle(pitch, roll, throttle, yaw, duration); //ROS_INFO("Command : Move Pitch-%f Roll-%f Throtele-%f Yaw-%f Dur%f",pitch, roll, throttle, yaw,duration);
}
bool AirsimNode::hover() { control_client->hover(); }

void AirsimNode::createThread(int n)
{

    msr::airlib::MultirotorRpcLibClient *private_client = new msr::airlib::MultirotorRpcLibClient(ip_adress);
    private_client->confirmConnection();
    string thread_name;
    int first_flag = 1;
    while (RUNNING_FLAG)
    {
        ros::Time sstart;
        switch (n)
        {
        case 0:
            getImuData(private_client);
            thread_name = "getImuData";

            break;
        case 1:
            getMagneticData(private_client);
            thread_name = "getMagneticData";

            break;
        case 2:
            //getGPSData(client);
            thread_name = "getGPSData";

            break;
        case 3:
            getBarometerData(private_client);
            thread_name = "getBarometerData";

            break;
        case 4:
            getImageDownRgbData(private_client);
            thread_name = "getImageDownRgbData";

            break;
        case 5:
            getImageFrontRgbData(private_client);
            thread_name = "getImageFrontRgbData";
            break;
        case 6:
            getImageFrontDepthData(private_client);
            thread_name = "getImageFrontDepthData";

            break;
        }
        if (first_flag == 1)
        {
            ROS_INFO("Thread Create : %s", thread_name.c_str());
            first_flag = 0;
            data_ready_flag[n] = 1;
        }
        ros::spinOnce();
    }
    exit_ready_flag[n] = 1;
}
void AirsimNode::run()
{
    RUNNING_FLAG = 1;
    int thread_num[] = {0, 1, 2, 3, 4, 5, 6};
    int N = sizeof(thread_num) / sizeof(int);
    std::thread t[N];

    for (int i = 0; i < N; i++)
        t[i] = thread(&AirsimNode::createThread, this, thread_num[i]);

    std::for_each(t, t + N, [](thread &t) { t.detach(); });
    int sum = accumulate(data_ready_flag, data_ready_flag + 7, (int)0);
    ROS_INFO("DDDDDDDDDDDD %d", sum);
    while (sum < N)
    {
        sleep(1);
        ROS_INFO("DDDDDDDDDDDD %d", sum);
        sum = accumulate(data_ready_flag, data_ready_flag + 7, (int)0);
    }

    ROS_INFO("All Thread Create & Exit Airsim Run Function");
}
void AirsimNode::callb(const gf_perception::ObjectList &msg)
{
    ROS_INFO("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
}