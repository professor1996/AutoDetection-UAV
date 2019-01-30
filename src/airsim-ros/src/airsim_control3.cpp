#include "airsim_control.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int16.h>
using namespace std;
double kp, ki, kd, pid_max, kp2, ki2, kd2, pid_max2;
AirsimControl::AirsimControl()
{
    sub_imu = nh.subscribe("airsim/imu/fusion", 1, &AirsimControl::cb_imu, this);
    sub_gps = nh.subscribe("airsim/gps", 1, &AirsimControl::cb_gps, this);
    sub_magnetic = nh.subscribe("airsim/magnetic", 1, &AirsimControl::cb_magnetic, this);
    sub_baromrter = nh.subscribe("airsim/barometer", 1, &AirsimControl::cb_barometer, this);
    sub_object_front = nh.subscribe("gaofen_detection/number", 1, &AirsimControl::cb_object_front, this);
    sub_object_down = nh.subscribe("airsim/object/down", 1, &AirsimControl::cb_object_down, this);
    sub_object_circle = nh.subscribe("airsim/depth/circle", 1, &AirsimControl::cb_object_circle, this);
    sub_depth_count = nh.subscribe("airsim/depth/count", 1, &AirsimControl::cb_depth_count, this);
    sub_depth_count_full = nh.subscribe("airsim/depth/count_full", 1, &AirsimControl::cb_depth_count_full, this);
    pub_front_camera_pose = nh.advertise<std_msgs::Int16>("airsim/front_camera/pose_state", 1);
    pub_height_filter = nh.advertise<geometry_msgs::Vector3Stamped>("airsim/height_filter", 1);
    detect_num = -1;
    memset(target_mode, 0, sizeof(int) * 11);
    memset(target_mode_count, 0, sizeof(int) * 11 * 2);

    control_client = new msr::airlib::MultirotorRpcLibClient("192.168.1.100");
    control_client->confirmConnection();
    error_code = 0;

    running_count = 0;
    leftright_count = -1;
    forward_count = 0;
    search_flag = 0;
}

void AirsimControl::cb_imu(const sensor_msgs::Imu &msg)
{
    msg_imu = msg;
}
void AirsimControl::cb_gps(const sensor_msgs::NavSatFix &msg)
{
    msg_gps = msg;
}
void AirsimControl::cb_magnetic(const sensor_msgs::MagneticField &msg)
{
    msg_magnetic = msg;
}
void AirsimControl::cb_barometer(const geometry_msgs::Vector3Stamped &msg)
{
    msg_barometer = msg;
}
void AirsimControl::cb_depth_count(const geometry_msgs::Vector3Stamped &msg)
{
    msg_pre_depth_count = msg_depth_count;
    msg_depth_count = msg;
}
void AirsimControl::cb_depth_count_full(const geometry_msgs::Vector3Stamped &msg)
{
    msg_pre_depth_count_full = msg_depth_count_full;
    msg_depth_count_full = msg;
}
void AirsimControl::cb_object_front(const gf_perception::ObjectList &msg)
{
    msg_objects_front = msg;
    object_front = gf_perception::Object();
    if (msg.count > 0)
    {
        for (int i = 0; i < msg.object.size(); i++)
        {
            ROS_ERROR("****FIND OBJ %d %d %d %d", msg.object[i].number, msg.object[i].type, target_mode_count[msg.object[i].number][0], target_mode_count[msg.object[i].number][1]);
            ROS_ERROR("SIZE %f", msg.object[i].size.x * msg.object[i].size.y);
            if (msg.object[i].type == 2 && msg.object[i].center.y > 450 && msg.object[i].size.x * msg.object[i].size.y > 5000)
                continue;

            if (msg.object[i].number == detect_num)
                object_front = msg.object[i];
            if (msg.object[i].number >= detect_num && msg.object[i].number < detect_num + 3)
            {
                if (msg.object[i].type == 1)
                {
                    target_mode_count[msg.object[i].number][0] += 1;
                }
                else if (msg.object[i].type == 2)
                {
                    target_mode_count[msg.object[i].number][1] += 2;
                }
                else if (msg.object[i].type == 3)
                {
                    target_mode_count[msg.object[i].number][1] += 5;
                }
            }

            if (target_mode_count[msg.object[i].number][0] > target_mode_count[msg.object[i].number][1] * 4 && target_mode_count[msg.object[i].number][0] > 30)
            {
                target_mode[msg.object[i].number] = 1;
            }
            if (target_mode_count[msg.object[i].number][1] > target_mode_count[msg.object[i].number][0] * 2 && target_mode_count[msg.object[i].number][1] > 30)
            {
                target_mode[msg.object[i].number] = 2;
            }
        }
    }
}
void AirsimControl::cb_object_down(const gf_perception::ObjectList &msg)
{
    msg_objects_down = msg;
    object_down = gf_perception::Object();
    if (detect_num == -1)
        detect_num = 1;
    if (msg.count > 0)
    {
        for (int i = 0; i < msg.object.size(); i++)
        {
            // target_mode[msg.object[i].number] = 2;
            if (msg.object[i].number == detect_num)
            {
                object_down = msg.object[i];
                target_mode[msg.object[i].number] = 2;
            }
        }
    }
}
void AirsimControl::cb_object_circle(const geometry_msgs::Vector3Stamped &msg)
{
    msg_circle = msg;
}

//均值滤波 -> 高度
////////////////////////////////////////////
#define filter_N 5

float filter_seq_height[filter_N] = {0};
float filtered_height = 0;

float sum(float a[], size_t len)
{
    float sum = 0;
    for (int i = 0; i < len; i++)
    {
        sum += a[i];
    }
    return sum;
}

float find_min(float a[], size_t len)
{
    float min = a[0];

    for (int j = 1; j < len; j++)
    {
        if (a[j] < min)
            min = a[j];
    }

    return min;
}

float find_max(float a[], size_t len)
{
    float max = a[0];

    for (int j = 1; j < len; j++)
    {
        if (a[j] > max)
            max = a[j];
    }
    return max;
}
////////////////////////////////////////////

//一阶滞后滤波
////////////////////////////////////////////
#define one_a 0.8

float last_filter_value = 0;
float this_time_value = 0;
float one_filter_back(float new_value, float last_filter_value)
{
    return one_a * last_filter_value + (1 - one_a) * new_value;
}

////////////////////////////////////////////

void AirsimControl::run()
{
    int detect_state = 0; //0-寻找,1-下视RGB检测 2-RGB图像检测,3-RGB+深度图像检测,4-圆圈检测，5-穿越/降落
    //0-未知,1-板子.2-停机坪
    target_mode[0] = 2;
    target_mode[1] = 0;
    target_mode[2] = 0;
    target_mode[3] = 0;
    target_mode[4] = 0;
    target_mode[5] = 0;
    target_mode[10] = 2;

    //PID
    double control_throttle;
    double control_pitch;
    double control_roll;
    double control_yaw;
    //高度
    PIDctrl pid_height;
    //起飞PID
    pid_height.init(kp, ki, kd, pid_max); //0.12, 0.0018, 1.5, 5
    double target_height = 15;
    PIDctrl pid_pitch;
    pid_pitch.init(0.0002, 0, 0.006, 0.025);
    PIDctrl pid_roll;
    pid_roll.init(0.0002, 0, 0.006, 0.025);
    PIDctrl pid_yaw;
    pid_yaw.init(1, 0, 0, 1);
    double target_yaw;

    //标志位
    int hover_flag = 0;
    //未知
    int rotate_flag = 1; //0-stay 1 center-left  2-left-center 3 center-right 4 right-center
    int lost_count = 0;
    int circle_count = 0;
    int circle_center_count = 0;
    int go_forward_count = 0;
    int up_down_mode = 0;
    //下视
    int count_detect_down_cam = 0;
    int lost_detect_down_cam = 0;
    int count_center_down_cam = 0;
    int down_search_mode = 0; //0 - 前视 1 -向右 2 - 向左
    float target_distance, pre_target_distance;
    ros::Rate rate(30);

    while (1)
    {
        sleep(1);
        ros::spinOnce();
        if (msg_barometer.vector.x != 0)
            break;
    }
    initial_height = msg_barometer.vector.x;
    takeoff(&pid_height);

    //起飞切换到move , 更换高度PID参数:
    //float dynamic_ki2 = 0.00108;
    pid_height.init(kp2, ki2, kd2, pid_max2); //0.08, 0.0003, 1.5, 5

    // //正式飞行前瞎飞计时:
    // int xiafei_cnt = 0 , stop_cnt_flag = 0;

    target_height = msg_barometer.vector.x;
    initial_yaw = tf::getYaw(msg_imu.orientation);
    target_yaw = initial_yaw;

    ros::NodeHandle private_nh("~");
    while (ros::ok())
    {

        /////
        //修改PID参数:
        private_nh.getParam("kp", kp);
        private_nh.getParam("ki", ki);
        private_nh.getParam("kd", kd);
        private_nh.getParam("max", pid_max);
        private_nh.getParam("kp2", kp2);
        private_nh.getParam("ki2", ki2);
        private_nh.getParam("kd2", kd2);
        private_nh.getParam("max2", pid_max2);
        pid_height.init(kp2, ki2, kd2, pid_max2);
        ///////////////////////////////////////////

        // ROS_ERROR("Detect Num: %d  Mode: %d State: %d", detect_num, target_mode[detect_num], detect_state);
        control_roll = 0;
        control_pitch = 0;
        control_yaw = 0;
        hover_flag = 0;
        std_msgs::Int16 front_cam_pose_msg;
        //数字
        if (detect_num > 0 && detect_num < 11)
        {

            //板子未知类型 探索
            if (target_mode[detect_num] == 0)
            {
                //寻找
                if (detect_state == 0)
                {
                    target_height = initial_height + 7;
                    detect_state = 1;
                }
                if (detect_state == 1)
                {
                    target_yaw = initial_yaw + 3.1415 / 2;
                    if (abs(tf::getYaw(msg_imu.orientation) - target_yaw) < 0.2)
                    {
                        if (go_forward_count < 30)
                        {
                            control_pitch = -0.05;
                        }
                        else
                        {
                            control_pitch = 0.0;
                        }
                        if (msg_depth_count_full.vector.x > 0 && msg_depth_count_full.vector.x >= msg_pre_depth_count_full.vector.x)
                        {
                            control_pitch = 0.3;
                        }
                        else if (msg_depth_count_full.vector.x > 0 &&msg_depth_count_full.vector.x <0.3 && msg_depth_count_full.vector.x < msg_pre_depth_count_full.vector.x)
                        {
                            control_client->hover();
                            sleep(3);
                            detect_state = 2;
                            go_forward_count = 0;
                        }
                    }
                }
                if (detect_state == 2)
                {
                    target_yaw = initial_yaw - 3.1415 / 2;
                    if (abs(tf::getYaw(msg_imu.orientation) - target_yaw) < 0.2)
                    {
                        if (go_forward_count < 30)
                        {
                            control_pitch = -0.05;
                        }
                        else
                        {
                            control_pitch = 0.0;
                        }
                        if (msg_depth_count_full.vector.x > 0 && msg_depth_count_full.vector.x >= msg_pre_depth_count_full.vector.x)
                        {
                            control_pitch = 0.3;
                        }
                        else if (msg_depth_count_full.vector.x > 0 &&msg_depth_count_full.vector.x <0.3&& msg_depth_count_full.vector.x < msg_pre_depth_count_full.vector.x)
                        {
                            control_client->hover();
                            sleep(3);
                            detect_state = 3;
                            go_forward_count = 0;
                        }
                    }
                }
                if (detect_state == 3)
                {
                    target_height = initial_height + 3;
                    target_yaw = initial_yaw;
                    if (abs(tf::getYaw(msg_imu.orientation) - target_yaw) < 0.2)
                    {
                        search(control_pitch, control_roll, target_yaw);
                    }
                }
            }
            //障碍圈
            if (target_mode[detect_num] == 1)
            {
            }
            //停机坪
            if (target_mode[detect_num] == 2)
            {
            }
        }
        //二维码
        else
        {
            ;
        }

        current_yaw = tf::getYaw(msg_imu.orientation);
        if (target_yaw > 0 && current_yaw < 0 && (target_yaw - current_yaw) > 3.14)
        {
            control_yaw = pid_yaw.calc(current_yaw + 6.28 - target_yaw);
        }
        else if (target_yaw < 0 && current_yaw > 0 && (current_yaw - target_yaw) > 3.14)
        {
            control_yaw = pid_yaw.calc(-(target_yaw + 6.28 - current_yaw));
        }
        else
        {
            control_yaw = pid_yaw.calc(current_yaw - target_yaw);
        }

        //filter:
        // for (int i = 0; i < filter_N - 1; i++)
        // {
        //     filter_seq_height[i] = filter_seq_height[i + 1];
        // }

        // filter_seq_height[filter_N - 1] = msg_barometer.vector.x;

        // filtered_height = (sum(filter_seq_height, filter_N) - find_max(filter_seq_height, filter_N) - find_min(filter_seq_height, filter_N)) / (filter_N - 2);

        filtered_height = one_filter_back(msg_barometer.vector.x, filtered_height);
        geometry_msgs::Vector3Stamped msg_h;
        msg_h.header.stamp = ros::Time::now();
        msg_h.vector.x = filtered_height;
        msg_h.vector.y = current_yaw;
        msg_h.vector.z = target_height;
        pub_height_filter.publish(msg_h);

        ////////////////////////////

        double d_height = target_height - filtered_height; //msg_barometer.vector.x;
        control_throttle = pid_height.calc(d_height);
        ROS_INFO("Target H:%f dH: %f Target Y:%f  dY,%f", target_height, d_height, target_yaw, tf::getYaw(msg_imu.orientation) - target_yaw);
        ROS_INFO("P: %f  R: %f  Y: %f  T: %f", control_pitch, control_roll, control_yaw, control_throttle);
        if (hover_flag)
        {
            control_client->hover();
            usleep(300000);
        }
        else
            move(control_pitch, control_roll, control_throttle + 0.58, control_yaw, 5);
        //pub_front_camera_pose.publish(front_cam_pose_msg);
        ros::spinOnce();
        rate.sleep();
    }
}

bool AirsimControl::takeoff(PIDctrl *pid)
{
    if (control_client->isApiControlEnabled() == 0)
    {
        control_client->enableApiControl(true);
        ROS_INFO("Command : Enable Api Control");
    }
    control_client->armDisarm(true);
    ROS_INFO("Command : Take Off");
    if (msg_barometer.vector.x == 0)
    {
        ROS_ERROR("Cannot get barometer data!");
        return 0;
    }

    // PIDctrl pid_height;
    // pid_height.init(0.12, 0.0018, 1.5, 5); // 0.12, 0.0018, 1.5, 5

    int count = 0;
    while (1)
    {
        filtered_height = one_filter_back(msg_barometer.vector.x, filtered_height);

        double d_height;

        d_height = initial_height + 8 - filtered_height;

        double d_throttle = pid->calc(d_height);
        move(0, 0, d_throttle, 0, 5);
        if (abs(d_height) < 1)
            break;
        // count++;
        else
            count = 0;
        usleep(33000);
        ros::spinOnce();
    }
    control_client->hover();
    ROS_INFO("Take Off Finish");
}

bool AirsimControl::land()
{
    PIDctrl pid_height;
    pid_height.init(0.06, 0.0003, 1.5, 5);
    double land_target_altitude = msg_barometer.vector.x;
    int highest_altitude = 25;
    int get_to_highest_altitude_flag = 0, start_land_flag = 0;

    double last_altitude = 0;
    int land_complete_cnt = 0;
    control_client->hover();
    sleep(1);
    while (1)
    {
        filtered_height = one_filter_back(msg_barometer.vector.x, filtered_height);

        //上升中：
        // if (get_to_highest_altitude_flag == 0)
        // {
        //     land_target_altitude = highest_altitude;
        // }
        // if (get_to_highest_altitude_flag == 0 && msg_barometer.vector.x > highest_altitude - 0.5)
        // {
        //     get_to_highest_altitude_flag = 1;
        //     start_land_flag = 1;
        // }
        //下降中：
        // if (start_land_flag == 1)
        // {
        land_target_altitude -= 0.055;
        // }
        //降落完成：
        if (land_target_altitude <= 10.50)
        {
            break;
        }
        // land_complete_cnt++;
        // if (land_complete_cnt >= 150)
        // {
        //     //如果3秒前后高度差大于1,则判断仍在下降
        //     if (abs(msg_barometer.vector.x - last_altitude) > 2)
        //     {
        //         land_complete_cnt = 0;
        //         last_altitude = msg_barometer.vector.x;
        //     }
        //     //否则降落成功：跳出循环
        //     else
        //     {
        //         break;
        //     }
        // }
        //PID部分
        double d_height = land_target_altitude - filtered_height;
        double d_throttle = pid_height.calc(d_height);
        move(0, 0, d_throttle, 0, 5);

        cout << "   target_altitude:" << land_target_altitude << "real_altitude :" << msg_barometer.vector.x << endl;
        usleep(10000);
        ros::spinOnce();
    }
    ROS_INFO("Land complete ");
}

// bool AirsimControl::land()
// {
//     control_client->hover();
//     sleep(1);
//     if (control_client->isApiControlEnabled() == 0)
//     {
//         control_client->enableApiControl(true);
//         ROS_INFO("Command : Enable Api Control");
//     }
//     float takeoffTimeout = 100;
//     control_client->land(takeoffTimeout);
//     sleep(1);
//     control_client->armDisarm(false);
//     ROS_INFO("Command : Land");
// }

bool AirsimControl::move(float pitch, float roll, float throttle, float yaw,
                         float duration)
{
    if (control_client->isApiControlEnabled() == 0)
    {
        control_client->enableApiControl(true);
        ROS_INFO("Command : Enable Api Control");
    }
    control_client->armDisarm(true);
    control_client->moveByAngleThrottle(pitch, roll, throttle, yaw, duration); //ROS_INFO("Command : Move Pitch-%f Roll-%f Throtele-%f Yaw-%f Dur%f",pitch, roll, throttle, yaw,duration);
}

void AirsimControl::search(double &pitch, double &roll, double &yaw)
{
    if (leftright_count > 0) //右
    {
        leftright_count++;
        if (abs(leftright_count) < 40)
        {
            roll = 0.1;
        }
        else if (abs(leftright_count) > 40 && abs(leftright_count) < 70)
        {
            roll = 0.05;
        }
        else
        {
            // pitch = 0.0;
            roll = 0.0;
        }
        if (msg_depth_count.vector.y > 0.3)
        {
            roll = -0.3;
            search_flag = 1;
        }
        else if (search_flag == 1 && msg_depth_count.vector.y > 0 && msg_depth_count.vector.y < msg_pre_depth_count.vector.y && abs(leftright_count) > 50)
        {
            control_client->hover();
            sleep(2);
            leftright_count = -1;
            search_flag = 0;
        }
    }
    else //左
    {
        leftright_count--;
        if (abs(leftright_count) < 50)
        {
            roll = -0.1;
        }
        else if (abs(leftright_count) > 50 && abs(leftright_count) < 80)
        {
            roll = -0.05;
        }
        else
        {
            // pitch = 0.0;
            roll = 0.0;
        }
        if (msg_depth_count.vector.x > 0.3)
        {
            roll = 0.3;
            search_flag = 1;
        }
        else if (search_flag == 1 && msg_depth_count.vector.x > 0 && msg_depth_count.vector.x < msg_pre_depth_count.vector.x && abs(leftright_count) > 50)
        {
            control_client->hover();
            sleep(2);
            leftright_count = 1;
            search_flag = 0;
            ROS_INFO("TURN");
        }
    }
    if (abs(leftright_count) > 400)
        leftright_count = -leftright_count / abs(leftright_count);
    ROS_INFO(" SEA %d,%f,%f,%f", leftright_count, msg_depth_count.vector.x, msg_depth_count.vector.y, msg_depth_count.vector.z);
}

bool AirsimControl::go_forward(double &pitch, double &roll, double &yaw)
{
    yaw = initial_yaw;
    if (abs(tf::getYaw(msg_imu.orientation) - yaw) < 0.02)
    {
        forward_count++;
        if (forward_count < 80)
        {
            pitch = -0.05;
        }
        else if (forward_count > 80 && forward_count < 200)
        {
            pitch = 0.0;
        }
        else if (forward_count > 200 && forward_count < 240)
        {
            pitch = 0.15;
        }
        else
        {
            control_client->hover();
            sleep(2);
            forward_count = 0;
            return 1;
        }
    }
    ROS_ERROR("##############33");
    return 0;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "airsim_control");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    private_nh.getParam("kp", kp);
    private_nh.getParam("ki", ki);
    private_nh.getParam("kd", kd);
    private_nh.getParam("max", pid_max);
    private_nh.getParam("kp2", kp2);
    private_nh.getParam("ki2", ki2);
    private_nh.getParam("kd2", kd2);
    private_nh.getParam("max2", pid_max2);
    AirsimControl airsim_ctrl;
    sleep(3);
    airsim_ctrl.run();
}

void gostraight()
{
    // PIDctrl pid_height;
    // pid_height.init(0.08, 0.0003, 1.5, 1000000);
    // double d_height = target_height - msg_barometer.vector.x;
    // control_throttle = pid_height.calc(d_height);
    ;
}