#include "airsim_node.hpp"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <signal.h>
#include <thread>
#include <gf_perception/Object.h>
#include <gf_perception/ObjectList.h>
using namespace std;

AirsimNode *ptr_airsim;
int target_mode[11] = {0}; //0-未知,1-板子.2-停机坪
int detect_num = -1;       //1-10 ,11-二维码
gf_perception::Object object_num;
void sig(int isg_no)
{
    if (ptr_airsim == NULL)
    {
        ROS_ERROR("Get Exit Signal Error");
    }
    else
    {
        ptr_airsim->RUNNING_FLAG = 0;
        ROS_INFO("Command : Get Exit Signal");
    }
}

void callb(const gf_perception::ObjectList& msg)
{
    ROS_ERROR("ENTT");
    object_num.number = -1;

    if (detect_num == -1)
        detect_num = 1;
    if (msg.count > 0)
    {
        for (int i = 0; i < msg.object.size(); i++)
        {
            target_mode[msg.object[i].number] = 2;
            if (msg.object[i].number == detect_num)
                object_num = msg.object[i];
        }
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "airsim_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    

    string ip("192.168.1.100");
    ROS_INFO("Connetct IP: %s", ip.c_str());
    AirsimNode airsim_node(&nh, ip);
    ptr_airsim = &airsim_node;
    signal(SIGINT, sig);

    //************ 获取数据 *******************
    airsim_node.run();
    ROS_INFO("Start Airsim Control");
    int thread_size = accumulate(airsim_node.data_ready_flag, airsim_node.data_ready_flag + 7, (int)0);
    //************* 控制部分 *****************
    ros::Subscriber sub_object_down = nh.subscribe("airsim/object/down", 1, &AirsimNode::callb,&airsim_node);
    //状态变量

    int detect_state = 0; //0-寻找,1-下视RGB检测 2-RGB图像检测,3-RGB+深度图像检测,4-圆圈检测，5-穿越/降落
    //0-未知,1-板子.2-停机坪
    target_mode[0] = 2;
    target_mode[1] = 2;
    target_mode[2] = 2;
    target_mode[10] = 2;

    //PID
    double control_throttle;
    double control_pitch;
    double control_roll;

    //高度
    PIDctrl pid_height;
    pid_height.init(0.08, 0.0003, 1.5, 5);
    double target_height = 15;

    //标志位
    int count_detect_down_cam = 0;
    int lost_detect_down_cam = 0;

    ros::Rate rate(30);

    // airsim_node.takeoff();
ros::spin();
    // while (1)
    // {
    //     ROS_INFO("Waiting %d", detect_num);
    //     usleep(10000);
       
    //     Once(); 
    //     if(detect_num!=-1)
    //         break;
    // }

    // while (airsim_node.RUNNING_FLAG)
    // {
    //     ROS_INFO("Detect Num: %d  Mode: %d State: %d", detect_num, target_mode[detect_num], detect_state);
    //     //数字
    //     if (detect_num < 11)
    //     {
    //         //板子未知类型 探索
    //         if (target_mode[detect_num] == 0)
    //         {
    //             ;
    //         }
    //         //障碍圈
    //         if (target_mode[detect_num] == 1)
    //         {
    //             ;
    //         }
    //         //停机坪
    //         if (target_mode[detect_num] == 2)
    //         {
    //             //寻找目标
    //             if (detect_state == 0)
    //             {
    //                 if (object_num.number != -1)
    //                 {
    //                     count_detect_down_cam++;
    //                 }
    //                 else
    //                 {
    //                     count_detect_down_cam -= 1;
    //                     if (count_detect_down_cam < 0)
    //                         count_detect_down_cam = 0;
    //                 }
    //                 if (count_detect_down_cam > 3)
    //                 {
    //                     detect_state = 1;
    //                 }
    //                 else
    //                 {

    //                     if (target_height < 25)
    //                         target_height = target_height + 0.1;
    //                 }
    //             }
    //             //下视深度检测
    //             else if (detect_state == 1)
    //             {
    //                 if (object_num.number == 1)
    //                 {
    //                     if (object_num.center.x > 320)
    //                         control_roll = -0.1;
    //                     else
    //                         control_roll = 0.1;
    //                     if (object_num.center.y > 240)
    //                         control_pitch = -0.1;
    //                     else
    //                         control_pitch = 0.1;
    //                     if (abs(object_num.center.x - 320) < 10)
    //                         control_roll = 0;
    //                     if (abs(object_num.center.y - 240) < 10)
    //                         control_pitch = 0;
    //                     if ((object_num.size.x * object_num.size.y) < 30000)
    //                         target_height = target_height - 0.1;
    //                 }
    //             }
    //         }
    //     }
    //     //二维码
    //     else
    //     {
    //         ;
    //     }

    //     double d_height = target_height - airsim_node.barometer_data.vector.x;
    //     control_throttle = pid_height.calc(d_height);
    //     ROS_INFO("Target :%f H: %f  D: %f", target_height, d_height, control_throttle);
    //     ROS_INFO("P: %f  R: %f", control_pitch, control_roll);
    //     airsim_node.move(-0.0, 0, control_throttle, 0, 5);
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    // // airsim_node.land();

    // int exit_flag_sum = accumulate(airsim_node.exit_ready_flag, airsim_node.exit_ready_flag + 7, (int)0);
    // while (exit_flag_sum < thread_size)
    // {
    //     sleep(1);
    //     ROS_INFO("Wait For Thread Exit");
    //     exit_flag_sum = accumulate(airsim_node.exit_ready_flag, airsim_node.exit_ready_flag + 7, (int)0);
    // }
    // ROS_INFO("Exit : Airsim Node");
}