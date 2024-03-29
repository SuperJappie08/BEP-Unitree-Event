/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <pthread.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <geometry_msgs/Twist.h>
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

// static geometry_msgs::Twist::Ptr msg;

// class CMDListener {
// public:
//   geometry_msgs::Twist::ConstPtr msg;
//   void callback(const geometry_msgs::Twist::ConstPtr& new_msg);
// };

// void CMDListener::callback(const geometry_msgs::Twist::ConstPtr& new_msg)
// {
// //   ROS_INFO("I heard: [%s]", msg->data.c_str());
//     g_msg = new_msg;
// }

// void cmdCallback(const geometry_msgs::Twist::ConstPtr& new_msg)
// {
//     msg->linear.x = new_msg->linear.x;
//     msg->linear.y = new_msg->linear.y;
//     msg->linear.z = new_msg->linear.z;
//     msg->angular.x = new_msg->angular.x;
//     msg->angular.y = new_msg->angular.y;
//     msg->angular.z = new_msg->angular.z;
// }

template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

void* update_stuff(void* param)
{
    geometry_msgs::Twist *data = (geometry_msgs::Twist *)param;
    while(ros::ok){
      auto tmp = ros::topic::waitForMessage<geometry_msgs::Twist>("/cmd_bep");
        
    data->linear.x = tmp->linear.x;
    data->linear.y = tmp->linear.y;
    data->linear.z = tmp->linear.z;
    data->angular.x = tmp->angular.x;
    data->angular.y = tmp->angular.y;
    data->angular.z = tmp->angular.z;
        usleep(2000);
    }
}

template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle n;

    // CMDListener cmd;

    // ros::Subscriber controller_sub = n.subscribe<geometry_msgs::Twist>("/cmd_bep", 1, cmdCallback);
    ros::Publisher state_pub = n.advertise<unitree_legged_msgs::HighState>("/bep_controller/robot_state", 10);

    ros::Rate loop_rate(500);

    // SetLevel(HIGHLEVEL);
    long motiontime = 0;
    TCmd SendHighLCM = {0};
    TState RecvHighLCM = {0};
    unitree_legged_msgs::HighCmd SendHighROS;
    unitree_legged_msgs::HighState RecvHighROS;
    geometry_msgs::Twist msg;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);
    pthread_create(&tid, NULL, update_stuff, &msg);
    while (ros::ok()){
        motiontime = motiontime+2;
        roslcm.Get(RecvHighLCM);
        RecvHighROS = ToRos(RecvHighLCM);
        
        state_pub.publish(RecvHighROS);

        SendHighROS.mode = 0;      
        SendHighROS.gaitType = 0;
        SendHighROS.speedLevel = 0;
        SendHighROS.footRaiseHeight = 0;
        SendHighROS.bodyHeight = 0;
        SendHighROS.euler[0]  = 0;
        SendHighROS.euler[1] = 0;
        SendHighROS.euler[2] = 0;
        SendHighROS.velocity[0] = 0.0f;
        SendHighROS.velocity[1] = 0.0f;
        SendHighROS.yawSpeed = 0.0f;
        SendHighROS.reserve = 0;

        // geometry_msgs::Twist::ConstPtr msg = ros::topic::waitForMessage<geometry_msgs::Twist>("/cmd_bep")

        if (abs(msg.linear.x) > 0.0f) {
            SendHighROS.mode = 2;
            SendHighROS.gaitType = 2;
            SendHighROS.velocity[0] = (float) msg.linear.x;
            SendHighROS.footRaiseHeight = 0.1;
            SendHighROS.bodyHeight = 0.28;
        }

        if (abs(msg.linear.y) > 0.0f) {
            SendHighROS.mode = 2;
            SendHighROS.gaitType = 2;
            SendHighROS.velocity[1] = (float) msg.linear.y;
            SendHighROS.footRaiseHeight = 0.1;
            SendHighROS.bodyHeight = 0.28;
        }

        if (abs(msg.angular.z) > 0) {
            SendHighROS.mode = 2;
            SendHighROS.gaitType = 2;
            SendHighROS.yawSpeed = (float) msg.angular.z;
            SendHighROS.footRaiseHeight = 0.1;
            SendHighROS.bodyHeight = 0.28;
        }

        // if(motiontime > 0 && motiontime < 1000){
        //     SendHighROS.mode = 1;
        //     SendHighROS.euler[0] = -0.3;
        // }
        // if(motiontime > 1000 && motiontime < 2000){
        //     SendHighROS.mode = 1;
        //     SendHighROS.euler[0] = 0.3;
        // }
        // if(motiontime > 2000 && motiontime < 3000){
        //     SendHighROS.mode = 1;
        //     SendHighROS.euler[1] = -0.2;
        // }
        // if(motiontime > 3000 && motiontime < 4000){
        //     SendHighROS.mode = 1;
        //     SendHighROS.euler[1] = 0.2;
        // }
        // if(motiontime > 4000 && motiontime < 5000){
        //     SendHighROS.mode = 1;
        //     SendHighROS.euler[2] = -0.2;
        // }
        // if(motiontime > 5000 && motiontime < 6000){
        //     SendHighROS.mode = 1;
        //     SendHighROS.euler[2] = 0.2;
        // }
        // if(motiontime > 6000 && motiontime < 7000){
        //     SendHighROS.mode = 1;
        //     SendHighROS.bodyHeight = -0.2;
        // }
        // if(motiontime > 7000 && motiontime < 8000){
        //     SendHighROS.mode = 1;
        //     SendHighROS.bodyHeight = 0.1;
        // }
        // if(motiontime > 8000 && motiontime < 9000){
        //     SendHighROS.mode = 1;
        //     SendHighROS.bodyHeight = 0.0;
        // }
        // if(motiontime > 9000 && motiontime < 11000){
        //     SendHighROS.mode = 5;
        // }
        // if(motiontime > 11000 && motiontime < 13000){
        //     SendHighROS.mode = 6;
        // }
        // if(motiontime > 13000 && motiontime < 14000){
        //     SendHighROS.mode = 0;
        // }
        // if(motiontime > 14000 && motiontime < 18000){
        //     SendHighROS.mode = 2;
        //     SendHighROS.gaitType = 2;
        //     SendHighROS.velocity[0] = 0.4f; // -1  ~ +1
        //     SendHighROS.yawSpeed = 2;
        //     SendHighROS.footRaiseHeight = 0.1;
        //     // printf("walk\n");
        // }
        // if(motiontime > 18000 && motiontime < 20000){
        //     SendHighROS.mode = 0;
        //     SendHighROS.velocity[0] = 0;
        // }
        // if(motiontime > 20000 && motiontime < 24000){
        //     SendHighROS.mode = 2;
        //     SendHighROS.gaitType = 1;
        //     SendHighROS.velocity[0] = 0.2f; // -1  ~ +1
        //     SendHighROS.bodyHeight = 0.1;
        //     // printf("walk\n");
        // }
        // if(motiontime>24000 ){
        //     SendHighROS.mode = 1;
        // }

        SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
        roslcm.Send(SendHighLCM);
        ros::spinOnce();
        loop_rate.sleep(); 
    }
    return 0;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "bep_onboard_controller");

    UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
    mainHelper<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
    
}