#include "uavData.h"
// #include "service.h"
#include "Utils.h"
#include "ttalinkUtils.h"

#include "ros/ros.h"

#include "ttauav_node/gimbalControl.h"
#include "ttauav_node/takeoffOrLanding.h"
#include "ttauav_node/flightByOffset.h"
#include "ttauav_node/flightByVel.h"

#include <iostream>
#include <thread>
#include <vector>
#include <mutex>

typedef float float32;

struct request_flight_ByVel{
    float32 vel_N;
    float32 vel_E;
    float32 vel_D;   
    float32 flight_time;
};


uavData *m_puavData = NULL;

// std::mutex mtx;

void takeoff_thread() {
    // 初始化 rosuav_ctrl_loop_input
    ttalink_rosuav_ctrl_loop_input_t rosuav_ctrl_loop_input = {0};
    rosuav_ctrl_loop_input.flight_ctrl_status = ROS_F_GPS_AUTO_TAKE_OFF;
    printf("request:takeoff\n");

    if (m_puavData) {
        // 调用 rosuav_ctrl_loopinput 函数
        ttalinkUtils::rosuav_ctrl_loopinput(m_puavData->getDataPort(), TTALINK_FC_ADDRESS, TTALINK_EMBE_ADDRESS, &rosuav_ctrl_loop_input, 0);
        printf("takeoff success\n");
    }

}

void land_thread(){
     // 初始化 rosuav_ctrl_loop_input
    ttalink_rosuav_ctrl_loop_input_t rosuav_ctrl_loop_input = {0};
    rosuav_ctrl_loop_input.flight_ctrl_status = ROS_F_GPS_NAVGATION_ALTI_VEL;
    printf("request:land\n");

    if (m_puavData) {
        // 调用 rosuav_ctrl_loopinput 函数
        ttalinkUtils::rosuav_ctrl_loopinput(m_puavData->getDataPort(), TTALINK_FC_ADDRESS, TTALINK_EMBE_ADDRESS, &rosuav_ctrl_loop_input, 0);
        printf("takeoff success\n");
    }
}

void flightByvel(ttauav_node::flightByVel::Request message){
     // 初始化 rosuav_ctrl_loop_input
    ttalink_rosuav_ctrl_loop_input_t rosuav_ctrl_loop_input = {0};

    rosuav_ctrl_loop_input.flight_ctrl_status = ROS_F_GPS_POS_VEL_ALTI_VEL;

    rosuav_ctrl_loop_input.velN = message.vel_n;
    rosuav_ctrl_loop_input.velE = message.vel_e;
    rosuav_ctrl_loop_input.velD = message.vel_d;

    rosuav_ctrl_loop_input.atti_yaw = message.targetYaw;

    rosuav_ctrl_loop_input.param[0] = message.fly_time;

    ttalinkUtils::rosuav_ctrl_loopinput(m_puavData->getDataPort(), TTALINK_FC_ADDRESS, TTALINK_EMBE_ADDRESS, &rosuav_ctrl_loop_input, 0);
}

void  gimbalControl(float32 targetPitch){
     ttalinkUtils::SendGeneralCommand(m_puavData->getDataPort(), TTALINK_PTZ_ADDRESS, TTALINK_EMBE_ADDRESS
     , 11223344, 0, 109, targetPitch, 0, 0,0,0,0,0);
}

int main(int argc, char  *argv[]) {
 
    ros::init(argc, argv, "flight_test");
    ros::NodeHandle n;

    m_puavData = new uavData(47150);
    m_puavData->start();
    printf("uavdata start\n");

    //  // 创建线程
    // std::thread thread(takeoff_thread);

    // // 等待线程完成
    // thread.join();

    // ttalink_rosuav_ctrl_loop_input_t rosuav_ctrl_loop_input = {0};
    // rosuav_ctrl_loop_input.flight_ctrl_status = ROS_F_GPS_AUTO_TAKE_OFF;
    // printf("request:takeoff\n");

    // if(m_puavData)
    // {
    //   ttalinkUtils::rosuav_ctrl_loopinput(m_puavData->getDataPort(), TTALINK_FC_ADDRESS, TTALINK_EMBE_ADDRESS, &rosuav_ctrl_loop_input, 0);
    //   printf("takeoff success\n");
    // }

    
    // rosuav_ctrl_loop_input.flight_ctrl_status = ROS_F_GPS_POS_VEL_ALTI_VEL;

    // rosuav_ctrl_loop_input.velN = 1;
    // rosuav_ctrl_loop_input.velE = 0;
    // rosuav_ctrl_loop_input.velD = 0;

    // rosuav_ctrl_loop_input.atti_yaw = 0;

    // rosuav_ctrl_loop_input.param[0] = 1000;
    // ttalinkUtils::rosuav_ctrl_loopinput(m_puavData->getDataPort(), TTALINK_FC_ADDRESS, TTALINK_EMBE_ADDRESS, &rosuav_ctrl_loop_input, 0);
    // printf("flight success\n");

    while (true) {
        int choice;
        std::cout << "请输入数字来选择" << std::endl;
        std::cout << "1 - takeoff,2 - land,3 - flightByvel,4 - gimbalControl,5 - quit" << std::endl;
        
        std::cin >> choice;

        // 根据用户输入，选择要启动的线程
        if (choice == 1) {
            // mtx.lock(); // 锁定互斥锁
            // std::thread t1(takeoff_thread);
            // t1.join(); // 等待线程1完成
            // mtx.unlock(); // 解锁互斥锁
            takeoff_thread();
        } else if (choice == 2) {
            // mtx.lock(); // 锁定互斥锁
            // std::thread t2(land_thread);
            // t2.join(); // 等待线程2完成
            // mtx.unlock(); // 解锁互斥锁
            land_thread();
        } else if (choice == 3) {
            ttauav_node::flightByVel::Request message;
            std::cout << "请输入：velN velE velD targetYaw flight_time(ms)" << std::endl;
            std::cin >> message.vel_n >> message.vel_e >> message.vel_d >> message.targetYaw >> message.fly_time ;
            flightByvel(message);
        }
         else if (choice == 4) {
            std::cout << "请输入：targetPitch" << std::endl;
            float32 targetPitch;
            std::cin >> targetPitch;
            gimbalControl(targetPitch);
        }
        else if (choice == 5) {
            break;
        }
        ros::spin();
    }

    // ros::spin();
    
	// return 0;
}