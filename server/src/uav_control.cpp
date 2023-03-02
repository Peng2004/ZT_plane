//需要的头文件
#include "uavData.h"
//#include "service.h"
#include "Utils.h"
#include "ttalinkUtils.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"

#include "ttauav_node/gimbalControl.h"
#include "ttauav_node/takeoffOrLanding.h"
#include "ttauav_node/flightByOffset.h"
#include "ttauav_node/flightByVel.h"

#include <eigen3/Eigen/Dense>
#include <cmath>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ttauav_node/uavdata.h>
#include <server/control_msg.h>

using namespace std; // 引入std命名空间的所有元素

uavData *m_puavData = NULL;
/*
一、无人机任务流程（预选赛：无人机扫描顶部货架）：
    1、里程计初始化
    2、起飞，确认高度 ———> 向无人车发送起飞信息（脱离后即发送/采用延时）
    3、调整至目标高度 
    4、按照预设轨迹运动至第一货架顶部最左侧，获取里程计位置，进行微调（角度不正确，可能导致较大的累积误差）
    5、打包无人机位置与当前图像数据发送给PC端（UDP/话题）
    6、循环上述步骤，完成两层货架的扫描与图像数据包发送。————> 向无人车发送任务完成信息
    7、运动至出发点，降落。

二、预选赛预案：
    1、针对误识别后面货架问题：可以获取斜视图像
    2、针对运动累积误差问题，可以扫描完第一排货架后，扫描无人车二维码，对里程计进行修正

三、细节问题：
    1、无人机初始位置并不确定，如何保证比赛现场起飞地点的精度问题，还有与无人车的初始位置差（使用卷尺测量固定起点）
    2、无人机初始角度微差，可能导致偏离预设轨迹，可以尝试使用apriltag微调（精度够的话）

四、代码框架：
    1、飞行控制：直接调用天途的API（要有动作结束反馈）
    2、位置获取：/uav_odom话题，主要校准位置和自转角度
    3、图像获取：解除图传节点的堵塞（发送阻塞），注意位置和图像时间戳的同步
    4、起飞状态确认，和无人车建立双向通信话题“/message”


*/

//无人机状态参数
int   uav_status = 0;         //无人机运动状态
float uav_pos[3];             //无人机位置坐标
float uav_yaw;                //无人机自转角度
float uav_gyro_pitch;         //云台pitch角

//参数定义
float shoot_pitch = -90;        //拍摄角度

void uav_pos_callback(const nav_msgs::Odometry::ConstPtr& odom_msg){
    uav_pos[0] = odom_msg.pose.pose.position.x;
    uav_pos[1] = odom_msg.pose.pose.position.y;
    uav_pos[2] = odom_msg.pose.pose.position.z;
    uav_yaw = odom.pose.pose.orientation.z;
}

//
//订阅无人机位置
ros::Subscriber uav_pos_sub
//通信话题
ros::Publisher commuition_pub
// 创建一个消息对象
std_msgs::String comm_msg;


//无人机运动控制
void takeoff() {
    // 初始化 rosuav_ctrl_loop_input
    ttalink_rosuav_ctrl_loop_input_t rosuav_ctrl_loop_input = {0};
    rosuav_ctrl_loop_input.flight_ctrl_status = ROS_F_GPS_AUTO_TAKE_OFF;
    printf("request:takeoff\n");

    if (m_puavData) {
        // 调用 rosuav_ctrl_loopinput 函数
        ttalinkUtils::rosuav_ctrl_loopinput(m_puavData->getDataPort(), TTALINK_FC_ADDRESS, TTALINK_EMBE_ADDRESS, &rosuav_ctrl_loop_input, 0);
        printf("takeoff success\n");
    }
    else{
        cout << "uavdata start unsuccess" << endl;
    }

}

void land(){
     // 初始化 rosuav_ctrl_loop_input
    ttalink_rosuav_ctrl_loop_input_t rosuav_ctrl_loop_input = {0};
    rosuav_ctrl_loop_input.flight_ctrl_status = ROS_F_GPS_NAVGATION_ALTI_VEL;
    printf("request:land\n");

    if (m_puavData) {
        // 调用 rosuav_ctrl_loopinput 函数
        ttalinkUtils::rosuav_ctrl_loopinput(m_puavData->getDataPort(), TTALINK_FC_ADDRESS, TTALINK_EMBE_ADDRESS, &rosuav_ctrl_loop_input, 0);
        printf("takeoff success\n");
    }
    else{
        cout << "uavdata start unsuccess" << endl;
    }
}

void flightByvel(float vel_n,float vel_e,float vel_d,float targetYaw, float fly_time){
     // 初始化 rosuav_ctrl_loop_input
    ttalink_rosuav_ctrl_loop_input_t rosuav_ctrl_loop_input = {0};

    rosuav_ctrl_loop_input.flight_ctrl_status = ROS_F_GPS_POS_VEL_ALTI_VEL;

    rosuav_ctrl_loop_input.velN = vel_n;
    rosuav_ctrl_loop_input.velE = vel_e;
    rosuav_ctrl_loop_input.velD = vel_d;

    rosuav_ctrl_loop_input.atti_yaw = targetYaw;

    rosuav_ctrl_loop_input.param[0] = fly_time;

    ttalinkUtils::rosuav_ctrl_loopinput(m_puavData->getDataPort(), TTALINK_FC_ADDRESS, TTALINK_EMBE_ADDRESS, &rosuav_ctrl_loop_input, 0);
}

void gimbalControl(float targetPitch){
     ttalinkUtils::SendGeneralCommand(m_puavData->getDataPort(), TTALINK_PTZ_ADDRESS, TTALINK_EMBE_ADDRESS
     , 11223344, 0, 109, targetPitch, 0, 0,0,0,0,0);
}


//无人机图像传输（pos和图像数据一起打包）、
void image_control(int goods_x, int goods_y){
    comm_msg.image_status = 1;
    comm_msg.goods_x = goods_x;
    comm_msg.goods_y = goods_y;
    commuition_pub.publish(comm_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "service");
    ros::NodeHandle n;

    m_puavData = new uavData(47100);
    m_puavData->start();

    //无人机状态初始化
    uav_status = 0;
    uav_gyro_pitch = 0;

    //订阅无人机位置
    uav_pos_sub = n.subscribe("uav_odom", 10 , uav_pos_callback);

    //通信话题
    commuition_pub = n.advertise<std_msgs::String>("communition_node",10);

    while(ros::ok()){ 
        if (uav_status == 0){
            //takeoff
            takeoff();
            cout << "uav is takeoff success" << endl;
            uav_status = 1;
            comm_msg.message = 1;
            commuition_pub.publish(comm_msg);
            //前往第一个货架:（运动控制+矫正）
            flightByVel(1 0 0 0 500);
            //调整拍摄角度
            gimbalControl(shoot_pitch-uav_gyro_pitch);
            cout << "gimbal control success" << endl;
            uav_gyro_pitch = shoot_pitch;
            cout << "pitach is :" << uav_gyro_pitch << endl;
            //传输图像
            image_control(1,1);
            cout << "image is send" << endl;
            ros::delay_time(2000);
            //降落
            land();
            cout << "uav is landing" << endl;

        } 
        ros::spin();
    }
    //起飞

    //ros::spinOnce();

	return 0;
}