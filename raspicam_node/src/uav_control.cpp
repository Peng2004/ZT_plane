//需要的头文件
#include "uavData.h"
//#include "service.h"
#include "Utils.h"
#include "ttalinkUtils.h"

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include <detection_msgs/AprilTagDetectionArray.h>
#include <detection_msgs/AprilTagDetection.h>

#include "ttauav_node/gimbalControl.h"
#include "ttauav_node/takeoffOrLanding.h"
#include "ttauav_node/flightByOffset.h"
#include "ttauav_node/flightByVel.h"

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ttauav_node/uavdata.h>
#include <detection_msgs/control_msg.h>

#include <chrono>

using namespace std; // 引入std命名空间的所有元素

typedef float float32;

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

五、常用参数
    1、平台高度 0.25m  无人机起飞高度 1.1m
    2、离货架距离 0.8-1m
    3、货架高度：3层——1.4m 2层——
    4、平台起飞：第三层——0度 第二层——-25度


*/

//无人机状态参数
int   uav_status = 0;         //无人机运动状态
float uav_pos[3];             //无人机位置坐标
float uav_yaw;                //无人机自转角度
float uav_gyro_pitch;         //云台pitch角
int img_send_status = 0;      //图像发送完成确认
int img_service_status = 0;   //图像发送权限
float april_pos[2] = {0,0};   //相对位姿
int dock_status = 0;          //调整标志位
// int aprilpos_get_status = 0;  //

int x_adjust_status = 0;
int y_adjust_status = 0;


//参数定义
float shoot_pitch = -90;        //拍摄角度
int recursion_count = 0;        //递归调用计数器
int adjust_num = 3;             //最大调整次数

void uav_pos_callback(const nav_msgs::Odometry::ConstPtr& odom_msg){
    // 提取位置信息
    uav_pos[0] = odom_msg->pose.pose.position.x;
    uav_pos[1] = odom_msg->pose.pose.position.y;
    uav_pos[2] = odom_msg->pose.pose.position.z;
    uav_yaw    = odom_msg->pose.pose.orientation.z;
}

//
//订阅无人机位置
ros::Subscriber uav_pos_sub;
ros::Subscriber img_send_sub;
//订阅AprilTag变换信息
ros::Subscriber april_sub;
//通信话题
ros::Publisher communition_pub;
// 创建一个消息对象
detection_msgs::control_msg comm_msg;

static inline int64_t now()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

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

    ros::Duration(6).sleep();
    // //位置检测
    // float uav_pos_z = uav_pos[2];
    // ROS_INFO("uav_pos_z = %f",uav_pos_z);
    // recursion_count++;
    // if (recursion_count >= adjust_num) {
    //     cout << "起飞次数过多，退出" << endl;
    //     recursion_count = 0;
    //     return;
    // }
    // if(uav_pos_z <= 0.5 && uav_pos_z != 0){
    //     ROS_WARN("uav takeoff failed, take off again");
    //     takeoff();
    // }
    // else if(uav_pos_z == 0){
    //     ROS_WARN("uav_odom error");
    // }
    // recursion_count = 0;
}

void land(){
     // 初始化 rosuav_ctrl_loop_input
    ttalink_rosuav_ctrl_loop_input_t rosuav_ctrl_loop_input = {0};
    rosuav_ctrl_loop_input.flight_ctrl_status = ROS_F_GPS_NAVGATION_ALTI_VEL;
    printf("request:land\n");

    if (m_puavData) {
        // 调用 rosuav_ctrl_loopinput 函数
        ttalinkUtils::rosuav_ctrl_loopinput(m_puavData->getDataPort(), TTALINK_FC_ADDRESS, TTALINK_EMBE_ADDRESS, &rosuav_ctrl_loop_input, 0);
        printf("land success\n");
    }
    else{
        cout << "uavdata start unsuccess" << endl;
    }

}

void flightByvel(float32 vel_n,float32 vel_e,float32 vel_d,float32 targetYaw, float32 fly_time){
     // 初始化 rosuav_ctrl_loop_input
    ttalink_rosuav_ctrl_loop_input_t rosuav_ctrl_loop_input = {0};

    rosuav_ctrl_loop_input.flight_ctrl_status = ROS_F_GPS_POS_VEL_ALTI_VEL;

    rosuav_ctrl_loop_input.velN = vel_n;
    rosuav_ctrl_loop_input.velE = vel_e;
    rosuav_ctrl_loop_input.velD = vel_d;

    rosuav_ctrl_loop_input.atti_yaw = targetYaw;

    rosuav_ctrl_loop_input.param[0] = fly_time;

    ttalinkUtils::rosuav_ctrl_loopinput(m_puavData->getDataPort(), TTALINK_FC_ADDRESS, TTALINK_EMBE_ADDRESS, &rosuav_ctrl_loop_input, 0);
    ros::Duration((fly_time/1000)+1).sleep();

}

void gimbalControl(float targetPitch){
     ttalinkUtils::SendGeneralCommand(m_puavData->getDataPort(), TTALINK_PTZ_ADDRESS, TTALINK_EMBE_ADDRESS
     , 11223344, 0, 109, targetPitch, 0, 0,0,0,0,0);
}

void img_send_sub_callback(const std_msgs::Int32::ConstPtr& msg){
    if(msg->data == 1){
        img_send_status = 1;
        cout << "image send is done" << endl;
    }
    else{
        cout << "communication_img send error" << endl;
    }
}

//无人机图像传输 
void image_control(int img_status ,int goods_x, int goods_y,ros::NodeHandle n){
    //获取图像发送权限
    n.getParam("uav_img_service_status",img_service_status);
    while(ros::ok() && img_service_status != 0) {
        n.getParam("uav_img_service_status",img_service_status);
        ros::spinOnce(); // 检查一次消息队列
    }
    detection_msgs::control_msg comm_msg;
    //发送货架图像
    if (img_status == 1){
        n.setParam("uav_img_status",1);

        comm_msg.image_status = 1;

        comm_msg.goods_x = goods_x;
        comm_msg.goods_y = goods_y;

        communition_pub.publish(comm_msg);

        cout << "access goods_img : "<< goods_x  << "  "<< goods_y << endl;
    }
    //发送AprilTag图像
    else if (img_status == 2){
        n.setParam("uav_img_status",2);

        comm_msg.image_status = 2;
        comm_msg.goods_x = 0;
        comm_msg.goods_y = 0;

        communition_pub.publish(comm_msg);

        cout << "access AprilTag_img " << endl;
    }
    else {
        cout << "access image error" << endl;
    }
    cout << "wait for img_send_status" << endl;

    n.getParam("uav_img_status",img_send_status);

    while(ros::ok() && img_send_status != 0) {
        // img_send_status = 0;
        n.getParam("uav_img_status",img_send_status);
        ros::spinOnce(); // 检查一次消息队列
    }

    // aprilpos_get_status = 1;
    // n.setParam("uav_img_status",0);
}

void april_callback(const detection_msgs::AprilTagDetectionArray::ConstPtr& msg){

    if (msg->detections.size() > 0) {
        detection_msgs::AprilTagDetection aprilDetection = msg->detections[0];

        april_pos[0] = (aprilDetection.pose.pose.pose.position.x);
        april_pos[1] = -(aprilDetection.pose.pose.pose.position.y) + 0.08;//摄像头偏差9cm

        cout << "-april_pos(img/cm):" << (april_pos[0])*100 << "  " << -(april_pos[1] - 0.08)*100 << endl;

        dock_status = 1;
    }
    //  else {
    //     // detections数组为空
        
    //     ROS_WARN("detections array is empty");
    // }
    // if 

    // detection_msgs::AprilTagDetection aprilDetection = msg->detections[0];

    // april_pos[0] = (aprilDetection.pose.pose.pose.position.x);
    // april_pos[1] = -(aprilDetection.pose.pose.pose.position.y) + 0.08;//摄像头偏差9cm

    // cout << "-april_pos(img/cm):" << (april_pos[0])*100 << "  " << -(april_pos[1] - 0.08)*100 << endl;

    // dock_status = 1;
}

//AprilTag空地对接
//x:0    y:0.1
void dock_pos_access(ros::NodeHandle n){
    //1、计算位置偏差 发送图像——》等待位置返回——》计算偏差
    image_control(2,0,0,n);

    int64_t t = now();
    while(ros::ok() && dock_status == 0) {
        int64_t ts = now();
        if (ts - t > 4000){
            cout << "AprilTag recognize timeout" << endl;
            image_control(2,0,0,n);
        }
        if (ts - t > 10000){
            cout << "AprilTag recognize error" << endl;
            break;
        }
        ros::spinOnce(); // 检查一次消息队列
    }
    dock_status = 0;

}

void x_adjustment(){
    float april_pos_x = april_pos[0];
    cout << "x_adjust:" << april_pos_x << endl;
    // 使用if语句来替代switch
    //第一级调整 1m/s 50cm
    if (april_pos_x >= 0.4) {
        flightByvel(0,0.8,0,0,50); // 位置大于或等于0.6
        cout << "第一级调整:右" << endl;
    } 
    else if (april_pos_x <= -0.4) {
        flightByvel(0,-0.8,0,0,50); // 位置小于或等于-0.6
        cout << "第一级调整：左" << endl;
    } 
    //第二级调整 0.3m/s 15cm
    if (april_pos_x >= 0.2 && april_pos_x <0.4) {
        flightByvel(0,0.4,0,0,50); // 位置大于或等于0.6
        cout << "第二级调整:右" << endl;
    } 
    else if (april_pos_x >= -0.4 && april_pos_x <= -0.2) {
        flightByvel(0,-0.4,0,0,50); // 位置小于或等于-0.6
        cout << "第二级调整:左" << endl;
    } 
    //第三级调整 0.2m/s 10cm
    else if (april_pos_x >= 0.1 && april_pos_x < 0.2) {
        flightByvel(0,0.2,0,0,50); // 位置在0.1到0.3
        cout << "第三级调整:右" << endl;
    } 
    else if (april_pos_x > -0.2 && april_pos_x <= -0.1) {
        flightByvel(0,-0.2,0,0,50); // 位置在-0.3到-0.1
        cout << "第三级调整:左" << endl;
    } 
    //第四级调整 0.1m/s 5cm
    else if (april_pos_x > 0.05 && april_pos_x <= 0.1) {
        flightByvel(0,0.12,0,0,50); // 位置在0.05到0.1
        cout << "第四级调整:右" << endl;
    } 
    else if (april_pos_x >= -0.1 && april_pos_x <= -0.05) {
        flightByvel(0,-0.12,0,0,50); // 位置在-0.1到-0.05
        cout << "第四级调整:左" << endl;
    } 
    //误差范围 3cm
    else if (april_pos_x >= -0.05 && april_pos_x <= 0.05) {
        x_adjust_status = 1; // 位置在-0.05到0.05
        cout << "x_adjustment is done:调整完成" << endl;
    }
}

void y_adjustment(){
    float april_pos_y = april_pos[1];
    cout << "y_adjust:" << april_pos_y << endl;
    // 使用if语句来替代switch
    //第一级调整 1m/s 50cm
    if (april_pos_y >= 0.5) {
        flightByvel(1,0,0,0,40); // 位置大于或等于0.6
        cout << "第一级调整:前" << endl;
    } 
    else if (april_pos_y <= -0.5) {
        flightByvel(-1,0,0,0,40); // 位置小于或等于-0.6
        cout << "第一级调整:后" << endl;
    } 
    //第二级调整
    if (april_pos_y >= 0.35 && april_pos_y <0.5) {
        flightByvel(0.6,0,0,0,40); // 位置大于或等于0.6
        cout << "第一级调整:前" << endl;
    } 
    else if (april_pos_y >=-0.5 && april_pos_y <= -0.35) {
        flightByvel(-0.6,0,0,0,40); // 位置小于或等于-0.6
        cout << "第一级调整:后" << endl;
    } 
    //第三级调整 0.3m/s 15cm
    if (april_pos_y >= 0.2 && april_pos_y <0.35) {
        flightByvel(0.4,0,0,0,50); // 位置大于或等于0.6
        cout << "第二级调整:前" << endl;
    } 
    else if (april_pos_y >= -0.35 && april_pos_y <= -0.2) {
        flightByvel(-0.4,0,0,0,50); // 位置小于或等于-0.6
        cout << "第二级调整:后" << endl;
    } 
    //第四级调整 0.2m/s 10cm
    else if (april_pos_y >= 0.08 && april_pos_y < 0.2) {
        flightByvel(0.2,0,0,0,50); // 位置在0.1到0.3
        cout << "第三级调整:前" << endl;
    } 
    else if (april_pos_y > -0.2 && april_pos_y <= -0.08) {
        flightByvel(-0.2,0,0,0,50); // 位置在-0.3到-0.1
        cout << "第三级调整:后" << endl;
    } 
    //第五级调整 0.1m/s 5cm
    else if (april_pos_y > 0.035 && april_pos_y < 0.08) {
        flightByvel(0.11,0,0,0,50); // 位置在0.05到0.1
        cout << "第四级调整:前" << endl;
    } 
    else if (april_pos_y > -0.08 && april_pos_y <= -0.035) {
        flightByvel(-0.11,0,0,0,50); // 位置在-0.1到-0.05
        cout << "第四级调整:后" << endl;
    } 
    //误差范围 4cm
    else if (april_pos_y > -0.035 && april_pos_y < 0.035) {
        y_adjust_status = 1; // 位置在-0.05到0.05
        cout << "y_adjustment is done:调整完成" << endl;
    }
}

void dock_adjustment(ros::NodeHandle n){
    //init
    x_adjust_status = 0;
    y_adjust_status = 0;
    recursion_count++;
    cout << "调整次数：" << recursion_count << endl;

    //第一次调整
    dock_pos_access(n);
    x_adjustment();
    // ros::Duration(0.5).sleep();
    y_adjustment();

    // //第二次调整
    // dock_pos_access(n);
    // x_adjustment();
    // y_adjustment();

    // //第三次调整
    // dock_pos_access(n);
    // x_adjustment();
    // y_adjustment();

    // if (recursion_count >= adjust_num) {
    //     cout << "调整次数过多，退出" << endl;
    //     recursion_count = 0;
    //     return;
    // }
    if (x_adjust_status == 1 && y_adjust_status == 1){
        recursion_count = 0;
        cout << "dock_adjustment is done" << endl;
        land();
        n.setParam("takeoff_status",0);
    }
    else {
        dock_adjustment(n);
    }
    
}

void task1(ros::NodeHandle n){
    int test_point;//断点测试
    // int img_delate ;
    float32 x_time,y_time;

    // cout << "请输入img_delate" << endl;
    // cin >> img_delate;
    cout << "请输入x_time:默认 1750" << endl;
    cin >> x_time;
    cout << "请输入y_time:默认 2100" << endl;
    cin >> y_time;
    cout << "请输入开始指令:1" << endl;
    cin >> test_point;

    //调整云台角度
    ROS_INFO("调整云台角度");
    gimbalControl(-130);
    ros::Duration(0.2).sleep();
    gimbalControl(85);
    ros::Duration(1).sleep();

    // ROS_INFO("开始飞行测试");
    takeoff();
    n.setParam("takeoff_status",1);
    // ROS_INFO("takeoff");

    //前往1-1   2m
    // cin >> test_point;         1m   1.5m 1.2m
    ROS_INFO("前往第一个货架");
    ros::Duration(1).sleep();
    flightByvel(0.3,0,0,0,50);
    ros::Duration(0.5).sleep();
    flightByvel(0,1,0,0,1800);//1800 1250 1500
    // ros::Duration(img_delate).sleep();
    image_control(1,1,1,n);
    // ros::Duration(1).sleep();
    // ROS_INFO("前往第一个货架");

    //前往1-2   1.5m
    // cin >> test_point;
    ROS_INFO("前往第二个货架");
    flightByvel(0,1,0,0,1300);//1300 1000 1100
    // ros::Duration(img_delate).sleep();
    image_control(1,1,2,n);
    ros::Duration(1).sleep();
    // ROS_INFO("前往第二个货架7

    //前往1-3   1.25m
    ROS_INFO("前往第三个货架");
    flightByvel(0,1,0,0,1050);//1050 750 900
    // ros::Duration(img_delate).sleep();
    image_control(1,1,3,n);
    ros::Duration(1).sleep();
    // ROS_INFO("前往第三个货架");

    //前往1-4   1m
    // cin >> test_point;
    ROS_INFO("前往第四个货架");
    flightByvel(0,1,0,0,800);//800 600 700
    // ros::Duration(img_delate).sleep();
    image_control(1,1,4,n);
    // ros::Duration(1).sleep();
    // ROS_INFO("前往第四个货架");

    //前往第二排货架    1.5m__2.6m
    // cin >> test_point;
    ROS_INFO("前往第二排货架");
    flightByvel(0,1,0,0,1100);//1300 750 1100
    ros::Duration(1.5).sleep();
    // cin >> test_point;
    flightByvel(1,0,0,0,2000); 
    ros::Duration(1.5).sleep();

    //前往2-4
    // cin >> test_point;
    ROS_INFO("前往第一个货架");
    flightByvel(0,-1,0,0,1100);//1300 750 1100 
    // ros::Duration(img_delate).sleep();
    image_control(1,2,4,n);
    ros::Duration(1).sleep();
    // ROS_INFO("前往第一个货架");

    //前往2-3
    // cin >> test_point;
    ROS_INFO("前往第二个货架");
    flightByvel(0,-1,0,0,800);//600
    // ros::Duration(img_delate).sleep();
    image_control(1,2,3,n);
    ros::Duration(1).sleep();
    // ROS_INFO("前往第二个货架");

    //前往2-2
    // cin >> test_point;
    ROS_INFO("前往第三个货架");
    flightByvel(0,-1,0,0,1050);//750
    // ros::Duration(img_delate).sleep();
    image_control(1,2,2,n);
    // ros::Duration(1).sleep();
    // ROS_INFO("前往第三个货架");

    //前往2-1
    // cin >> test_point;
    ROS_INFO("前往第四个货架");
    flightByvel(0,-1,0,0,1300);//1000
    // ros::Duration(img_delate).sleep();
    image_control(1,2,1,n);
    // ros::Duration(1).sleep();
    // ROS_INFO("前往第四个货架");


    //回到起点
    // cin >> test_point;
    ROS_INFO("回到起点");
    // flightByvel(0,-1,0,0,1750);//1250
    flightByvel(0,-1,0,0,x_time);
    ros::Duration(1).sleep();
    // cin >> test_point;
    // flightByvel(-1,0,0,0,2100);
    flightByvel(-1,0,0,0,y_time);
    // flightByvel(0,-0.5,0,0,100);
    // flightByvel(-1,0,0,0,100);
    ros::Duration(1).sleep();
    // ROS_INFO("回到起点");
    // cin >> test_point;
    ROS_INFO("降落");
    land();
}

void task2(ros::NodeHandle n){
    int test_point;//断点测试
    float img_delate ;
    float img_pitch;
    float32 x_time,y_time;

    cout << "请输入img_delate:默认 2" << endl;
    cin >> img_delate;
    cout << "请输入pitch角度:默认 28" << endl;
    cin >> img_pitch;
    cout << "请输入x_time:默认 1650" << endl;
    cin >> x_time;
    cout << "请输入y_time:默认 2000" << endl;
    cin >> y_time;
    cout << "请输入开始指令:1" << endl;
    cin >> test_point;
 
    //调整云台角度
    ROS_INFO("调整云台角度");
    gimbalControl(-130);
    ros::Duration(0.2).sleep();
    gimbalControl(90);
    ros::Duration(1).sleep();

    // ROS_INFO("开始飞行测试");
    takeoff();
    n.setParam("takeoff_status",1);
    // ROS_INFO("takeoff");

    //前往1-1   2m
    // cin >> test_point;         1m   1.5m 1.2m
    ROS_INFO("前往第一个货架");
    // cin >> test_point;
    ros::Duration(1.5).sleep();
    // flightByvel(0.3,0,0,0,50);
    // ros::Duration(0.5).sleep();
    flightByvel(0,1,0,0,1800);//1800 1250 1500
    ros::Duration(img_delate).sleep();
    image_control(1,1,1,n);
    gimbalControl(-img_pitch);
    ros::Duration(img_delate).sleep();
    image_control(1,1,5,n);
    gimbalControl(img_pitch);
    // ros::Duration(1).sleep();
    // ROS_INFO("前往第一个货架");

    //前往1-2   1.5m
    ROS_INFO("前往第二个货架");
    // cin >> test_point;
    flightByvel(0,1,0,0,1300);//1300 1000 1100
    ros::Duration(img_delate).sleep();
    image_control(1,1,2,n);
    gimbalControl(-img_pitch);
    ros::Duration(img_delate).sleep();
    image_control(1,1,6,n);
    gimbalControl(img_pitch);
    // ros::Duration(1).sleep();
    // ROS_INFO("前往第二个货架");

    //前往1-3   1.25m
    ROS_INFO("前往第三个货架");
    // cin >> test_point;
    flightByvel(0,1,0,0,1050);//1050 750 900
    ros::Duration(img_delate).sleep();
    image_control(1,1,3,n);
    gimbalControl(-img_pitch);
    ros::Duration(img_delate).sleep();
    image_control(1,1,7,n);
    gimbalControl(img_pitch);
    // ros::Duration(1).sleep();
    // ROS_INFO("前往第三个货架");

    //前往1-4   1m
    ROS_INFO("前往第四个货架");
    // cin >> test_point;
    flightByvel(0,1,0,0,800);//800 600 700
    ros::Duration(img_delate).sleep();
    image_control(1,1,4,n);
    gimbalControl(-img_pitch);
    ros::Duration(img_delate).sleep();
    image_control(1,1,8,n);
    gimbalControl(img_pitch);
    // ros::Duration(1).sleep();
    // ROS_INFO("前往第四个货架");

    //前往第二排货架    1.5m__2.6m
    ROS_INFO("前往第二排货架");
    flightByvel(0,1,0,0,1100);//1300 750 1100  
    ros::Duration(1).sleep();
    flightByvel(1,0,0,0,2300); 
    ros::Duration(0.5).sleep();

    //准备对接
    n.setParam("dock_status",1);
    n.setParam("takeoff_status",0);
    ROS_INFO("准备开始对接");
    // cin >> test_point;
    gimbalControl(-90);
    ros::Duration(0.5).sleep();
    dock_adjustment(n);

    //重新起飞
    ros::Duration(8).sleep();
    ROS_INFO("准备再次起飞");
    takeoff();
    n.setParam("dock_status",0);
    n.setParam("takeoff_status",1);
    gimbalControl(90);
    //往后微调10cm
    flightByvel(-0.3,0,0,0,50);//1300 750 1100 
    ros::Duration(2).sleep();

    //前往2-4
    ROS_INFO("前往第一个货架");
    flightByvel(0,-1,0,0,1050);//1300 750 1100 
    ros::Duration(img_delate).sleep();
    image_control(1,2,4,n);
    gimbalControl(-img_pitch);
    ros::Duration(img_delate).sleep();
    image_control(1,2,8,n);
    gimbalControl(img_pitch);
    // ros::Duration(1).sleep();
    // ROS_INFO("前往第一个货架");

    //前往2-3
    ROS_INFO("前往第二个货架");
    // cin >> test_point;
    flightByvel(0,-1,0,0,800);//600
    ros::Duration(img_delate).sleep();
    image_control(1,2,3,n);
    gimbalControl(-img_pitch);
    ros::Duration(img_delate).sleep();
    image_control(1,2,7,n);
    gimbalControl(img_pitch);
    // ros::Duration(1).sleep();
    // ROS_INFO("前往第二个货架");

    //前往2-2
    ROS_INFO("前往第三个货架");
    // cin >> test_point;
    flightByvel(0,-1,0,0,1050);//750
    ros::Duration(img_delate).sleep();
    image_control(1,2,2,n);
    gimbalControl(-img_pitch);
    ros::Duration(img_delate).sleep();
    image_control(1,2,6,n);
    gimbalControl(img_pitch);
    // ros::Duration(1).sleep();
    // ROS_INFO("前往第三个货架");

    //前往2-1
    ROS_INFO("前往第四个货架");
    // cin >> test_point;
    flightByvel(0,-1,0,0,1300);//1000
    ros::Duration(img_delate).sleep();
    image_control(1,2,1,n);
    gimbalControl(-img_pitch);
    ros::Duration(img_delate).sleep();
    image_control(1,2,5,n);
    gimbalControl(img_pitch);
    // ros::Duration(1).sleep();
    // ROS_INFO("前往第四个货架");


    //回到起点
    ROS_INFO("回到起点");
    // cin >> test_point;
    // flightByvel(0,-1,0,0,1750);//1250
    flightByvel(0,-1,0,0,x_time);
    ros::Duration(1).sleep();
    // cin >> test_point;
    // flightByvel(-1,0,0,0,2200);
    flightByvel(-1,0,0,0,y_time);
    // flightByvel(0,-0.5,0,0,100);
    // flightByvel(-1,0,0,0,100);
    ros::Duration(1).sleep();
    // ROS_INFO("回到起点");
    // cin >> test_point;
    ROS_INFO("降落");
    // cin >> test_point;
    land();
}


int main(int argc, char** argv) {
    //设置编码
    setlocale(LC_ALL,"");

    ros::init(argc, argv, "uav_control");

    ros::NodeHandle n;

    m_puavData = new uavData(47142);
    m_puavData->start();

    //无人机状态初始化
    uav_status = 0;
    uav_gyro_pitch = 0;

    //订阅无人机位置
    // uav_pos_sub = n.subscribe<nav_msgs::Odometry>("uav_odom", 10 , uav_pos_callback);
    //AprilTag_tf
    april_sub = n.subscribe<detection_msgs::AprilTagDetectionArray>("/tag_detections",10,april_callback);
    //通信话题
    communition_pub = n.advertise<detection_msgs::control_msg>("communication_uav",10);
    cout << "communition_node is start" << endl;

    ros::Rate loop_rate(10); // 每10秒循环一次

    int task_choice = 0;
    cout << "请输入:1——task1 2——task2 3——按键控制" << endl;
    cin >> task_choice;
    switch (task_choice) {
        case 1:
            cout << "task1 start" << endl;
            task1(n);
            cout << "task1 done" << endl;
            break;
        case 2:
            cout << "task2 start" << endl;
            task2(n);
            cout << "task2 done" << endl;
            break;
        default:
            cout << "按键控制" << endl;
            break;   
    }

    while(ros::ok()){ 
        static int choice = 0;
        cout << "按键控制：请输入数字来选择" << endl;
        cout << "0-状态检查, 1-takeoff, 2-land, 3-flight, 4-gimbal, 5-quit, 6-dock_access, 7-x_adjust, 8-y_adjust, 9-image" << endl;
        
        cin >> choice;
        switch (choice) {
        case 0:
            ros::Duration(2).sleep(); // 等待5秒以确保起飞完成
            takeoff();
            gimbalControl(-90);
            dock_pos_access(n);
            flightByvel(0,1,0,0,800);
            break;
        case 1:
            takeoff();
            // ros::Duration(5).sleep(); // 等待5秒以确保起飞完成
            break;
        case 2:
            land();
            break;
        case 3:
            {
                float32 vel_n, vel_e, vel_d, targetYaw, fly_time;
                std::cout << "请输入:velN velE velD targetYaw flight_time(ms)" << std::endl;

                if (!(std::cin >> vel_n >> vel_e >> vel_d >> targetYaw >> fly_time)) {
                    std::cout << "输入无效，请确保输入数值并遵循正确的格式。" << std::endl;
                    break;
                }

                if (fly_time < 49) {
                    std::cout << "飞行时间应>49" << std::endl;
                    break;
                }
                flightByvel(vel_n, vel_e, vel_d, targetYaw, fly_time);
            }
            break;
        case 4:
            {
                std::cout << "请输入：targetPitch" << std::endl;
                float32 targetPitch;
                std::cin >> targetPitch;
                gimbalControl(targetPitch);
            }
            break;
        case 5:
            ros::spin();
            break; // 退出循环
        case 6:
            dock_pos_access(n);
            break;
        case 7:
            x_adjustment();
            break;
        case 8:
            y_adjustment();
            break;
        case 9:
            int row,num;
            cout << "请输入:row,num" << endl;
            cin >> row >> num;
            image_control(1,row,num,n);
            break;
        case 10:
            dock_adjustment(n);
            break;
        default:
            std::cout << "无效的输入" << std::endl;
            break;
        }
        loop_rate.sleep(); // 让节点休眠一段时间，直到达到10Hz的频率
        ros::spinOnce();
    }
    //起飞

    //ros::spinOnce();

	return 0;

}