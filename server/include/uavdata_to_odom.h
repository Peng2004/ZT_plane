#ifndef UAVDATA_TO_ODOM_H
#define UAVDATA_TO_ODOM_H

#include "ros/ros.h"
#include "ros/time.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include "ttauav_node/uavdata.h"

struct Piont
{
    Eigen::Vector3d pos;//位置
    Eigen::Matrix3d orien;//姿态 旋转矩阵表示
    Eigen::Vector3d w;//角速度
    Eigen::Vector3d v;//线速度
};

Eigen::MatrixXd X = Eigen::MatrixXd(3,1); //创建一个3*1矩阵
 
using namespace std;
 
class kalman_filter
{
private:
    Eigen::MatrixXd A; //系统状态矩阵
    Eigen::MatrixXd P; //协方差
    Eigen::MatrixXd Q; //测量过程噪音（预测）
    Eigen::MatrixXd R; //真实传感器噪音
    Eigen::MatrixXd H; //测量矩阵
 
    bool isinitized = false; //判断是否进行了初始化
 
public:
    kalman_filter();
    Eigen::MatrixXd predictAndupdate( Eigen::MatrixXd x,Eigen::MatrixXd z);
    ~kalman_filter();
};
 
Eigen::MatrixXd kalman_filter::predictAndupdate(Eigen::MatrixXd x,Eigen::MatrixXd z)
{
    if(!isinitized)
    {
        P = Eigen::MatrixXd(3,3); //协方差 3*3矩阵
        P<< 1,0,0,
            0,1,0,
            0,0,1; //协方差的初始化
        isinitized=true;
    }
    x = A*x; // 状态一步预测方程
    P = A*P*(A.transpose())+Q; //一步预测协方差阵
    Eigen::MatrixXd  K = P*(H.transpose())*((H*P*(H.transpose())+R).inverse()); //kalman增益
    x = x+K*(z-H*x); //状态更新：
    int x_size = x.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
    P = (I-K*H)*P; // 协方差阵更新：
    return x;
}
 
kalman_filter::kalman_filter()
{
//参数初始化设置
    A=Eigen::MatrixXd(3,3); //系统状态矩阵
    A<< 1,0,0,
        0,1,0,
        0,0,1;
    H=Eigen::MatrixXd(3,3); //测量矩阵
    H<< 1,0,0,
        0,1,0,
        0,0,1;
    Q=Eigen::MatrixXd(3,3); //（预测）过程噪音
    Q<< 0.03,0,0,
        0,0.03,0,
        0,0,0.03;
    R=Eigen::MatrixXd(3,3); //真实传感器噪音
    R<< 3.65,0,0,            //R太大，卡尔曼滤波响应会变慢
        0,3.65,0,
        0,0,3.65;     
}
 
kalman_filter::~kalman_filter(){}


class uavOdom
{
private:
    ros::NodeHandle nhi;
    ros::Subscriber uavsub;
    ros::Publisher odompub;
    nav_msgs::Odometry odom;
    ros::Time time;
    Piont point;
    Eigen::Vector3d gravity;

    double deltaT;
    bool firstT;

    //uavdata->geometry_msgs::Vector3
    geometry_msgs::Vector3 linear_acc;
    geometry_msgs::Vector3 linear_vel;
    geometry_msgs::Vector3 angular_vel;

public:
    uavOdom(ros::NodeHandle& nh);
    ~uavOdom();

    void uavCallback(const ttauav_node::uavdata::ConstPtr& msg);
    void setGravity(const geometry_msgs::Vector3 &msg);
    void calcPosition(const geometry_msgs::Vector3 &msg);
    void calcOrientation(const geometry_msgs::Vector3 &msg);
    void updateodom(const Piont point);
    Eigen::Vector3d RC_fliter(Eigen::Vector3d vel);

};

//RC低通滤波
Eigen::Vector3d uavOdom::RC_fliter(Eigen::Vector3d vel){
  //获取新数据
  Eigen::Vector3d newData = vel;
  static Eigen::Vector3d oldData(0,0,0);
  //计算比例系数a
  double a = deltaT / (deltaT + 1 / (2 * 3.14159 * 30));//截止频率为10Hz
  std::cout << "RC比例系数:" << a << std::endl;
  //RC滤波
  Eigen::Vector3d vel_RC = oldData * (1 - a) + newData * a;
  //更新数据
  oldData = vel;
  return  vel_RC;
}

uavOdom::uavOdom(ros::NodeHandle& nh):nhi(nh)
{
    //参数初始化
  printf("uav_odom start\n");
  uavsub = nhi.subscribe("/uavdata",30 ,&uavOdom::uavCallback,this);
  ros::Rate loop_rate(150);

  odompub = nhi.advertise<nav_msgs::Odometry>("uav_odom", 30);
  
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  Eigen::Vector3d zero(0, 0, 0);
  point.pos = zero;
  point.orien = Eigen::Matrix3d::Identity();
  point.v = zero;
  point.w = zero;
  firstT = true;
}

void uavOdom::uavCallback(const ttauav_node::uavdata::ConstPtr& msg)
{
    //uavdata
    linear_vel.x = msg->velN;
    linear_vel.y = msg->velE;
    linear_vel.z = msg->velD;
    linear_acc.x = msg->accN;
    linear_acc.y = msg->accE;
    linear_acc.z = msg->accD;
    angular_vel.x = msg->gyro_pitch;
    angular_vel.y = msg->gyro_roll;
    angular_vel.z = msg->gyro_yaw;
    odom.twist.twist.angular.x = msg->atti_pitch;
    odom.twist.twist.angular.y = msg->atti_roll;
    odom.twist.twist.angular.z = msg->atti_yaw;

    printf("l_vel=%f,%f,%f\n",linear_vel.x,linear_vel.y,linear_vel.z);

    if(firstT){
        time = msg->header.stamp;
        deltaT = 0;

        setGravity(linear_acc);//这里只是粗略的拿第一帧数据作为当前的重力即速度，可以变成用前几帧的数据估计出一个g
        printf("Gravity set to: x: %f, y: %f, z: %f\n",gravity[0],gravity[1],gravity[2]);
        firstT = false;
    } else {
        deltaT = (msg->header.stamp - time).toSec();//当前帧和上一帧的时间差
        time = msg->header.stamp;   
        odom.header.seq = msg->header.seq;
        odom.header.stamp = msg->header.stamp;
        calcOrientation(angular_vel);//计算角度，四元数表示
        calcPosition(linear_vel);//计算位置
        updateodom(point);
    }
}

void uavOdom::setGravity(const geometry_msgs::Vector3 &msg) {
  gravity[0] = msg.x;
  gravity[1] = msg.y;
  gravity[2] = msg.z;
  
}

void uavOdom::calcOrientation(const geometry_msgs::Vector3 &msg) {
  point.w << msg.x, msg.y, msg.z;
  //基于旋转矩阵表示方法
  Eigen::Matrix3d B;
  B << 0, -msg.z * deltaT, msg.y * deltaT, 
       msg.z * deltaT, 0, -msg.x * deltaT,
      -msg.y * deltaT, msg.x * deltaT, 0;
  //欧拉法
  double sigma =
      std::sqrt(std::pow(msg.x, 2) + std::pow(msg.y, 2) + std::pow(msg.z, 2)) *
      deltaT;

  //罗德里格斯公式
  point.orien = point.orien *
               (Eigen::Matrix3d::Identity() + (std::sin(sigma) / sigma) * B -
                ((1 - std::cos(sigma)) / std::pow(sigma, 2)) * B * B);
//   printf("sigma is: %lf\n", sigma);
}

void uavOdom::calcPosition(const geometry_msgs::Vector3 &msg) {
//   Eigen::Vector3d acc_l(msg.x, msg.y, msg.z);//imu坐标系下的加速度
//   Eigen::Vector3d acc_g = point.orien * acc_l;//转化到里程计坐标系下的加速度
//   // Eigen::Vector3d acc(msg.x - gravity[0], msg.y - gravity[1], msg.z -
//   // gravity[2]);
//   point.v = point.v + deltaT * (acc_g - gravity);//积分得到速度
//   point.pos = point.pos + deltaT * point.v;//积分得到位置
    Eigen::Vector3d vel(msg.x, msg.y, msg.z);
    if(deltaT!=0){
        // 计算平方和
        double sumOfSquares = 0;
        for (int i = 0; i < 3; ++i) {
            sumOfSquares += vel[i] * vel[i];
        }
        if(sumOfSquares >= 0.0001){
            point.pos = point.pos + deltaT * vel;
        }
    }
    // point.pos = point.pos + deltaT * vel;
    point.v = vel;
}

void uavOdom::updateodom(const Piont point) {
    //位置
    odom.pose.pose.position.x = point.pos(0);
    odom.pose.pose.position.y = point.pos(1);
    odom.pose.pose.position.z = point.pos(2);
    //姿态 四元数
    odom.pose.pose.orientation.x = (point.orien(2,1) - point.orien(1,2)) / 4; 
    odom.pose.pose.orientation.y = (point.orien(0,2) - point.orien(2,0)) / 4;
    odom.pose.pose.orientation.z = (point.orien(1,0) - point.orien(0,1)) / 4;
    // odom.pose.pose.orientation.w = std::sqrt(1 + point.orien(0,0) + point.orien(1,1) + point.orien(2,2)) / 2;
    //线速度
    odom.twist.twist.linear.x = point.v(0);
    odom.twist.twist.linear.y = point.v(1);
    odom.twist.twist.linear.z = point.v(2);

    // //角速度
    // odom.twist.twist.angular.x = point.w(0);
    // odom.twist.twist.angular.y = point.w(1);
    // odom.twist.twist.angular.z = point.w(2);
    //发布里程计
    odompub.publish(odom);
    // printf("linear:x=%f ,y=%f ,z=%f\n",point.v(0),point.v(1),point.v(2));

}

#endif

