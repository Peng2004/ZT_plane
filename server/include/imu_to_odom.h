#ifndef IMU_TO_ODOM_IMU_TO_ODOM_HPP
#define IMU_TO_ODOM_IMU_TO_ODOM_HPP

// ROS includes
#include "ros/ros.h"
#include "ros/time.h"
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

struct Piont
{
    Eigen::Vector3d pos;//位置
    Eigen::Matrix3d orien;//姿态 旋转矩阵表示
    Eigen::Vector3d w;//角速度
    Eigen::Vector3d v;//线速度
};

//imu处理
class ImuOdom
{
private:
    ros::NodeHandle nhi;
    ros::Subscriber imusub;
    ros::Publisher odompub;
    nav_msgs::Odometry odom;
    ros::Time time;
    Piont point;
    Eigen::Vector3d gravity;

    double deltaT;
    bool firstT;
    
public:
    //! Constructor.
    ImuOdom(ros::NodeHandle& nh);
    //! Destructor.
    ~ImuOdom();

    void ImuCallback(const sensor_msgs::Imu &msg);
    void setGravity(const geometry_msgs::Vector3 &msg);
    void calcPosition(const geometry_msgs::Vector3 &msg);
    void calcOrientation(const geometry_msgs::Vector3 &msg);
    void updateodom(const Piont point);

};

ImuOdom::ImuOdom(ros::NodeHandle& nh):nhi(nh) {
  //参数初始化
  printf("imu_odom start\n");
  imusub = nhi.subscribe("/imu", 32, &ImuOdom::ImuCallback, this);
  ros::Rate loop_rate(30);
  
  odompub = nhi.advertise<nav_msgs::Odometry>("imu_odom", 32);
  
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  Eigen::Vector3d zero(0, 0, 0);
  point.pos = zero;
  point.orien = Eigen::Matrix3d::Identity();
  point.v = zero;
  point.w = zero;
  firstT = true;
}

void ImuOdom::ImuCallback(const sensor_msgs::Imu &msg) {
  if (firstT) {
    time = msg.header.stamp;
    deltaT = 0;
    setGravity(msg.linear_acceleration);//这里只是粗略的拿第一帧数据作为当前的重力即速度，可以变成用前几帧的数据估计出一个g
    printf("Gravity set to: x: %f, y: %f, z: %f\n",gravity[0],gravity[1],gravity[2]);
    firstT = false;
  } else {
    deltaT = (msg.header.stamp - time).toSec();//当前帧和上一帧的时间差
    time = msg.header.stamp;   
    odom.header.seq = msg.header.seq;
    odom.header.stamp = msg.header.stamp;
    calcOrientation(msg.angular_velocity);//计算角度，四元数表示
    calcPosition(msg.linear_acceleration);//计算位置
    updateodom(point);
  }
  // std::cout << pose.pos << std::endl;
}

void ImuOdom::setGravity(const geometry_msgs::Vector3 &msg) {
  gravity[0] = msg.x;
  gravity[1] = msg.y;
  gravity[2] = msg.z;
  
}

void ImuOdom::calcOrientation(const geometry_msgs::Vector3 &msg) {
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
  // //  
  // std::cout << "sigma: " << sigma << std::endl;
  // Matrix3d result = Matrix3d::Identity() + (sin(sigma) / sigma) * B;
  // std::cout << result << std::endl;
  // // 假设 pose.orien 是一个已经定义的 Matrix3d 类型的变量
  // std::cout << "pose orientation: " << pose.orien << std::endl;
  // //
  // std::cout << "sigma: " << sigma << std::endl << Eigen::Matrix3d::Identity()
  // + (std::sin(sigma) / sigma) * B << std::endl << pose.orien << std::endl;

  //罗德里格斯公式
  point.orien = point.orien *
               (Eigen::Matrix3d::Identity() + (std::sin(sigma) / sigma) * B -
                ((1 - std::cos(sigma)) / std::pow(sigma, 2)) * B * B);
  printf("sigma is: %lf\n", sigma);
}

void ImuOdom::calcPosition(const geometry_msgs::Vector3 &msg) {
  Eigen::Vector3d acc_l(msg.x, msg.y, msg.z);//imu坐标系下的加速度
  Eigen::Vector3d acc_g = point.orien * acc_l;//转化到里程计坐标系下的加速度
  // Eigen::Vector3d acc(msg.x - gravity[0], msg.y - gravity[1], msg.z -
  // gravity[2]);
  point.v = point.v + deltaT * (acc_g - gravity);//积分得到速度
  point.pos = point.pos + deltaT * point.v;//积分得到位置

}

void ImuOdom::updateodom(const Piont point) {
  //位置
  odom.pose.pose.position.x = point.pos(0);
  odom.pose.pose.position.y = point.pos(1);
  odom.pose.pose.position.z = point.pos(2);
  //姿态 四元数
  odom.pose.pose.orientation.x = (point.orien(2,1) - point.orien(1,2)) / 4; 
  odom.pose.pose.orientation.y = (point.orien(0,2) - point.orien(2,0)) / 4;
  odom.pose.pose.orientation.z = (point.orien(1,0) - point.orien(0,1)) / 4;
  odom.pose.pose.orientation.w = std::sqrt(1 + point.orien(0,0) + point.orien(1,1) + point.orien(2,2)) / 2;
  //线速度
  odom.twist.twist.linear.x = point.v(0);
  odom.twist.twist.linear.y = point.v(1);
  odom.twist.twist.linear.z = point.v(2);

  //角速度
  odom.twist.twist.angular.x = point.w(0);
  odom.twist.twist.angular.y = point.w(1);
  odom.twist.twist.angular.z = point.w(2);
  //发布里程计
  odompub.publish(odom);
  // printf("linear:x=%f ,y=%f ,z=%f\n",point.v(0),point.v(1),point.v(2));

}




#endif