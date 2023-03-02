#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/Imu.h>
#include <ttauav_node/uavdata.h>


ros::Publisher imu_pub;

void uavDataCallback(const ttauav_node::uavdata::ConstPtr& msg)
{
    // 创建一个新的IMU消息
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "imu_link";

    // 填充IMU数据
    imu_msg.angular_velocity.x = msg->gyro_pitch;
    imu_msg.angular_velocity.y = msg->gyro_roll;
    imu_msg.angular_velocity.z = msg->gyro_yaw;

    imu_msg.linear_acceleration.x = msg->accN;
    imu_msg.linear_acceleration.y = msg->accE;
    imu_msg.linear_acceleration.z = msg->accD;

    // 发布IMU消息
    imu_pub.publish(imu_msg);
    printf("imu sending\n");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_pub");
    ros::NodeHandle nh;

    // 订阅"/uavdata"话题
    ros::Subscriber uav_data_sub = nh.subscribe("/uavdata", 100, uavDataCallback);
    ros::Rate loop_rate(30);

    // 发布"/imu"话题
    imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 100);

    ros::spin();

    return 0;
}
