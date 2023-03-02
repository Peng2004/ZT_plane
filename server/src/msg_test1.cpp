#include "ros/ros.h"
#include "server/control_msg.h"

void doPerson(const server::control_msg::ConstPtr& p){
    ROS_INFO("message:%d", p->image_status);
    ROS_INFO("goods_x:%d", p->goods_x);
    ROS_INFO("goods_y:%d", p->goods_y);
}

int main(int argc, char *argv[])
{   
    setlocale(LC_ALL,"");

    //1.初始化 ROS 节点
    ros::init(argc,argv,"listener_msg");
    //2.创建 ROS 句柄
    ros::NodeHandle nh;
    //3.创建订阅对象
    ros::Subscriber sub = nh.subscribe<server::control_msg>("communition_node",10,doPerson);

    //4.回调函数中处理 person

    //5.ros::spin();
    ros::spin();    
    return 0;
}