#include "ros/ros.h"
#include "server/control_msg.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    //1.初始化 ROS 节点
    ros::init(argc,argv,"talker_msg");

    //2.创建 ROS 句柄
    ros::NodeHandle nh;

    //3.创建发布者对象
    ros::Publisher pub = nh.advertise<server::control_msg>("chatter_control",10);

    //4.组织被发布的消息，编写发布逻辑并发布消息
    server::control_msg p;
    p.message = 1;
    p.goods_x = 2;
    p.goods_y = 3;
    
    ros::Rate r(1);
    while (ros::ok())
    {
        pub.publish(p);
        ROS_INFO("p.message:%d", p.message);

        r.sleep();
        ros::spinOnce();
    }



    return 0;
}