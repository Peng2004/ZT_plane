#include <uavdata_to_odom.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_pub");

  ros::NodeHandle nh;

  uavOdom *uav_to_odom = new uavOdom(nh);
  printf("odom start\n");
  ros::spin();
  

  return 0;
}