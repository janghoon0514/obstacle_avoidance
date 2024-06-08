#include "obstacle_publisher.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_position");
  ros::NodeHandle nh;
  while (ros::ok()){
    try{
    DynamicObstacle dynamic_obstacle(&nh);
    dynamic_obstacle.obstaclePub();
    ros::Rate loop_rate(500);
    ros::spin();
    } 
    catch(ros::Exception &e){
      ROS_ERROR("Error occured: %s ", e.what());
    }
  }
  return 0;
}
