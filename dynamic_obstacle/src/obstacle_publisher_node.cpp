#include "obstacle_publisher.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_position");
  ros::NodeHandle nh;
  DynamicObstacle dynamic_obstacle(&nh);
  ros::Rate loop_rate(500);

  while (ros::ok()){
    try{  
    dynamic_obstacle.obstaclePub();
    ros::spinOnce();
    } 
    catch(ros::Exception &e){
      ROS_ERROR("Error occured: %s ", e.what());
    }
    loop_rate.sleep();
  }
  return 0;
}
