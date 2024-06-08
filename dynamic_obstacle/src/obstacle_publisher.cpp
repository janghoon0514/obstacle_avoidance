#include "obstacle_publisher.h"

DynamicObstacle::DynamicObstacle(ros::NodeHandle* nh):nh_(*nh)
{
  initSub();
  initPub();
  std::fill(std::begin(object_point_), std::end(object_point_), 0);
  right_obstacle_.link_name = "obstacle_right";
  left_obstacle_.link_name = "obstacle_left";

  // Set pose
  right_obstacle_.pose.position.x = 0.0; 
  right_obstacle_.pose.position.y = 0.0;
  right_obstacle_.pose.position.z = 2.0;
  right_obstacle_.pose.orientation.x = 0.0;
  right_obstacle_.pose.orientation.y = 0.0;
  right_obstacle_.pose.orientation.z = 0.0;
  right_obstacle_.pose.orientation.w = 1.0;

  // Set pose
  left_obstacle_.pose.position.x = 0.0;
  left_obstacle_.pose.position.y = 0.0;
  left_obstacle_.pose.position.z = 3.0;
  left_obstacle_.pose.orientation.x = 0.0;
  left_obstacle_.pose.orientation.y = 0.0;
  left_obstacle_.pose.orientation.z = 0.0;
  left_obstacle_.pose.orientation.w = 1.0;
}

void DynamicObstacle::initSub()
{
  sub_ = nh_.subscribe("/obstacle/points", 10, &DynamicObstacle::subCB, this);
}

void DynamicObstacle::initPub()
{
  pub_ = nh_.advertise<gazebo_msgs::LinkState>("/gazebo/set_link_state", 100);
}

void DynamicObstacle::subCB(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    // Callback function to handle incoming PoseArray messages
    right_obstacle_.pose.position.x = msg->poses[0].position.y;
    right_obstacle_.pose.position.y = msg->poses[0].position.x;
    right_obstacle_.pose.position.z = msg->poses[0].position.z;

    left_obstacle_.pose.position.x = msg->poses[1].position.y;
    left_obstacle_.pose.position.y = msg->poses[1].position.x;
    left_obstacle_.pose.position.z = msg->poses[1].position.z;  
}
