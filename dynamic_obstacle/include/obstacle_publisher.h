#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/LinkState.h>
#include <vector>

class DynamicObstacle
{
public:
  DynamicObstacle(ros::NodeHandle* nh);
  
  void obstaclePub(){
    // ROS_INFO("CALL CHECK");
    pub_.publish(right_obstacle_); 
    pub_.publish(left_obstacle_);    
  };

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  
  std::array<double, 2> object_point_;
  gazebo_msgs::LinkState right_obstacle_;
  gazebo_msgs::LinkState left_obstacle_;
  
  void initSub();
  void initPub();
  void subCB(const geometry_msgs::PoseArray::ConstPtr& msg);
};

