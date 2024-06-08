#include <ros/ros.h>
#include <gazebo_msgs/LinkState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_position");
  ros::NodeHandle n;

  ros::Publisher obstacle_publisher = n.advertise<gazebo_msgs::LinkState>("/gazebo/set_link_state", 1000);
  ros::Rate loop_rate(500);

  gazebo_msgs::LinkState link_state;
  link_state.link_name = "obstacle_right";

  int count = 0;
  ros::Time update_time;

  while (ros::ok())
  { 

    update_time = ros::Time::now();

    // Set pose
    link_state.pose.position.x = 0.5 * std::cos(M_PI / 5.0 * update_time.toSec());
    link_state.pose.position.y = 0.5 * std::sin(M_PI / 5.0 * update_time.toSec());
    link_state.pose.position.z = 1.0;
    link_state.pose.orientation.x = 0.0;
    link_state.pose.orientation.y = 0.0;
    link_state.pose.orientation.z = 0.0;
    link_state.pose.orientation.w = 1.0;

    // Set twist
    link_state.twist.linear.x = 0.0;
    link_state.twist.linear.y = 0.0;
    link_state.twist.linear.z = 0.0;
    link_state.twist.angular.x = 0.0;
    link_state.twist.angular.y = 0.0;
    link_state.twist.angular.z = 0.0;

    obstacle_publisher.publish(link_state);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
