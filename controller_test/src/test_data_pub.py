#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_msgs.msg import Header
import tf.transformations as tf
import numpy as np

class PosePublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pose_publisher', anonymous=True)

        # Create publishers for the Pose and PoseArray messages
        self.pose_publisher = rospy.Publisher('/target_pose', Pose, queue_size=10)

        # Set the publishing rate

    def publish_pose(self):
        # Create a Pose message
        pose_msg = Pose()
        # Set the position (example values)
        pose_msg.position.x = 0.4
        pose_msg.position.y = 0.4
        pose_msg.position.z = 0.3

        # Set the orientation (example quaternion values)
        quaternion = tf.quaternion_from_euler(np.pi, 0, 0)  # Example: Roll, Pitch, Yaw
        pose_msg.orientation.x = quaternion[0]
        pose_msg.orientation.y = quaternion[1]
        pose_msg.orientation.z = quaternion[2]
        pose_msg.orientation.w = quaternion[3]

        # Publish the Pose message
        for i in range(300):
            self.pose_publisher.publish(pose_msg)
            rospy.loginfo(f"Published Pose: {pose_msg}")


        # Sleep to maintain the publishing rate

if __name__ == '__main__':
    try:
        pose_publisher = PosePublisher()
        pose_publisher.publish_pose()
    except rospy.ROSInterruptException:
        pass
