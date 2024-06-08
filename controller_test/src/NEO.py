#!/usr/bin/env python

import rospy
import spatialgeometry as sg
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
import qpsolvers as qp
import cProfile

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseArray

from QP import quadratic_programming

class NEO:
    def __init__(self):
        rospy.init_node("NEO", anonymous=True)

        self.joint_state_subscribe = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        self.des_ee_pos_subscribe = rospy.Subscriber("/target_pose", Pose, self.des_ee_pos_callback)
        # self.collision_subscribe = rospy.Subscriber("/collisions", PoseArray, self.collision_callback)

        self.joint_vel_pub = rospy.Publisher("/neo/velocity", JointState, queue_size=10)

        self.joint_vel_msg = JointState()
        self.joint_vel_msg.name = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

        self.timer = rospy.Timer(rospy.Duration(0.01),  self.timer_callback)

        self.qp = quadratic_programming()
        self.panda = rtb.models.Panda()

        self.curr_ee_pos = sm.SE3()
        self.des_ee_pos = sm.SE3()
        self.collisions = []

        s = sg.Sphere(radius=0.05, base=sm.SE3(0., 2., 1.2))
        s.v = [0, -0.0, 0, 0, 0, 0]
        self.collisions.append(s)

        self.joint_state = []

    def joint_state_callback(self, msg):
        self.joint_state = np.array(msg.position[:7])
        self.curr_ee_pos = self.panda.fkine(self.joint_state)

    def des_ee_pos_callback(self, msg):
        position = msg.position
        orientation = msg.orientation

        # Convert quaternion to rotation matrix
        qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w
        rot_matrix = np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
        ])

        # Create transformation matrix
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rot_matrix
        transform_matrix[0, 3] = position.x
        transform_matrix[1, 3] = position.y
        transform_matrix[2, 3] = position.z

        self.des_ee_pos = sm.SE3(transform_matrix) # target ee pose


    def collision_callback(self, msg):

        # self.collisions.clear()

        for pose in msg.poses:
            s = sg.Sphere(radius=0.05, base=sm.SE3(pose.position.x, pose.position.y, pose.position.z))
            s.v = [0, -0.2, 0, 0, 0, 0]
            # self.collisions.append(s)
        

    def timer_callback(self, event):
        self.joint_vel_msg.header.stamp = rospy.Time.now()

        if len(self.joint_state) == 0:
            self.joint_vel_msg.velocity = [0., 0., 0., 0., 0., 0., 0.]
            self.joint_vel_pub.publish(self.joint_vel_msg)
            return

        try:
            
            joint_vel = self.qp.qpsolver(self.panda, self.joint_state, self.curr_ee_pos, self.des_ee_pos, self.collisions)
            self.joint_vel_msg.velocity = joint_vel
            self.joint_vel_pub.publish(self.joint_vel_msg)

        except Exception as e:
            rospy.logerr(f"Error in timer_callback: {e}")

if __name__ == "__main__":
    try:
        neo = NEO()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass