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

        self.joint_state_subscribe = rospy.Subscriber("/joint_state", JointState, self.joint_state_callback)
        self.des_ee_pos_subscribe = rospy.Subscriber("/target_pose", Pose, self.des_ee_pos_callback)
        self.collision_subscribe = rospy.Subscriber("collision", PoseArray, self.collision_callback)

        self.timer = rospy.Timer(rospy.Duration(0.1),  self.timer_callback)


        self.qp = quadratic_programming()
        self.panda = rtb.models.Panda()

        self.curr_ee_pos = sm.SE3()
        self.des_ee_pos = sm.SE3()
        self.collision = []

    def joint_state_callback(self, msg):
        self.joint_state = np.array(msg.position)
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

        collision_list = []

        for pose in msg.poses:
            position = [pose.position.x, pose.position.y, pose.position.z]
            collision_list.append(position)

        

    def timer_callback(self, event):

        joint_vel = self.qp.qpsolver(self.panda, self.joint_state, self.curr_ee_pos, self.des_ee_pos, collision)
        


