#!/usr/bin/env python
"""
Author: Jesse Haviland
"""

import rospy
import time
import numpy as np
import spatialgeometry as sg
import roboticstoolbox as rtb
import spatialmath as sm
import qpsolvers as qp
import tf.transformations as tf

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseArray


class NEO:
    def __init__(self):
        rospy.init_node("NEO", anonymous=True)

        # Initialize the Panda robot model
        self.panda = rtb.models.Panda()

        # ROS Subscribers
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        self.target_pose_sub = rospy.Subscriber("/target_pose", Pose, self.des_ee_pos_callback)
        # self.collision_sub = rospy.Subscriber("/collisions", PoseArray, self.collision_callback)

        # ROS Publisher
        self.joint_vel_pub = rospy.Publisher("/neo/velocity", JointState, queue_size=10)

        # Initialize JointState message
        self.joint_vel_msg = JointState()
        self.joint_vel_msg.name = [
            'panda_joint1', 'panda_joint2', 'panda_joint3',
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]

        # Example collision objects
        self.collisions = [
            sg.Sphere(radius=0.05, base=sm.SE3(10, 10, 0.3)),
            sg.Sphere(radius=0.05, base=sm.SE3(10., 10., 0.65))
        ]

        self.joint_state = np.array([])

        # Flags to check if messages are received
        self.joint_state_received = False
        self.target_pose_received = False

        # Timer for periodic callback
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)

    def joint_state_callback(self, msg):
        """Callback function for joint state updates."""
        self.joint_state = np.array(msg.position[:7])
        self.curr_ee_pos = self.panda.fkine(self.joint_state)
        self.joint_state_received = True

    def des_ee_pos_callback(self, msg):
        """Callback function for target end-effector position updates."""
        position = msg.position
        orientation = msg.orientation

        # Convert quaternion to rotation matrix
        qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w
        rotation_matrix = tf.quaternion_matrix([qx, qy, qz, qw])

        # Create transformation matrix
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix[:3, :3]
        transform_matrix[0, 3] = position.x
        transform_matrix[1, 3] = position.y
        transform_matrix[2, 3] = position.z

        self.des_ee_pos = sm.SE3(transform_matrix)  # target ee pose
        self.target_pose_received = True
        rospy.loginfo("Received target pose!")

    def collision_callback(self, msg):
        """Callback function for collision updates (currently not used)."""
        # self.collisions.clear()
        for pose in msg.poses:
            s = sg.Sphere(radius=0.05, base=sm.SE3(pose.position.x, pose.position.y, pose.position.z))
            s.v = [0, -0.2, 0, 0, 0, 0]
            # self.collisions.append(s)

    def timer_callback(self, event):
        """Timer callback function for controlling the robot."""
        self.joint_vel_msg.header.stamp = rospy.Time.now()

        if not (self.joint_state_received and self.target_pose_received):
            rospy.loginfo(f"Waiting for data... Joint state received: {self.joint_state_received}, Target pose received: {self.target_pose_received}")
            return

        try:
            n = 7
            Te = self.panda.fkine(self.joint_state)
            eTep = Te.inv() * self.des_ee_pos
            e = np.sum(np.abs(np.r_[eTep.t, eTep.rpy() * np.pi / 180]))

            v, arrived = rtb.p_servo(Te, self.des_ee_pos, 0.5, 0.01)

            # Gain term (lambda) for control minimization
            Y = 0.01

            # Quadratic component of objective function
            Q = np.eye(n + 6)

            # Joint velocity component of Q
            Q[:n, :n] *= Y

            # Slack component of Q
            Q[n:, n:] = (1 / e) * np.eye(6)

            # Equality constraints
            Aeq = np.c_[self.panda.jacobe(self.joint_state), np.eye(6)]
            beq = v.reshape((6,))

            # Inequality constraints for joint limit avoidance
            Ain = np.zeros((n + 6, n + 6))
            bin = np.zeros(n + 6)

            # Minimum angle in radians for joint limit avoidance
            ps = 0.05
            # Influence angle in radians for velocity damper
            pi = 0.9

            # Form the joint limit velocity damper
            Ain[:n, :n], bin[:n] = self.panda.joint_velocity_damper(ps, pi, n)

            for collision in self.collisions:
                # Form velocity damper inequality constraint for each collision object
                c_Ain, c_bin = self.panda.link_collision_damper(
                    collision,
                    self.joint_state,
                    0.3,
                    0.05,
                    1.0,
                    start=self.panda.link_dict["panda_link1"],
                    end=self.panda.link_dict["panda_link7"]
                )

                if c_Ain is not None and c_bin is not None:
                    c_Ain = np.c_[c_Ain, np.zeros((c_Ain.shape[0], 6))]

                    # Stack the inequality constraints
                    Ain = np.r_[Ain, c_Ain]
                    bin = np.r_[bin, c_bin]

            # Linear component of objective function: the manipulability Jacobian
            c = np.r_[-self.panda.jacobm(self.joint_state).reshape((n,)), np.zeros(6)]

            # Lower and upper bounds on joint velocity and slack variable
            lb = -np.r_[self.panda.qdlim[:n], 10 * np.ones(6)]
            ub = np.r_[self.panda.qdlim[:n], 10 * np.ones(6)]

            # Solve for the joint velocities dq
            qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver='osqp')

            # Apply the joint velocities to the Panda
            self.joint_vel_msg.velocity = qd[:n]
            self.joint_vel_pub.publish(self.joint_vel_msg)
            self.panda.q = self.joint_state

        except Exception as e:
            self.joint_vel_msg.velocity = [0., 0., 0., 0., 0., 0., 0.]
            self.joint_vel_pub.publish(self.joint_vel_msg)
            rospy.logerr(f"Error in timer_callback: {e}")


if __name__ == "__main__":
    try:
        neo = NEO()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
