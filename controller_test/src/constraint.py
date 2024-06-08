#!/usr/bin/env python

import rospy
import spatialgeometry as sg
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np


'''
input : current e-e pose, desired e-e pose --> sm3
output : desired e-e spatial velocity --> sm3
'''

class constraint:

    def __init__(self, robot):

        self.panda = robot
        self.n = 7

        # PBS parameter
        self.gain = 0.5
        self.threshold = 0.01

    def pbs(self, curr_ee_pos, des_ee_pos):

        Te = curr_ee_pos
        Tep = des_ee_pos

        # Transform from the end-effector to desired pose
        eTep = Te.inv() * Tep

        # Spatial error
        e = np.sum(np.abs(np.r_[eTep.t, eTep.rpy() * np.pi / 180]))

        # Calulate the required end-effector spatial velocity for the robot
        # to approach the goal. Gain is set to 1.0
        v, arrived = rtb.p_servo(Te, Tep, self.gain, self.threshold)

        return e, v, arrived


    def velocity_damper(self, collisions, joint_state):

        # Form the joint limit velocity damper
        Ain = np.zeros((self.n + 6, self.n + 6))
        bin = np.zeros(self.n + 6)


        # The minimum angle (in radians) in which the joint is allowed to approach
        # to its limit
        ps = 0.05

        # The influence angle (in radians) in which the velocity damper
        # becomes active
        pi = 0.9

        Ain[:self.n, :self.n], bin[:self.n] = self.panda.joint_velocity_damper(ps, pi, self.n)



        for collision in collisions:

            # Form the velocity damper inequality contraint for each collision
            # object on the robot to the collision in the scene
            c_Ain, c_bin = self.panda.link_collision_damper(
                collision,
                joint_state[:self.n],
                0.3,
                0.05,
                1.0,
                start=self.panda.link_dict["panda_link1"],
                end=self.panda.link_dict["panda_link7"],
            )

            # If there are any parts of the robot within the influence distance
            # to the collision in the scene
            if c_Ain is not None and c_bin is not None:
                c_Ain = np.c_[c_Ain, np.zeros((c_Ain.shape[0], 6))]

                # Stack the inequality constraints
                Ain = np.r_[Ain, c_Ain]
                bin = np.r_[bin, c_bin]

        return Ain, bin


    def bound(self):
        # The lower and upper bounds on the joint velocity and slack variable
        lb = -np.r_[self.panda.qdlim[:self.n], 10 * np.ones(6)]
        ub = np.r_[self.panda.qdlim[:self.n], 10 * np.ones(6)]

        return lb, ub


    def equality_constraint(self, curr_ee_pos, des_ee_pos, joint_state):
    
        e, v, arrived = self.pbs(curr_ee_pos, des_ee_pos)

        # The equality contraints
        Aeq = np.c_[self.panda.jacobe(joint_state), np.eye(6)]
        beq = v.reshape((6,))

        return Aeq, beq, e, arrived
    
    def inequality_constraint(self, collisions, joint_state):

        Ain, bin = self.velocity_damper(collisions, joint_state)

        lb, ub = self.bound()

        return Ain, bin, lb, ub


    
