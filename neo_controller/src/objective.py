#!/usr/bin/env python

import rospy
import spatialgeometry as sg
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np


'''
# The following code defines the objective function of a Quadratic Programming (QP) problem.
# The objective function aims to minimize x while maximizing manipulability.
# The parameter Q is the weight for minimizing x, and the parameter C is the weight for maximizing manipulability.
# The variable vector x consists of the velocities of the joints and a slack vector.
'''

'''
input : robot[rtb robot model], joint_state[ndarray], error
output : C(linear component)
'''


class objective_function():

    def __init__(self):

        self.n = 7

    def manipulability(self, robot, joint_state):

        # maximize manupulability
        c = np.r_[-robot.jacobm(joint_state).reshape((self.n,)), np.zeros(6)]

        return c
    
    def quadratic_weight(self, e):

        # Gain term (lambda) for control minimisation
        Y = 0.01

        # Quadratic component of objective function
        Q = np.eye(self.n + 6)

        # Joint velocity component of Q
        Q[:self.n, :self.n] *= Y

        # Slack component of Q
        Q[self.n:, self.n:] = (1 / e) * np.eye(6)

    def objective(self, robot, joint_state, e):

        Q = self.quadratic_weight(e)
        c = self.manipulability(robot, joint_state)

        return Q, c
        