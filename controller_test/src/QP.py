#!/usr/bin/env python

import rospy
import spatialgeometry as sg
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
import qpsolvers as qp

from objective import objective_function
from constraint import constraint


'''
input : QP parameter, variable
output : target joint velocity
'''

class quadratic_programming:

    def __init__(self):

        self.n = 7
        self.panda = rtb.models.Panda()
        self.obj = objective_function()
        self.cons = constraint(self.panda)


    def qpsolver(self, robot, joint_state, curr_ee_pos, des_ee_pos, collisions):

        Aeq, beq, e, arrived = self.cons.equality_constraint(curr_ee_pos, des_ee_pos, joint_state)

        Ain, bin, lb, ub = self.cons.inequality_constraint(collisions, joint_state)

        Q, C = self.obj.objective(robot, joint_state, e)
        qd = qp.solve_qp(Q, C, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver='osqp')

        return qd[:self.n]
