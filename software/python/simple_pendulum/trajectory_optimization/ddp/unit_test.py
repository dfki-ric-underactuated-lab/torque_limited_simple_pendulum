"""
Unit Tests
==========
"""


import eigenpy
# Matrix deprecated -> this should ensure that everything
# is treated/converted in array
eigenpy.switchToNumpyArray()

import os
from pathlib import Path
import unittest
import numpy as np

# crocoddyl imports
import crocoddyl
from crocoddyl.utils.pendulum import ActuationModelDoublePendulum

# pinocchio imports
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import *


class Test(unittest.TestCase):

    epsilon = 0.2

    def test_0_DDP_trajOPtimization(self):

        # Loading the double pendulum model

        WORK_DIR = Path(Path(os.path.abspath(__file__)).parents[5])

        filename = os.path.join(WORK_DIR, "data/urdf/simplependul_dfki_pino_Modi.urdf")

        # building the robot from the urdf
        robot = RobotWrapper.BuildFromURDF(filename)

        # creating the robot model and data in Crocoddyl
        robot_model = robot.model
        rmodel = robot.model
        rdata = robot_model.createData()

        # adding the toques and velocities limites
        lims = rmodel.effortLimit

        # *****************************************************************************
        # defining the state and actuation model
        state = crocoddyl.StateMultibody(robot_model)
        actModel = ActuationModelDoublePendulum(state, actLink=1)
        # the actuated link is the 1


        # *****************************************************************************
        # definning the initial state
        x0 = np.array([0., 0.])
        rmodel.defaultState = x0
        # defining the final desired state =[q, dot(q)]
        stateF = np.asarray([3.14159, 0.])

        # *****************************************************************************

        weights = np.array([1] + [0.1] * 1)

        # *****************************************************************************

        # defining the running cost model as a sun of cost
        runningCostModel = crocoddyl.CostModelSum(state, actModel.nu)

        # defining the terminal cost model as a sun of cost
        terminalCostModel = crocoddyl.CostModelSum(state, actModel.nu)

        # *****************************************************************************

        # definning the costs for the running and the terminal model

        xPendCost = crocoddyl.CostModelState(state,
                                             crocoddyl.ActivationModelWeightedQuad(weights),
                                             stateF,
                                             actModel.nu)

        uRegCost = crocoddyl.CostModelControl(state, 
                                              crocoddyl.ActivationModelQuad(1),
                                              actModel.nu)

        # *****************************************************************************

        # definning the dt, the discretization step, the time between each node
        dt = 4e-2

        # definning the number of nodes (time horizon)
        T = 150

        # *****************************************************************************

        # adding costs for the running model
        runningCostModel.addCost("xGoal", xPendCost, 1e-5)
        runningCostModel.addCost("uReg", uRegCost, 1e-4)

        # adding costs for the terminal model
        terminalCostModel.addCost("xGoalTerminal", xPendCost, 1e10)

        # *****************************************************************************

        # models
        runningModel = crocoddyl.IntegratedActionModelEuler(
            crocoddyl.DifferentialActionModelFreeFwdDynamics(state,
                                                             actModel,
                                                             runningCostModel), dt)

        terminalModel = crocoddyl.IntegratedActionModelEuler(
            crocoddyl.DifferentialActionModelFreeFwdDynamics(state,
                                                             actModel,
                                                             terminalCostModel), 0)

        # *****************************************************************************

        # Creating the shooting problem and the FDDP solver

        problem = crocoddyl.ShootingProblem(x0, [runningModel] * T, terminalModel)
        fddp = crocoddyl.SolverFDDP(problem)

        # *****************************************************************************

        # Solving the problem with the FDDP solver
        fddp.solve()

        xT = fddp.xs[-1]

        # *****************************************************************************

        swingup_success = True

        if np.abs((xT[0] % (2*np.pi)) - np.pi) > self.epsilon:
            if np.abs(xT[1]) > self.epsilon:
                swingup_success = False
                print("FDDP Trajectory optimization did not swingup",
                      "final state: ", xT)
        else:
            print("FDDP Trajectory optimization did swingup",
                  "final state: ", xT)

        self.assertTrue(swingup_success)
