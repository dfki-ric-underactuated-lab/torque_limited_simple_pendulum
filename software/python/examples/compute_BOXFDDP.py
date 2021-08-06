import sys

import eigenpy
# Matrix deprecated -> this should ensure that everything
# is treated/converted in array
eigenpy.switchToNumpyArray()

import subprocess
import time
import numpy as np
import sys
import os
from pathlib import Path

# crocoddyl imports
import crocoddyl
from crocoddyl.utils.pendulum import ActuationModelDoublePendulum

# pinocchio imports
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import *

# call the gepetto viewer server
gvs = subprocess.Popen(["./gepetto-viewer.sh", "&"])
print('Loading the viewer ...')
time.sleep(1)


# *****************************************************************************

# Loading the double pendulum model
WORK_DIR = Path(Path(os.path.abspath(__file__)).parents[3])
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
# definning the state ad actuation model
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

# Extract Time
time_traj = np.linspace(0.0000, 6.0000, 150)
time_traj = time_traj.reshape(150, 1).T
print("Time Array Shape: {}".format(time_traj.shape))

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

# for the display definning the position and orientation of the camera
#  by a quaternion
cameraTF = [1.4, 0., 0.2, 0.5, 0.5, 0.5, 0.5]

# *****************************************************************************

display = crocoddyl.GepettoDisplay(robot, 4, 4, cameraTF, True)
fddp.setCallbacks([crocoddyl.CallbackLogger(),
                   crocoddyl.CallbackVerbose(),
                   crocoddyl.CallbackDisplay(display)])
# *****************************************************************************

# to test computational time
startdtTest = time.time()
# Solving the problem with the FDDP solver
fddp.solve()
print("time taken--- %s seconds ---" % (time.time() - startdtTest))
xT = fddp.xs[-1]

# *****************************************************************************

log = fddp.getCallbacks()[0]

# ******************************************************************************
# saving the data as a numpy files and as .csv in order to send
# them to the robot

torque = []
Q = []
V = []


for i in range(150):
    torque.extend([fddp.us[i]])
    Q.extend([fddp.xs[i][:1]])
    V.extend([fddp.xs[i][1:]])


TOR = np.asarray(torque)
torque_traj = TOR[:].reshape(T, 1).T


ROT = np.asarray(Q)
theta = ROT[:].reshape(T, 1).T


VEL = np.asarray(V)
theta_dot = VEL[:].reshape(T, 1).T


csv_data = np.vstack((time_traj, theta, theta_dot, torque_traj)).T

np.savetxt("../../../data/trajectories/ddp/trajectory.csv",
           csv_data, delimiter=',', header="time,pos,vel,torque", comments="")


# *****************************************************************************


# Plotting the entire motion

crocoddyl.plotOCSolution(log.xs, log.us, figIndex=1, show=True)
crocoddyl.plotConvergence(log.costs, log.u_regs, log.x_regs,
                          log.grads, log.stops, log.steps, figIndex=2)

# Display the entire motion

display = crocoddyl.GepettoDisplay(robot, floor=False)
display.displayFromSolver(fddp)
