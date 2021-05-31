import sys

import eigenpy  
#eigenpy.switchToNumpyMatrix()  
eigenpy.switchToNumpyArray() # Matrix deprecated -> this should ensure that everything is treated/converted in array

import subprocess
import time
import numpy as np
import os
import sys
import math
import matplotlib.pyplot as plt
#crocoddyl imports
import crocoddyl
from crocoddyl.utils.pendulum import CostModelDoublePendulum, ActuationModelDoublePendulum
#pinocchio imports
from pinocchio.robot_wrapper import RobotWrapper
import pinocchio
from pinocchio.utils import *
from pinocchio import GeometryModel, GeometryObject
import pinocchio as pin
#import the robots with data
import example_robot_data
# import the driver file in order to control the pundulum
from canmotorlib import CanMotorController


#********************************************************************************************************************************

# for controlling the robot using the driver 
'''

# Motor ID
motor_id = 0x01

if len(sys.argv) != 2:
    print('Provide CAN device name (can0, slcan0 etc.)')
    sys.exit(0)

print("Using Socket {} for can communucation".format(sys.argv[1],))
# print(type((sys.argv[1],)))

motor_controller = CanMotorController(sys.argv[1], motor_id)

pos, vel, curr = motor_controller.enable_motor()

# time.sleep(1)

while abs(np.rad2deg(pos)) > 0.5:
    pos, vel, curr = motor_controller.set_zero_position()
    print("Position: {}, Velocity: {}, Torque: {}".format(np.rad2deg(pos), np.rad2deg(vel), curr))

# time.sleep(1)

#********************************************************************************************************************************


'''
# call the gepetto viewer server
gvs = subprocess.Popen(["./gepetto-viewer.sh","&"]) 
print('Loading the viewer ...')
time.sleep(1)


#********************************************************************************************************************************

# Loading the double pendulum model
filename="~/simplependul_dfki_pino.urdf"

#building the robot from the urdf
robot =  RobotWrapper.BuildFromURDF(filename)

#creating the robot model and data in Crocoddyl
robot_model = robot.model
rmodel = robot.model
rdata  = robot_model.createData()

#adding the toques and velociies limites 
lims = rmodel.effortLimit

lims = rmodel.velocityLimit

#********************************************************************************************************************************
# definning the state ad actuation model
state = crocoddyl.StateMultibody(robot_model)
actModel = ActuationModelDoublePendulum(state, actLink=1) # the actuated link is the 1, in the case of a double pundulum you have to change it


#********************************************************************************************************************************
# definning the final desired state =[q, dot(q)]
stateF = np.asarray([3.14159, 0.])

#********************************************************************************************************************************

# 
#weights = np.array([1] + [0.1] * 1)  # not used in this script but could be used to tune the "crocoddyl.ActivationModelQuad" in the state and ontrol costs by remplacing (state.ndx) by your weights

#********************************************************************************************************************************

# defining the running cost model as a sun of cost
runningCostModel = crocoddyl.CostModelSum(state, actModel.nu)

# difining the terminal cost model as a sun of cost
terminalCostModel = crocoddyl.CostModelSum(state, actModel.nu)

#********************************************************************************************************************************

# definning the costs for the running and the terminal model
xRegCost = crocoddyl.CostModelState(state, crocoddyl.ActivationModelQuad(state.ndx), stateF, actModel.nu)
uRegCost = crocoddyl.CostModelControl(state, crocoddyl.ActivationModelQuad(1), actModel.nu)

xPendCost = crocoddyl.CostModelState(state, crocoddyl.ActivationModelQuad(state.ndx), stateF, actModel.nu)

#********************************************************************************************************************************

#definning the dt, the discretization step, the time between each node
dt = 1e-3

#definning the number of nodes (time horizon)
T = 4000

#********************************************************************************************************************************

# Extract Time
time_traj = np.linspace(0.0000, 4.0000, 4000)
time_traj = time_traj.reshape(4000, 1).T
print("Time Array Shape: {}".format(time_traj.shape))

#********************************************************************************************************************************


#adding costs for the running model
runningCostModel.addCost("xGoal", xPendCost, 0.15)
runningCostModel.addCost("uReg", uRegCost, 0.000010)

#adding costs for the terminal model
terminalCostModel.addCost("xGoal", xPendCost, 1e10)

#********************************************************************************************************************************

#models
runningModel = crocoddyl.IntegratedActionModelEuler(
    crocoddyl.DifferentialActionModelFreeFwdDynamics(state, actModel, runningCostModel), dt)

terminalModel = crocoddyl.IntegratedActionModelEuler(
    crocoddyl.DifferentialActionModelFreeFwdDynamics(state, actModel, terminalCostModel), dt)

#********************************************************************************************************************************

#definning the initial state
x0 = np.array([0., 0.])

# Creating the shooting problem and the FDDP solver

problem = crocoddyl.ShootingProblem(x0, [runningModel] * T, terminalModel)
fddp = crocoddyl.SolverBoxFDDP(problem)


#********************************************************************************************************************************

#for the display definning the position and orientation of the camera by a quaternion
cameraTF = [1.4, 0., 0.2, 0.5, 0.5, 0.5, 0.5]

#********************************************************************************************************************************

display = crocoddyl.GepettoDisplay(robot, 4, 4, cameraTF, True)
fddp.setCallbacks([crocoddyl.CallbackLogger(), crocoddyl.CallbackVerbose(), crocoddyl.CallbackDisplay(display)])
#********************************************************************************************************************************
#to test computational time
startdtTest = time.time()
#********************************************************************************************************************************

# Solving the problem with the FDDP solver
fddp.solve()

#********************************************************************************************************************************

enddtTest = time.time()

dt = (enddtTest - startdtTest) / 1000
cmd_freq = 1 / dt
print("Dt = {}".format(dt))
print("Command Frequency: {} Hz".format(cmd_freq))

#********************************************************************************************************************************

log = fddp.getCallbacks()[0]


#*********************************************************************************************************************************************************8
#saving the data as a nuöpy files and as .csv in order to send them to the robot

torque = []
Q = []
V = []


for i in range(4000):
    torque.extend([fddp.us[i]])
    Q.extend([fddp.xs[i][:1]])
    V.extend([fddp.xs[i][1:]])


TOR = np.asarray(torque)
torque_traj = TOR[:].reshape(T,1).T


ROT = np.asarray(Q)
theta = ROT[:].reshape(T,1).T


VEL = np.asarray(V)
theta_dot = VEL[:].reshape(T,1).T


np.save('torque.npy',torque)
np.save('rotations.npy',Q)
np.save('velocity.npy',V)

csv_data = np.vstack((time_traj, theta, theta_dot, torque_traj)).T

np.savetxt("traj_opt_traj.csv", csv_data, delimiter=',', header="time,pos,vel,torque", comments="")


#***************************************************************************************************


# Plotting the entire motion

crocoddyl.plotOCSolution(log.xs, log.us, figIndex=1, show=True)
crocoddyl.plotConvergence(log.costs, log.u_regs, log.x_regs, log.grads, log.stops, log.steps, figIndex=2)

# Display the entire motion

display = crocoddyl.GepettoDisplay(robot, floor=False)
display.displayFromSolver(fddp)



