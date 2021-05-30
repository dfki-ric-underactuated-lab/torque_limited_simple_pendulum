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
from math import pi

import crocoddyl
import numpy as np
import example_robot_data
from crocoddyl.utils.pendulum import CostModelDoublePendulum, ActuationModelDoublePendulum

from pinocchio.robot_wrapper import RobotWrapper

import pinocchio
from pinocchio.utils import *
from pinocchio import GeometryModel, GeometryObject
import pinocchio as pin

from canmotorlib import CanMotorController

from scipy.signal import lfilter, lfilter_zi, filtfilt, butter

b, a = butter(3, 0.05)

time_traj = np.linspace(0.0000, 6.0000, 3000)



#********************************************************************************************************************************


# Motor ID
motor_id = 0x03

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
'''

#********************************************************************************************************************************

# Loading the double pendulum model
filename="./simplependul_dfki_pino.urdf"


robot =  RobotWrapper.BuildFromURDF(filename)




robot_model = robot.model
rmodel = robot.model
rdata  = robot_model.createData()

lims = rmodel.effortLimit

lims = rmodel.velocityLimit

#********************************************************************************************************************************

state = crocoddyl.StateMultibody(robot_model)
actModel = ActuationModelDoublePendulum(state, actLink=1) # the catuated link is the 2


#********************************************************************************************************************************
stateF = np.asarray([3.14159, 0.])

#********************************************************************************************************************************

weights = np.array([1, 1] + [0.1] * 1)

runningCostModel = crocoddyl.CostModelSum(state, actModel.nu)
terminalCostModel = crocoddyl.CostModelSum(state, actModel.nu)

#********************************************************************************************************************************

xRegCost = crocoddyl.CostModelState(state, crocoddyl.ActivationModelQuad(state.ndx), stateF, actModel.nu)
uRegCost = crocoddyl.CostModelControl(state, crocoddyl.ActivationModelQuad(1), actModel.nu)

xPendCost = crocoddyl.CostModelState(state, crocoddyl.ActivationModelQuad(state.ndx), stateF, actModel.nu)

#********************************************************************************************************************************

dt = 2e-3

#********************************************************************************************************************************
#adding costs
runningCostModel.addCost("xGoal", xPendCost, 0.15)
runningCostModel.addCost("uReg", uRegCost, 0.00001)

terminalCostModel.addCost("xGoal", xPendCost, 1e10)

#********************************************************************************************************************************

#models
runningModel = crocoddyl.IntegratedActionModelEuler(
    crocoddyl.DifferentialActionModelFreeFwdDynamics(state, actModel, runningCostModel), dt)

terminalModel = crocoddyl.IntegratedActionModelEuler(
    crocoddyl.DifferentialActionModelFreeFwdDynamics(state, actModel, terminalCostModel), dt)


#********************************************************************************************************************************

# Creating the shooting problem and the FDDP solver

cameraTF = [1.4, 0., 0.2, 0.5, 0.5, 0.5, 0.5]


T = 3000
x0 = np.array([0., 0.])

torque = []
Q = []
V = []

posA = 0
velA = 0
fddp = []

#********************************************************************************************************************************

for j in range(T):


    #print("Initial Position: {}, Velocity: {}, Torque: {}".format(np.rad2deg(pos), np.rad2deg(vel),curr))
                                                                
    x0 = np.array([posA, velA])
    problem = crocoddyl.ShootingProblem(x0, [runningModel] * (T-j), terminalModel)

    fddp = crocoddyl.SolverBoxFDDP(problem)

    #display = crocoddyl.GepettoDisplay(robot, 4, 4, cameraTF, True)
    #fddp.setCallbacks([crocoddyl.CallbackLogger(), crocoddyl.CallbackVerbose(), crocoddyl.CallbackDisplay(display)])


    # Solving the problem with the FDDP solver
    fddp.solve()

    #log = fddp.getCallbacks()[0]
    
    torque.extend([fddp.us[j]])
    Q.extend([fddp.xs[j][:1]])
    V.extend([fddp.xs[j][1:]])
    
    #print("torque: ", fddp.us[0])
    #posA, velA, currA = motor_controller.send_deg_command(0, 0, 0, 0, fddp.us[0])
    posA, velA, currA = motor_controller.send_rad_command(fddp.xs[0][:1], fddp.xs[0][1:], 5, 0.1, fddp.us[0])


'''
torque = []
Q = []
V = []

'''
for i in range(3000):
    torque.extend([fddp.us[i]])
    Q.extend([fddp.xs[i][:1]])
    V.extend([fddp.xs[i][1:]])





TOR = np.asarray(torque)

ROT = np.asarray(Q)

VEL = np.asarray(V)
'''

kp = 50
kd = 2*np.sqrt(kp)
kd = 2
'''

np.save('torque.npy',torque)
np.save('rotations.npy',Q)
np.save('velocity.npy',V)


#***************************************************************************************************




pos, vel, curr = motor_controller.disable_motor()
print("Final Position: {}, Velocity: {}, Torque: {}".format(np.rad2deg(pos), np.rad2deg(vel), curr))

'''
# Plotting the entire motion

crocoddyl.plotOCSolution(log.xs, log.us, figIndex=1, show=True)
crocoddyl.plotConvergence(log.costs, log.u_regs, log.x_regs, log.grads, log.stops, log.steps, figIndex=2)

# Display the entire motion

display = crocoddyl.GepettoDisplay(robot, floor=False)
display.displayFromSolver(fddp)
'''


