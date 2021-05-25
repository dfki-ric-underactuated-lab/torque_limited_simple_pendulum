import sys
import time
from canmotorlib import CanMotorController
import matplotlib.pyplot as plt
import math
import numpy as np
import pandas as pd
import os

def setZeroPosition(motor, initPos):

    pos = initPos

    while abs(np.rad2deg(pos)) > 0.5:
        pos, vel, curr = motor.set_zero_position()
        print("Position: {}, Velocity: {}, Torque: {}".format(np.rad2deg(pos), np.rad2deg(vel),
                                                                curr))


# Motor ID
motor_id = 0x01

g = -9.81                           # gravitational acceleration on earth
m = 0.546                           # mass at the end of the pendulum
l = 0.5                             # length of the rod

# CAN port
can_port = 'can0'

if len(sys.argv) != 2:
    print('Provide CAN device name (can0, slcan0 etc.)')
    sys.exit(0)

print("Using Socket {} for can communucation".format(sys.argv[1],))

# Create motor controller objects
motor_controller = CanMotorController(sys.argv[1], motor_id)

print("Enabling Motors..")

pos, vel, torque = motor_controller.enable_motor()

print("Shoulder Motor Status: Pos: {}, Vel: {}, Torque: {}".format(spos, vel, torque))

#print("Setting Shoulder Motor to Zero Position...")

#setZeroPosition(motor_controller, pos)

print("Start")
numSteps = 10000

time_vec = np.zeros(numSteps)

position = np.zeros(numSteps)

velocity = np.zeros(numSteps)

torque = np.zeros(numSteps)

desired_torque = np.zeros(numSteps)

tau = 0.0

start_time = time.time()

for i in range(numSteps):

    dt = time.time()
    traj_time = dt - start_time
    
    time_vec[i] = traj_time

    # Send only the tau_ff command and use the in-built low level controller
    pos, vel, tau = motor_controller.send_rad_command(0.0,0.0,0.0,0.0,tau[0])

    # Compute gravity torques 
    tau = m*g*l*math.sin(pos)    

    # Store data in lists
    position[i] = pos
    velocity[i] = vel
    torque[i] = tau

    desired_torque[i] = tau

print("Disabling Motors...")

pos, vel, tau = motor_controller.disable_motor()

print("Motor Status: Pos: {}, Vel: {}, Torque: {}".format(pos, vel, tau))

plt.figure
plt.plot(time_vec, position)
plt.xlabel("Time (s)")
plt.ylabel("Position (rad)")
plt.title("Position (rad) vs Time (s)")
plt.show()

plt.figure
plt.plot(time_vec, velocity)
plt.xlabel("Time (s)")
plt.ylabel("Velocity (rad/s)")
plt.title("Velocity (rad/s) vs Time (s)")
plt.show()

plt.figure
plt.plot(time_vec, torque)
plt.plot(time_vec, desired_torque)
plt.xlabel("Time (s)")
plt.ylabel("Torque (Nm)")
plt.title("Torque (Nm) vs Time (s)")
plt.legend(['Measured', 'Desired'])
plt.show()

def running_mean(x, N):
    cumsum = np.cumsum(np.insert(x, 0, 0))
    return (cumsum[N:] - cumsum[:-N]) / float(N)


filtered_torque = running_mean(np.array(torque), 10)
time_vec_filtered = running_mean(np.array(time_vec), 10)
filtered_desired_torque = running_mean(np.array(desired_torque), 10)

plt.figure
plt.plot(time_vec_filtered, filtered_torque)
plt.plot(time_vec_filtered, filtered_desired_torque)
plt.xlabel("Time (s)")
plt.ylabel("Torque (Nm)")
plt.title("Filtered Torque (Nm) vs Time (s) with moving average filter (window = 100)")
plt.legend(['Measured', 'Desired'])
plt.show()


measured_csv_data = np.array([np.array(time_vec),
                    np.array(position),
                    np.array(velocity),
                    np.array(torque)]).T
np.savetxt("measured_data.csv", measured_csv_data, delimiter=',', header="time,pos,vel,torque", comments="")
