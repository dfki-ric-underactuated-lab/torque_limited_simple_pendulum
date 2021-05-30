import sys
import time
from canmotorlib import CanMotorController
import matplotlib.pyplot as plt
import math
import numpy as np
import pandas as pd

Kp = 50.0
Kd = 2.0

# Motor ID
motor_id = 0x02
can_port = 'can0'

motor_controller = CanMotorController(can_port, motor_id)

motor_controller.enable_motor()

pos, vel, effort = motor_controller.send_deg_command(0, 0, 0, 0, 0)
print("After enabling motor, pos: ", pos, ", vel: ", vel, ", effort: ", effort)

while abs(pos) > 1 or abs(vel) > 1 or abs(effort) > 0.1:
    motor_controller.set_zero_position()
    pos, vel, effort = motor_controller.send_rad_command(0, 0, 0, 0, 0)
    print("Motor position after setting zero position: ", pos, ", vel: ", vel, ", effort: ", effort)

# Read the trajectory file for swing up control
data = pd.read_csv("trajectory_optimisation/traj_opt_traj.csv")
#data = pd.read_csv("controllers/energy_shaping/traj_opt_traj.csv")
#data = pd.read_csv("ssystem_identification/excitation_trajectories/tmotor-2021-01-29-09-28-14/trajectory-pos-20.csv")
numSteps = len(data)
position = np.zeros(numSteps+500)
velocity = np.zeros(numSteps+500)
torque = np.zeros(numSteps+500)

time_vec = np.zeros(numSteps+500)

# New better method: Precompute Trajectory
print("Generating Trajectory...")
pos_traj = data["pos"]
vel_traj = data["vel"]
tau_traj = data["torque"]
dt = data["time"][1] - data["time"][0]
print("Sending Trajectory to Motor... ")

t = 0.0
print("Start")
realStartT = time.time()
for i in range(numSteps):
    
    # Send pos, vel and tau_ff command and use the in-built low level controller
    pos, vel, tau = motor_controller.send_rad_command(pos_traj[i], vel_traj[i], Kp, Kd, tau_traj[i])

    position[i] = pos
    velocity[i] = vel
    torque[i] = tau
    time_vec[i] = t

    t = t + dt
    time.sleep(dt)

pos_final = pos_traj[numSteps-1]

for i in range(500):
    
    # Send pos, vel and tau_ff command and use the in-built low level controller
    pos, vel, tau = motor_controller.send_rad_command(pos_final, 0.0, Kp, Kd, 0.0)

    pos_traj[i+numSteps] = pos_final
    vel_traj[i+numSteps] = 0.0
    tau_traj[i+numSteps] = 0.0

    position[i+numSteps] = pos
    velocity[i+numSteps] = vel
    torque[i+numSteps] = tau
    time_vec[i+numSteps] = t

    t = t + dt
    time.sleep(dt)

realEndT = time.time()

realdT = (realEndT - realStartT) / numSteps

print("End. New dt: {}".format(realdT))

# Set Kp = Kd = 0
motor_controller.send_deg_command(0, 0, 0, 0, 0)
# Disable the motor
motor_controller.disable_motor()

plt.figure
plt.plot(time_vec, position)
plt.plot(time_vec, pos_traj)
plt.xlabel("Time (s)")
plt.ylabel("Position (deg)")
plt.title("Position (deg) vs Time (s)")
plt.legend(['position_measured', 'position_desired'])
plt.show()

plt.figure
plt.plot(time_vec, velocity)
plt.plot(time_vec, vel_traj)
plt.xlabel("Time (s)")
plt.ylabel("Velocity (deg/s)")
plt.legend(['velocity_measured', 'velocity_desired'])
plt.title("Velocity (deg/s) vs Time (s)")
plt.show()

plt.figure
plt.plot(time_vec, torque)
plt.plot(time_vec, tau_traj)
plt.xlabel("Time (s)")
plt.ylabel("Torque (Nm)")
plt.title("Torque (Nm) vs Time (s)")
plt.legend(['Measured Torque', 'Estimated Torque'])
plt.show()

def running_mean(x, N):
    cumsum = np.cumsum(np.insert(x, 0, 0))
    return (cumsum[N:] - cumsum[:-N]) / float(N)


filtered_torque = running_mean(np.array(torque), 10)
filtered_tau_traj = running_mean(np.array(tau_traj), 10)

plt.figure
plt.plot(time_vec[:filtered_torque.size], filtered_torque)
plt.plot(time_vec[:filtered_torque.size], filtered_tau_traj)
plt.xlabel("Time (s)")
plt.ylabel("Torque (Nm)")
plt.title("Filtered Torque (Nm) vs Time (s) with moving average filter (window = 100)")
plt.legend(['Measured Torque', 'Estimated Torque'])
plt.show()

measured_csv_data = np.array([np.array(time_vec),
                    np.array(position),
                    np.array(velocity),
                    np.array(torque)]).T
np.savetxt("measured_data.csv", measured_csv_data, delimiter=',', header="time,pos,vel,torque", comments="")

desired_csv_data = np.array([np.array(time_vec),
                    np.array(pos_traj),
                    np.array(vel_traj),
                    np.array(tau_traj)]).T
np.savetxt("desired_data.csv", desired_csv_data, delimiter=',', header="time,pos,vel,torque", comments="")
