import os
import numpy as np
import matplotlib.pyplot as plt

from simple_pendulum.trajectory_optimization.direct_collocation.direct_collocation import DirectCollocationCalculator
from simple_pendulum.model.pendulum_plant import PendulumPlant
from simple_pendulum.simulation.simulation import Simulator
from simple_pendulum.controllers.open_loop.open_loop import OpenLoopController, \
                                                            OpenLoopAndLQRController
from simple_pendulum.controllers.pid.pid import PIDController
from simple_pendulum.controllers.tvlqr.tvlqr import TVLQRController
from simple_pendulum.utilities.process_data import prepare_trajectory

log_dir = "log_data/direct_collocation"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)

# pendulum parameters
mass = 0.57288
length = 0.5
damping = 0.10
gravity = 9.81
coulomb_fric = 0.0
torque_limit = 1.5

# swingup parameters
x0 = [0.0, 0.0]
goal = [np.pi, 0.0]

# direct collocation parameters
N = 21
max_dt = 0.5


####################
# Compute trajectory
####################


dircal = DirectCollocationCalculator()
dircal.init_pendulum(mass=mass,
                     length=length,
                     damping=damping,
                     gravity=gravity,
                     torque_limit=torque_limit)
x_trajectory, dircol, result = dircal.compute_trajectory(N=N,
                                                         max_dt=max_dt,
                                                         start_state=x0,
                                                         goal_state=goal)
T, X, XD, U = dircal.extract_trajectory(x_trajectory, dircol, result, N=1000)

# plot results
fig, ax = plt.subplots(3, 1, figsize=(18, 6), sharex="all")

ax[0].plot(T, X, label="theta")
ax[0].set_ylabel("angle [rad]")
ax[0].legend(loc="best")
ax[1].plot(T, XD, label="theta dot")
ax[1].set_ylabel("angular velocity [rad/s]")
ax[1].legend(loc="best")
ax[2].plot(T, U, label="u")
ax[2].set_xlabel("time [s]")
ax[2].set_ylabel("input torque [Nm]")
ax[2].legend(loc="best")
plt.show()

dircal.plot_phase_space_trajectory(x_trajectory)

# save results
csv_data = np.vstack((T, X, XD, U)).T
csv_path = os.path.join(log_dir, "trajectory.csv")
np.savetxt(csv_path, csv_data, delimiter=',',
           header="time,pos,vel,torque", comments="")


#####################
# Simulate trajectory
#####################

pendulum = PendulumPlant(mass=mass,
                         length=length,
                         damping=damping,
                         gravity=gravity,
                         coulomb_fric=coulomb_fric,
                         inertia=None,
                         torque_limit=torque_limit)

sim = Simulator(plant=pendulum)

csv_path = "../../../data/trajectories/direct_collocation/trajectory.csv"
data_dict = prepare_trajectory(csv_path)

# controller = OpenLoopController(data_dict=data_dict)
# controller = PIDController(data_dict=data_dict, Kp=3.0, Ki=1.0, Kd=1.0)
controller = TVLQRController(data_dict=data_dict, mass=mass, length=length,
                             damping=damping, gravity=gravity,
                             torque_limit=torque_limit)
controller.set_goal(goal)
trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
dt = trajectory[1][0] - trajectory[0][0]
t_final = trajectory[-1][0] + 0.0

x0 = [trajectory[0][1], trajectory[0][2]]

T, X, U = sim.simulate_and_animate(t0=trajectory[0][0],
                                   x0=x0,
                                   tf=t_final,
                                   dt=dt,
                                   controller=controller,
                                   integrator="runge_kutta",
                                   phase_plot=False,
                                   save_video=False)

fig, ax = plt.subplots(3, 1, figsize=(18, 6), sharex="all")

ax[0].plot(T, np.asarray(X).T[0], label="theta")
ax[0].set_ylabel("angle [rad]")
ax[0].legend(loc="best")
ax[1].plot(T, np.asarray(X).T[1], label="theta dot")
ax[1].set_ylabel("angular velocity [rad/s]")
ax[1].legend(loc="best")
ax[2].plot(T, U, label="u")
ax[2].set_xlabel("time [s]")
ax[2].set_ylabel("input torque [Nm]")
ax[2].legend(loc="best")
plt.show()
