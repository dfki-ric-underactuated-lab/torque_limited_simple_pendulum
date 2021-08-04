import numpy as np
import matplotlib.pyplot as plt

from model.pendulum_plant import PendulumPlant
from simulation.simulation import Simulator
from controllers.open_loop.open_loop import OpenLoopController
from utilities.process_data import prepare_trajectory

mass = 0.57288
length = 0.5
damping = 0.15
gravity = 9.81
coulomb_fric = 0.0
torque_limit = 10.0
inertia = mass*length*length

pendulum = PendulumPlant(mass=mass,
                         length=length,
                         damping=damping,
                         gravity=gravity,
                         coulomb_fric=coulomb_fric,
                         inertia=inertia,
                         torque_limit=torque_limit)

sim = Simulator(plant=pendulum)

csv_path = "../../../../data/trajectories/ilqr/trajectory.csv"
data_dict = prepare_trajectory(csv_path)

controller = OpenLoopController(data_dict)

controller.set_goal([np.pi, 0])

trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
dt = trajectory[1][0] - trajectory[0][0]
t_final = trajectory[-1][0]

T, X, U = sim.simulate_and_animate(t0=0.0,
                                   x0=[0.0, 0.0],
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
