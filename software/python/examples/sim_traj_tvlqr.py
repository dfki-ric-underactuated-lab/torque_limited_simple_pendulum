import numpy as np
import matplotlib.pyplot as plt

from simple_pendulum.model.pendulum_plant import PendulumPlant
from simple_pendulum.simulation.simulation import Simulator
from simple_pendulum.utilities.process_data import (
    load_trajectory,
)
from simple_pendulum.controllers.open_loop.open_loop import OpenLoopController
from simple_pendulum.utilities.plot import plot_trajectory

from simple_pendulum.controllers.tvlqr.tvlqr import TVLQRController

# pendulum parameters
mass = 0.57288
length = 0.5
damping = 0.10
gravity = 9.81
coulomb_friction = 0.0
torque_limit = 10.0
inertia = mass * length * length
integrator = "runge_kutta"

csv_path = "../../../data/trajectories/ilqr/ilqr_trajectory.csv"
# csv_path = "../../../data/trajectories/ddp/ddp_trajectory.csv"
# csv_path = "../../../data/trajectories/direct_collocation/dircol_trajectory.csv"

pendulum = PendulumPlant(
    mass=mass,
    length=length,
    damping=damping,
    gravity=gravity,
    coulomb_fric=coulomb_friction,
    inertia=inertia,
    torque_limit=torque_limit,
)

sim = Simulator(plant=pendulum)

data_dict = load_trajectory(csv_path)

controller = TVLQRController(
    data_dict=data_dict,
    mass=mass,
    length=length,
    damping=damping,
    gravity=gravity,
    torque_limit=torque_limit,
)

controller.set_goal([np.pi, 0])

dt = data_dict["des_time"][1] - data_dict["des_time"][0]
t_final = data_dict["des_time"][-1]

T, X, U = sim.simulate_and_animate(
    t0=0.0,
    x0=[0.0, 0.0],
    tf=10.0,
    dt=dt,
    controller=controller,
    integrator="runge_kutta",
    phase_plot=False,
    save_video=False,
)

plot_trajectory(T, X, U, save_to=None, show=True)

# fig, ax = plt.subplots(3, 1, figsize=(18, 6), sharex="all")
#
# ax[0].plot(T, np.asarray(X).T[0], label="theta")
# ax[0].set_ylabel("angle [rad]")
# ax[0].legend(loc="best")
# ax[1].plot(T, np.asarray(X).T[1], label="theta dot")
# ax[1].set_ylabel("angular velocity [rad/s]")
# ax[1].legend(loc="best")
# ax[2].plot(T, U, label="u")
# ax[2].set_xlabel("time [s]")
# ax[2].set_ylabel("input torque [Nm]")
# ax[2].legend(loc="best")
# plt.show()
#
