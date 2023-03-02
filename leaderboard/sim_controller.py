# Other imports
import os
import argparse
import importlib
import numpy as np
import matplotlib.pyplot as plt

# Local imports
from simple_pendulum.model.pendulum_plant import PendulumPlant
from simple_pendulum.simulation.simulation import Simulator
from simple_pendulum.utilities.process_data import data_dict_from_TXU, save_trajectory

from sim_parameters import mass, length, damping, gravity, coulomb_fric, torque_limit, inertia, dt, t_final, t0, x0, goal, integrator

parser = argparse.ArgumentParser()
parser.add_argument("controller", help="name of the controller to simulate")
controller_arg = parser.parse_args().controller
if controller_arg[-3:] == ".py":
    controller_arg = controller_arg[:-3]

controller_name = controller_arg[4:]
print(f"Simulating controller {controller_name}")

save_dir = f"data/{controller_name}"

imp = importlib.import_module(controller_arg)

controller = imp.controller

pendulum = PendulumPlant(mass=mass,
                         length=length,
                         damping=damping,
                         gravity=gravity,
                         coulomb_fric=coulomb_fric,
                         inertia=inertia,
                         torque_limit=torque_limit)

sim = Simulator(plant=pendulum)

T, X, U = sim.simulate(t0=t0,
                       x0=x0,
                       tf=t_final,
                       dt=dt,
                       controller=controller,
                       integrator=integrator)

data_dict = data_dict_from_TXU(T, X, U)
if not os.path.exists(save_dir):
    os.makedirs(save_dir)
save_trajectory(os.path.join(save_dir, "sim_swingup.csv"), data_dict)

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
