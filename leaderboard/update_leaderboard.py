# Other imports
import os
import importlib
# import numpy as np
# import pandas

# Local imports
from simple_pendulum.model.pendulum_plant import PendulumPlant
from simple_pendulum.simulation.simulation import Simulator
from simple_pendulum.utilities.process_data import data_dict_from_TXU, save_trajectory

from sim_parameters import mass, length, damping, gravity, coulomb_fric, torque_limit, inertia, dt, t_final, t0, x0, goal, integrator
from compute_leaderboard import compute_leaderboard


recompute_leaderboard = False

# leaderboard = pandas.read_csv("data/leaderboard.csv")
# existing_list = list(leaderboard["Controller"])
existing_list = os.listdir("data")
for con in existing_list:
    if not os.path.exists(os.path.join("data", con, "sim_swingup.csv")):
        existing_list.remove(con)

for file in os.listdir("."):
    if file[:4] == "con_":
        if file[4:-3] in existing_list:
            print(f"Controller {file} already registered in leaderboad")
        else:
            print(f"Simulating new controller {file}")

            controller_arg = file[:-3]
            controller_name = controller_arg[4:]

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
            recompute_leaderboard = True

if recompute_leaderboard:
    compute_leaderboard()
