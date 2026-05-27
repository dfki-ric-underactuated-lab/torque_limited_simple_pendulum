import numpy as np
import matplotlib.pyplot as plt

from simple_pendulum.model.pendulum_plant import PendulumPlant
from simple_pendulum.simulation.simulation import Simulator
from simple_pendulum.controllers.ddpg.ddpg_controller import ddpg_controller
from simple_pendulum.utilities.process_data import save_trajectoryTXU
from simple_pendulum.utilities.plot import plot_trajectory

mass = 0.57288
length = 0.5
damping = 0.1
gravity = 9.81
coulomb_fric = 0.0
torque_limit = 1.5
inertia = mass * length * length

dt = 0.01
t_final = 10.0

pendulum = PendulumPlant(
    mass=mass,
    length=length,
    damping=damping,
    gravity=gravity,
    coulomb_fric=coulomb_fric,
    inertia=inertia,
    torque_limit=torque_limit,
)


model_path = "../../../data/models/ddpg_model/L0.5/actor.keras"
# model_path = "log_data/ddpg_training/actor.keras"
controller = ddpg_controller(
    model_path=model_path, torque_limit=torque_limit, state_representation=3
)


sim = Simulator(plant=pendulum)

T, X, U = sim.simulate_and_animate(
    t0=0.0,
    x0=[0.0, 0.0],
    tf=t_final,
    dt=dt,
    controller=controller,
    integrator="runge_kutta",
)
# save_trajectoryTXU("log_data/sim_ddpg/trajectory.csv", T, X, U)

plot_trajectory(T, X, U, save_to=None, show=True)
