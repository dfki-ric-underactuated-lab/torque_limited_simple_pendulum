# Other Imports
import numpy as np
import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

# Local imports
from simple_pendulum.model.pendulum_plant import PendulumPlant
from simple_pendulum.simulation.simulation import Simulator
from simple_pendulum.controllers.sac.sac_controller import SacController
from simple_pendulum.utilities.process_data import save_trajectoryTXU
from simple_pendulum.utilities.plot import plot_trajectory

# get the simulator
torque_limit = 1.5
mass = 0.57288
length = 0.5
damping = 0.10
gravity = 9.81
coulomb_fric = 0.0
inertia = mass * length**2

pendulum = PendulumPlant(
    mass=mass,
    length=length,
    damping=damping,
    gravity=gravity,
    coulomb_fric=coulomb_fric,
    inertia=inertia,
    torque_limit=10.0,
)

sim = Simulator(plant=pendulum)

# get the controller
model_path = "../../../data/models/sac/L0.5/sac_model.zip"
# model_path = "log_data/sac_training/best_model/best_model"

controller = SacController(
    model_path=model_path,
    torque_limit=torque_limit,
    use_symmetry=False,
    state_representation=3,
)

# simulate
x0_sim = [0.0, 0.0]
dt = 0.01
t_final = 10
integrator = "runge_kutta"

T, X, U = sim.simulate_and_animate(
    t0=0.0, x0=x0_sim, tf=t_final, dt=dt, controller=controller, integrator=integrator
)
# save_trajectoryTXU("log_data/sim_sac/trajectory.csv", T, X, U)

plot_trajectory(T, X, U, save_to=None, show=True)
