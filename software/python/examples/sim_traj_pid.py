import numpy as np

from simple_pendulum.model.pendulum_plant import PendulumPlant
from simple_pendulum.simulation.simulation import Simulator
from simple_pendulum.utilities.process_data import (
    load_trajectory,
)
from simple_pendulum.utilities.plot import plot_trajectory

from simple_pendulum.controllers.pid.pid import PIDController

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

controller = PIDController(data_dict=data_dict, Kp=1.0, Ki=0.0, Kd=0.1)

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
