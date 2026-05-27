# Other imports
import numpy as np
import matplotlib.pyplot as plt

# Local imports
from simple_pendulum.model.pendulum_plant import PendulumPlant
from simple_pendulum.simulation.simulation import Simulator
from simple_pendulum.controllers.ilqr.iLQR_MPC_controller_cpp import (
    iLQRMPCcppController,
)
from simple_pendulum.utilities.process_data import save_trajectoryTXU
from simple_pendulum.utilities.plot import plot_trajectory

mass = 0.57288
length = 0.5
damping = 0.1
gravity = 9.81
coulomb_fric = 0.0
torque_limit = 10.0
inertia = mass * length * length

pendulum = PendulumPlant(
    mass=mass,
    length=length,
    damping=damping,
    gravity=gravity,
    coulomb_fric=coulomb_fric,
    inertia=inertia,
    torque_limit=torque_limit,
)

sim = Simulator(plant=pendulum)

dt = 0.01
t_final = 10.0
x0 = np.array([0.0, 0.0])
x0_sim = x0.copy()
goal = np.array([np.pi, 0])

controller = iLQRMPCcppController(
    mass=mass,
    length=length,
    damping=damping,
    coulomb_friction=coulomb_fric,
    gravity=gravity,
    inertia=inertia,
    dt=dt,
    n=200,  # horizon size
    max_iter=3,
    break_cost_redu=1e-1,
    sCu=50.0,
    sCp=10.0,
    sCv=1.0,
    sCen=1.0,
    fCp=10.0,
    fCv=1.0,
    fCen=80.0,
    dynamics="runge_kutta",
)


controller.set_goal(goal)
# controller.load_initial_guess(filepath="../../../data/trajectories/ilqr/trajectory.csv")
controller.init(x0=x0_sim)

T, X, U = sim.simulate_and_animate(
    t0=0.0,
    x0=x0_sim,
    tf=t_final,
    dt=dt,
    controller=controller,
    integrator="runge_kutta",
)

# save_trajectoryTXU("log_data/sim_sac/trajectory.csv", T, X, U)
plot_trajectory(T, X, U, save_to=None, show=True)
