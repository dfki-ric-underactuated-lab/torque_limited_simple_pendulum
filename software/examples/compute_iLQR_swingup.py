import os
import numpy as np
import matplotlib.pyplot as plt
from functools import partial

try:
    import pydrake
    pydrake_available = True
    print("Using Pydrake")
except ModuleNotFoundError:
    pydrake_available = False
    print("Using Sympy")

if pydrake_available:
    from simple_pendulum.trajectory_optimization.ilqr.ilqr import iLQR_Calculator
else:
    from simple_pendulum.trajectory_optimization.ilqr.ilqr_sympy import iLQR_Calculator
from simple_pendulum.trajectory_optimization.ilqr.pendulum import (
                                        pendulum_discrete_dynamics_euler,
                                        pendulum_discrete_dynamics_rungekutta,
                                        pendulum_swingup_stage_cost,
                                        pendulum_swingup_final_cost,
                                        pendulum3_discrete_dynamics_euler,
                                        pendulum3_discrete_dynamics_rungekutta,
                                        pendulum3_swingup_stage_cost,
                                        pendulum3_swingup_final_cost)
from simple_pendulum.model.pendulum_plant import PendulumPlant
from simple_pendulum.simulation.simulation import Simulator
from simple_pendulum.utilities.process_data import prepare_trajectory
from simple_pendulum.controllers.open_loop.open_loop import OpenLoopController
# from simple_pendulum.controllers.tvlqr.tvlqr import TVLQRController

log_dir = "log_data/ilqr"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)

# pendulum parameters
mass = 0.57288
length = 0.5
damping = 0.10
gravity = 9.81
coulomb_friction = 0.0
torque_limit = 10.0
inertia = mass*length*length
integrator = "runge_kutta"
n_x = 2
n_u = 1

# swingup parameters
x0 = np.array([0.0, 0.0])
dt = 0.01
goal = np.array([np.pi, 0])
if n_x == 3:
    x0 = np.array([np.cos(x0[0]), np.sin(x0[0]), x0[1]])
    goal = np.array([np.cos(goal[0]), np.sin(goal[0]), goal[1]])

# ilqr parameters
N = 1000
max_iter = 100
regu_init = 100

# cost function weights
if n_x == 2:
    sCu = 0.1
    sCp = 0.0
    sCv = 0.0
    sCen = 0.0
    fCp = 1000.0
    fCv = 1.0
    fCen = 0.0
if n_x == 3:
    N = 300
    sCu = 10.0
    sCp = 5.0
    sCv = 2.0
    sCen = 0.0
    fCp = 2000.0
    fCv = 200.0
    fCen = 0.0

iLQR = iLQR_Calculator(n_x=n_x, n_u=n_u)

# set dynamics
if n_x == 2:
    if integrator == "euler":
        dyn_func = pendulum_discrete_dynamics_euler
    else:
        dyn_func = pendulum_discrete_dynamics_rungekutta
elif n_x == 3:
    if integrator == "euler":
        dyn_func = pendulum3_discrete_dynamics_euler
    else:
        dyn_func = pendulum3_discrete_dynamics_rungekutta

dyn = partial(dyn_func,
              dt=dt,
              m=mass,
              l=length,
              b=damping,
              cf=coulomb_friction,
              g=gravity,
              inertia=inertia)
iLQR.set_discrete_dynamics(dyn)

# set costs
s_cost_func = pendulum_swingup_stage_cost
f_cost_func = pendulum_swingup_final_cost
s_cost = partial(s_cost_func,
                 goal=goal,
                 Cu=sCu,
                 Cp=sCp,
                 Cv=sCv,
                 Cen=sCen,
                 m=mass,
                 l=length,
                 b=damping,
                 cf=coulomb_friction,
                 g=gravity)
f_cost = partial(f_cost_func,
                 goal=goal,
                 Cp=fCp,
                 Cv=fCv,
                 Cen=fCen,
                 m=mass,
                 l=length,
                 b=damping,
                 cf=coulomb_friction,
                 g=gravity)
iLQR.set_stage_cost(s_cost)
iLQR.set_final_cost(f_cost)

iLQR.init_derivatives()
iLQR.set_start(x0)

# computation
(x_trj, u_trj, cost_trace, regu_trace,
 redu_ratio_trace, redu_trace) = iLQR.run_ilqr(N=N,
                                               init_u_trj=None,
                                               init_x_trj=None,
                                               max_iter=max_iter,
                                               regu_init=regu_init,
                                               break_cost_redu=1e-6)
# save results
time = np.linspace(0, N-1, N)*dt
if n_x == 3:
    TH = np.arctan2(x_trj.T[1], x_trj.T[0])
    THD = x_trj.T[2]
else:
    TH = x_trj.T[0]
    THD = x_trj.T[1]
csv_data = np.vstack((time, TH, THD,
                      np.append(u_trj.T[0], 0.0))).T

csv_path = os.path.join(log_dir, "trajectory.csv")
np.savetxt(csv_path, csv_data, delimiter=',',
           header="time,pos,vel,torque", comments="")

# plot results
fig, ax = plt.subplots(3, 1, figsize=(18, 6), sharex="all")

if n_x == 2:
    ax[0].plot(dt*np.linspace(0, N, np.shape(x_trj)[0]),
               x_trj.T[0],
               label="theta")
if n_x == 3:
    ax[0].plot(dt*np.linspace(0, N, np.shape(x_trj)[0]),
               x_trj.T[0],
               label="cos(theta)")
    ax[0].plot(dt*np.linspace(0, N, np.shape(x_trj)[0]),
               x_trj.T[1],
               label="sin(theta)")
ax[0].set_ylabel("angle [rad]")
ax[0].legend(loc="best")
ax[1].plot(dt*np.linspace(0, N, np.shape(x_trj)[0]),
           x_trj.T[-1], label="theta dot")
ax[1].set_ylabel("angular velocity [rad/s]")
ax[1].legend(loc="best")
ax[2].plot(dt*np.linspace(0, N, np.shape(u_trj)[0]), u_trj, label="u")
ax[2].set_xlabel("time [s]")
ax[2].set_ylabel("input torque [Nm]")
ax[2].legend(loc="best")
plt.show()

plt.subplots(figsize=(10, 6))
plt.subplot(2, 2, 1)
plt.plot(cost_trace)
plt.xlabel('# Iteration')
plt.ylabel('Total cost')
plt.title('Cost trace')

plt.subplot(2, 2, 2)
delta_opt = (np.array(cost_trace) - cost_trace[-1])
plt.plot(delta_opt)
plt.yscale('log')
plt.xlabel('# Iteration')
plt.ylabel('Optimality gap')
plt.title('Convergence plot')

plt.subplot(2, 2, 3)
plt.plot(redu_ratio_trace)
plt.title('Ratio of actual reduction and expected reduction')
plt.ylabel('Reduction ratio')
plt.xlabel('# Iteration')

plt.subplot(2, 2, 4)
plt.plot(regu_trace)
plt.title('Regularization trace')
plt.ylabel('Regularization')
plt.xlabel('# Iteration')
plt.tight_layout()

plt.show()


#####################
# simulate the result
#####################

pendulum = PendulumPlant(mass=mass,
                         length=length,
                         damping=damping,
                         gravity=gravity,
                         coulomb_fric=coulomb_friction,
                         inertia=inertia,
                         torque_limit=torque_limit)

sim = Simulator(plant=pendulum)

data_dict = prepare_trajectory(csv_path)

controller = OpenLoopController(data_dict)
# controller = TVLQRController(data_dict=data_dict, mass=mass, length=length,
#                              damping=damping, gravity=gravity,
#                              torque_limit=torque_limit)

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
