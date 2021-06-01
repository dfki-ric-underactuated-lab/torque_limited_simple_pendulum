import numpy as np
import matplotlib.pyplot as plt
from functools import partial

from controllers.iLQR.iLQR import iLQR_Calculator
from controllers.iLQR.pendulum import pendulum_discrete_dynamics_euler, \
                                      pendulum_discrete_dynamics_rungekutta, \
                                      pendulum_swingup_stage_cost, \
                                      pendulum_swingup_final_cost


# pendulum parameters
mass = 0.57288  # 0.5
length = 0.5
damping = 0.15  # 0.15
gravity = 9.81
coulomb_friction = 0.0
torque_limit = 10.0
inertia = mass*length*length
n_x = 2
n_u = 1

# swingup parameters
x0 = np.array([0.0, 0.0])
dt = 0.01
goal = np.array([np.pi, 0])

# iLQR parameters
N = 1000
max_iter = 100
regu_init = 100

# cost function weights
sCu = 0.1
sCp = 0.0
sCv = 0.0
sCen = 0.0
fCp = 1000.0
fCv = 1.0
fCen = 0.0

iLQR = iLQR_Calculator(n_x=n_x, n_u=n_u)

# set dynamics
# dyn_func = pendulum_discrete_dynamics_euler
dyn_func = pendulum_discrete_dynamics_rungekutta
dyn = partial(dyn_func,
              dt=dt,
              m=mass,
              l=length,
              b=damping,
              cf=coulomb_friction,
              g=gravity)
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
csv_data = np.vstack((time, x_trj.T[0], x_trj.T[1],
                      np.append(u_trj.T[0], 0.0))).T
np.savetxt("../../../../data/trajectories/iLQR/trajectory.csv",
           csv_data, delimiter=',', header="time,pos,vel,torque", comments="")

# plot results
fig, ax = plt.subplots(3, 1, figsize=(18, 6), sharex="all")

ax[0].plot(dt*np.linspace(0, N, np.shape(x_trj)[0]), x_trj.T[0], label="theta")
ax[0].set_ylabel("angle [rad]")
ax[0].legend(loc="best")
ax[1].plot(dt*np.linspace(0, N, np.shape(x_trj)[0]),
           x_trj.T[1], label="theta dot")
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
