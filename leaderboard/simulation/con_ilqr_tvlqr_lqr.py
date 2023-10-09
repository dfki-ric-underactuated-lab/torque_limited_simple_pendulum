import numpy as np
from functools import partial

from simple_pendulum.trajectory_optimization.ilqr.ilqr import iLQR_Calculator
from simple_pendulum.trajectory_optimization.ilqr.pendulum import (
    pendulum_discrete_dynamics_euler,
    pendulum_discrete_dynamics_rungekutta,
    pendulum_swingup_stage_cost,
    pendulum_swingup_final_cost,
    pendulum3_discrete_dynamics_euler,
    pendulum3_discrete_dynamics_rungekutta,
    pendulum3_swingup_stage_cost,
    pendulum3_swingup_final_cost,
)
from simple_pendulum.controllers.tvlqr.tvlqr import TVLQRController
from simple_pendulum.controllers.lqr.lqr_controller import LQRController
from simple_pendulum.controllers.combined_controller import CombinedController
from simple_pendulum.utilities.process_data import prepare_empty_data_dict

from sim_parameters import (
    mass,
    length,
    damping,
    gravity,
    coulomb_fric,
    inertia,
    dt,
    t_final,
    t0,
    x0,
    goal,
    integrator,
)

name = "ilqr_tvlqr_lqr"
leaderboard_config = {
    "csv_path": name + "/sim_swingup.csv",
    "name": name,
    "simple_name": "iLQR and TVLQR",
    "short_description": "iLQR trajectory stabilized with time-varying LQR.",
    "readme_path": f"readmes/{name}.md",
    "username": "fwiebe",
}

n_x = 2
n_u = 1

# swingup parameters
if n_x == 3:
    x0 = np.array([np.cos(x0[0]), np.sin(x0[0]), x0[1]])
    goal = np.array([np.cos(goal[0]), np.sin(goal[0]), goal[1]])

# ilqr parameters
N = int(5 / dt)  # int(t_final / dt)
max_iter = 100
regu_init = 100

cf = coulomb_fric
torque_limit = 1.5

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

dyn = partial(
    dyn_func, dt=dt, m=mass, l=length, b=damping, cf=cf, g=gravity, inertia=inertia
)
iLQR.set_discrete_dynamics(dyn)

# set costs
s_cost_func = pendulum_swingup_stage_cost
f_cost_func = pendulum_swingup_final_cost
s_cost = partial(
    s_cost_func,
    goal=goal,
    Cu=sCu,
    Cp=sCp,
    Cv=sCv,
    Cen=sCen,
    m=mass,
    l=length,
    b=damping,
    cf=cf,
    g=gravity,
)
f_cost = partial(
    f_cost_func,
    goal=goal,
    Cp=fCp,
    Cv=fCv,
    Cen=fCen,
    m=mass,
    l=length,
    b=damping,
    cf=cf,
    g=gravity,
)
iLQR.set_stage_cost(s_cost)
iLQR.set_final_cost(f_cost)

iLQR.init_derivatives()
iLQR.set_start(x0)

# computation
(x_trj, u_trj, cost_trace, regu_trace, redu_ratio_trace, redu_trace) = iLQR.run_ilqr(
    N=N,
    init_u_trj=None,
    init_x_trj=None,
    max_iter=max_iter,
    regu_init=regu_init,
    break_cost_redu=1e-6,
)
# save results
time = np.linspace(0, N - 1, N) * dt
if n_x == 3:
    TH = np.arctan2(x_trj.T[1], x_trj.T[0])
    THD = x_trj.T[2]
else:
    TH = x_trj.T[0]
    THD = x_trj.T[1]

data_dict = prepare_empty_data_dict(dt, N * dt)
data_dict["des_time"] = time
data_dict["des_pos"] = TH
data_dict["des_vel"] = THD
data_dict["des_tau"] = np.append(u_trj.T[0], 0.0)

controller1 = TVLQRController(
    data_dict=data_dict,
    mass=mass,
    length=length,
    damping=damping,
    gravity=gravity,
    torque_limit=torque_limit,
)

controller2 = LQRController(
    mass=mass,
    length=length,
    damping=damping,
    coulomb_fric=coulomb_fric,
    gravity=gravity,
    torque_limit=torque_limit,
    Q=np.diag([10, 1]),
    R=np.array([[1]]),
    compute_RoA=False,
)


def condition1(meas_pos, meas_vel, meas_tau, meas_time):
    return False


def condition2(meas_pos, meas_vel, meas_tau, meas_time):
    goal = np.asarray([np.pi, 0.0])
    delta_pos = meas_pos - goal[0]
    delta_pos_wrapped = (delta_pos + np.pi) % (2 * np.pi) - np.pi
    if np.abs(delta_pos_wrapped) < 0.1 and np.abs(meas_vel) < 0.1:
        return True
    else:
        return False


controller = CombinedController(
    controller1=controller1,
    controller2=controller2,
    condition1=condition1,
    condition2=condition2,
)
controller.set_goal(goal)
