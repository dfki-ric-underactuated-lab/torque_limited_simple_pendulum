import numpy as np

from simple_pendulum.trajectory_optimization.direct_collocation.direct_collocation import (
    DirectCollocationCalculator,
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
    torque_limit,
    inertia,
    dt,
    t_final,
    t0,
    x0,
    goal,
    integrator,
)

name = "dircol_tvlqr_lqr"
leaderboard_config = {
    "csv_path": "data/" + name + "/sim_swingup.csv",
    "name": name,
    "simple_name": "Direct collocation and TVLQR",
    "short_description": "Direct collocation trajectory stabilized with time-varying LQR.",
    "readme_path": f"readmes/{name}.md",
    "username": "fwiebe",
}

# direct collocation parameters
N = 21
max_dt = 0.5

torque_limit = 1.5

####################
# Compute trajectory
####################


dircal = DirectCollocationCalculator()
dircal.init_pendulum(
    mass=mass,
    length=length,
    damping=damping,
    gravity=gravity,
    torque_limit=torque_limit,
)
x_trajectory, dircol, result = dircal.compute_trajectory(
    N=N, max_dt=max_dt, start_state=x0, goal_state=goal
)
T, X, XD, U = dircal.extract_trajectory(
    x_trajectory, dircol, result, N=int(x_trajectory.end_time() / dt)
)


# save results
data_dict = prepare_empty_data_dict(dt, t_final)
data_dict["des_time"] = T
data_dict["des_pos"] = X
data_dict["des_vel"] = XD
data_dict["des_tau"] = U

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
