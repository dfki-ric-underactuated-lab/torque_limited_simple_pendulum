# Other imports
import os
import numpy as np

# Local imports
from simple_pendulum.controllers.energy_shaping.energy_shaping_controller import (
    EnergyShapingController,
)
from simple_pendulum.controllers.lqr.lqr_controller import LQRController
from simple_pendulum.controllers.sac.sac_controller import SacController
from simple_pendulum.controllers.ddpg.ddpg_controller import ddpg_controller
from simple_pendulum.controllers.discrete_graph.graph_controller import GraphController
from simple_pendulum.analysis.plot_policy import plot_policy
from simple_pendulum.model.parameters import get_params

# con = "energy_shaping"
# con = "lqr"
# con = "sac"
# con = "ddpg"
con = "graph_controller"

mass = 0.57288
length = 0.5
damping = 0.1
gravity = 9.81
coulomb_fric = 0.0
torque_limit = 1.5
inertia = mass * length * length

position_range = [-np.pi, np.pi]
velocity_range = [-10, 10]
samples_per_dim = 100
plotstyle = "2d"

if con == "energy_shaping":
    controller = EnergyShapingController(
        mass, length, damping, gravity, torque_limit=torque_limit
    )
    controller.set_goal([np.pi, 0])

if con == "lqr":
    controller = LQRController(
        mass=mass,
        length=length,
        damping=damping,
        gravity=gravity,
        torque_limit=torque_limit,
    )

    controller.set_goal([np.pi, 0])
    position_range = [np.pi - 0.5, np.pi + 0.5]
    velocity_range = [-2, 2]
    plotstyle = "2d"

if con == "sac":
    # model_path = "../../../data/models/sac_model.zip"
    model_path = "log_data/sac_training/best_model/best_model.zip"

    controller = SacController(
        model_path=model_path,
        torque_limit=torque_limit,
        use_symmetry=True,
        state_representation=3,
    )

elif con == "ddpg":

    # model_path = "../../../data/models/ddpg_model/actor"
    model_path = "log_data/ddpg_training/actor.keras"

    controller = ddpg_controller(
        model_path=model_path, torque_limit=torque_limit, state_representation=3
    )

elif con == "graph_controller":
    # policy_path = "../../../data/models/dijkstra_723600.csv"
    # policy_path = "../../../data/models/dijkstra_18360.csv"
    policy_path = "../../../data/models/graph/L0.5/dijkstra_36360.csv"
    graph_dt = 0.1
    graph_pos_res = np.deg2rad(1.0)
    graph_vel_res = graph_pos_res / graph_dt
    graph_vel_max = 50 * graph_vel_res

    controller = GraphController(
        graph_csv_path=policy_path,
        pos_res=graph_pos_res,
        vel_res=graph_vel_res,
        vel_max=graph_vel_max,
    )

plot_policy(
    controller,
    position_range=position_range,
    velocity_range=velocity_range,
    samples_per_dim=samples_per_dim,
    plotstyle=plotstyle,
    save_path=os.path.join("log_data/plot_controller/", con),
    # save_path=None,
    show=False,
)
