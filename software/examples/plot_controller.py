# Other imports
import numpy as np

# Local imports
from simple_pendulum.controllers.energy_shaping.energy_shaping_controller import EnergyShapingController
from simple_pendulum.controllers.lqr.lqr_controller import LQRController
from simple_pendulum.controllers.sac.sac_controller import SacController
from simple_pendulum.controllers.ddpg.ddpg_controller import ddpg_controller
from simple_pendulum.analysis.plot_policy import plot_policy
from simple_pendulum.model.parameters import get_params

con = "energy_shaping"
# con = "lqr"
# con = "sac"
# con = "ddpg"

mass = 0.57288
length = 0.5
damping = 0.15
gravity = 9.81
coulomb_fric = 0.0
torque_limit = 1.0
inertia = mass*length*length

if con == "energy_shaping":
    controller = EnergyShapingController(mass,
                                         length,
                                         damping,
                                         gravity)
    controller.set_goal([np.pi, 0])

    position_range = [-np.pi, np.pi]
    velocity_range = [-2, 2]
    samples_per_dim = 100
    plotstyle = "3d"

if con == "lqr":
    controller = LQRController(mass=mass,
                               length=length,
                               damping=damping,
                               gravity=gravity,
                               torque_limit=torque_limit)

    controller.set_goal([np.pi, 0])
    position_range = [np.pi-0.5, np.pi+0.5]
    velocity_range = [-2, 2]
    plotstyle = "2d"

if con == "sac":
    model_path = "../../../data/models/sac_model.zip"
    params_path = "../../../data/parameters/sp_parameters_sac.yaml"
    params = get_params(params_path)
    controller = SacController(model_path=model_path,
                               params_path=params_path)

    position_range = [-np.pi, np.pi]
    velocity_range = [-8, 8]
    samples_per_dim = 100
    plotstyle = "3d"

elif con == "ddpg":

    model_path = "../../../data/models/ddpg_model/actor"
    tl = 1.5
    state_rep = 3

    controller = ddpg_controller(model_path=model_path,
                                 torque_limit=tl,
                                 state_representation=state_rep)
    position_range = [-np.pi, np.pi]
    velocity_range = [-8, 8]
    samples_per_dim = 100
    plotstyle = "3d"

plot_policy(controller,
            position_range=position_range,
            velocity_range=velocity_range,
            samples_per_dim=samples_per_dim,
            plotstyle=plotstyle,
            save_path=None)
