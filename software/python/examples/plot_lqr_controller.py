# Other imports
import numpy as np

# Local imports
from simple_pendulum.controllers.lqr.lqr_controller import LQRController
from simple_pendulum.utilities.plot_policy import plot_policy

mass = 0.57288
length = 0.5
damping = 0.05
gravity = 9.81
coulomb_fric = 0.0
torque_limit = 1.0
inertia = mass*length*length

controller = LQRController(mass=mass,
                           length=length,
                           damping=damping,
                           gravity=gravity,
                           torque_limit=torque_limit)
controller.set_goal([np.pi, 0])

plot_policy(controller,
            position_range=[np.pi-0.5, np.pi+0.5],
            velocity_range=[-2, 2],
            samples_per_dim=100,
            plotstyle="2d")
