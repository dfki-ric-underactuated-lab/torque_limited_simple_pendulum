# Other imports
import numpy as np

# Local imports
from controllers.energy_shaping.energy_shaping_controller import EnergyShapingController
from utilities.plot_policy import plot_policy

mass = 0.57288
length = 0.5
damping = 0.05
gravity = 9.81
coulomb_fric = 0.0
torque_limit = 1.0
inertia = mass*length*length

controller = EnergyShapingController(mass,
                                     length,
                                     damping,
                                     gravity)
controller.set_goal([np.pi, 0])

plot_policy(controller,
            velocity_range=[-2, 2],
            samples_per_dim=100,
            plotstyle="3d")
