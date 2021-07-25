# Local imports
from controllers.sac.sac_controller import SacController
from utilities.plot_policy import plot_policy

controller = SacController()

plot_policy(controller,
            velocity_range=[-8, 8],
            samples_per_dim=100,
            plotstyle="3d")
