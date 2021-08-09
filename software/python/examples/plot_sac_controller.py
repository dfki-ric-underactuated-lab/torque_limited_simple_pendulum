import os
import argparse

# Local imports
from simple_pendulum.controllers.sac.sac_controller import SacController
from simple_pendulum.utilities.plot_policy import plot_policy
from simple_pendulum.model.parameters import get_params

parser = argparse.ArgumentParser()
parser.add_argument('--model_path', default=None)
parser.add_argument('--params_path', default=None)
args = parser.parse_args()

# get the controller
if args.model_path is None:
    controller = SacController()
else:
    model_path = os.path.join(os.getcwd(), args.model_path)
    params_path = os.path.join(os.getcwd(), args.params_path)
    params = get_params(params_path)
    controller = SacController(model_path=args.model_path,
                               params_path=args.params_path)

plot_policy(controller,
            velocity_range=[-8, 8],
            samples_per_dim=100,
            plotstyle="3d")
