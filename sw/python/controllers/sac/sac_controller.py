# Other imporst
from stable_baselines import SAC
import numpy as np
import yaml
import os
from pathlib import Path

# Set path for local imports
import site
site.addsitedir('../..')

# Local imports
from utilities.abstract import AbstractController

# default parameters, can be changed
model_path = os.path.join(Path(__file__).parents[4], 'data/models/sac_model.zip')
params_path = os.path.join(Path(__file__).parents[4], 'data/models/sac_parameters.yaml')


class SacController(AbstractController):
    def __init__(self, model_path=model_path, params_path=params_path):

        self.model = SAC.load(model_path)
        with open(params_path, 'r') as fle:
            self.params = yaml.safe_load(fle)

    def get_control_output(self, meas_pos, meas_vel, meas_tau, meas_time):
        observation = np.squeeze(np.array([meas_pos, meas_vel]))
        des_tau, _states = self.model.predict(observation)
        des_tau *= float(self.params['torque_limit'])
        
        # since this is a pure torque controller, set pos_des and vel_des to None
        des_pos = None
        des_vel = None
        
        return des_pos, des_vel, des_tau

