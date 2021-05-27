# imports
from abstract_controller import AbstractController
from stable_baselines import SAC
import numpy as np
import yaml

# default parameters, can be changed
model_path = '../../data/models/sac_model.zip'
params_path = '../../data/models/sac_params.yaml'


class SacController(AbstractController):
    def __init__(self, model_path=model_path, params_path=params_path):

        self.model = SAC.load(model_path)
        with open(params_path, 'r') as fle:
            self.params = yaml.safe_load(fle)

    def get_control_output(self, pos, vel, tau, t, i):
        observation = np.array([pos, vel])
        tau_des, _states = self.model.predict(observation)
        
        # since this is a pure torque controller, set pos_des and vel_des to None
        pos_des = None
        vel_des = None
        
        return pos_des, vel_des, tau_des

