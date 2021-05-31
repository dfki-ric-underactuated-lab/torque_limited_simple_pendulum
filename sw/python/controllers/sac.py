# imports
from utilities.abstract_controller import AbstractController
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

    def get_control_output(self, meas_pos, meas_vel, meas_tau, meas_time, i):
        observation = np.array([meas_pos, meas_vel])
        des_tau, _states = self.model.predict(observation)
        
        # since this is a pure torque controller, set pos_des and vel_des to None
        des_pos = None
        des_vel = None
        
        return des_pos, des_vel, des_tau

