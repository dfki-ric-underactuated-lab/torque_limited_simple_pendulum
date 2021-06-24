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
from controllers.abstract_controller import AbstractController

# default parameters, can be changed
model_path = os.path.join(Path(__file__).parents[4], 'data/models/sac_model.zip')
params_path = os.path.join(Path(__file__).parents[4],
                           'data/parameters/sac_parameters.yaml')


class SacController(AbstractController):
    def __init__(self, model_path=model_path):

        self.model = SAC.load(model_path)

    def get_params(self, params_path):
        with open(params_path, 'r') as fle:
            params = yaml.safe_load(fle)

        return params

    def prepare_data(self, params):
        dt = params['dt']
        t = params['runtime']
        n = t/dt

        # create 4 empty numpy array, where measured data can be stored
        meas_pos_list = np.zeros(n)
        meas_vel_list = np.zeros(n)
        meas_tau_list = np.zeros(n)
        meas_time_list = np.zeros(n)

        return meas_pos_list, meas_vel_list, meas_tau_list, meas_time_list, \
               n, t

    def get_control_output(self, meas_pos, meas_vel, meas_tau, meas_time=0):
        observation = np.squeeze(np.array([meas_pos, meas_vel]))
        des_tau, _states = self.model.predict(observation)
        des_tau *= float(self.params['torque_limit'])
        
        # since this is a pure torque controller, set pos_des and vel_des to None
        des_pos = None
        des_vel = None
        
        return des_pos, des_vel, des_tau

#    def return_all(self):
#        return SacController()
