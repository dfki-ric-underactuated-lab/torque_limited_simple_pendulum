# Other imports
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
                           'data/parameters/sp_parameters_sac.yaml')


class SacController(AbstractController):
    def __init__(self, model_path=model_path):

        self.model = SAC.load(model_path)
        self.params = None

    def get_params(self, params_path=params_path):
        with open(params_path, 'r') as fle:
            params = yaml.safe_load(fle)
            self.params = params
        return params

    def prepare_data(self, params):
        dt = params['dt']
        t = params['runtime']
        n = int(t/dt)

        # create 4 empty numpy array, where measured data can be stored
        des_time_list = np.zeros(n)
        des_pos_list = np.zeros(n)
        des_vel_list = np.zeros(n)
        des_tau_list = np.zeros(n)

        meas_time_list = np.zeros(n)
        meas_pos_list = np.zeros(n)
        meas_vel_list = np.zeros(n)
        meas_tau_list = np.zeros(n)
        vel_filt_list = np.zeros(n)

        data_dict = {"des_time_list": des_time_list,
                     "des_pos_list": des_pos_list,
                     "des_vel_list": des_vel_list,
                     "des_tau_list": des_tau_list,
                     "meas_time_list": meas_time_list,
                     "meas_pos_list": meas_pos_list,
                     "meas_vel_list": meas_vel_list,
                     "meas_tau_list": meas_tau_list,
                     "vel_filt_list": vel_filt_list,
                     "n": n,
                     "dt": dt,
                     "t": t}
        return data_dict

    def get_control_output(self, meas_pos, meas_vel, meas_tau, meas_time=0):

        observation = np.squeeze(np.array([meas_pos, meas_vel]))
        des_tau, _states = self.model.predict(observation)
        des_tau *= float(self.params['torque_limit'])
        
        # since this is a pure torque controller, set pos_des and vel_des to None
        des_pos = 0
        des_vel = 0
        
        return des_pos, des_vel, des_tau

#    def return_all(self):
#        return SacController()
