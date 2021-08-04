# Other imports
import numpy as np
import yaml
import os
import math
from pathlib import Path

# Set path for local imports
import site
site.addsitedir('../..')

# Local imports
from controllers.abstract_controller import AbstractController

# default parameters, can be changed
params_file = "sp_parameters_fftau.yaml"
params_path = os.path.join(Path(__file__).parents[4], 'data/parameters/' +
                           params_file)


class GravityCompController(AbstractController):
    def __init__(self):
        self.counter = 0
        self.u = 0
        self.data_dict = None
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

        data_dict = {"des_time_list": des_time_list,
                     "des_pos_list": des_pos_list,
                     "des_vel_list": des_vel_list,
                     "des_tau_list": des_tau_list,
                     "meas_time_list": meas_time_list,
                     "meas_pos_list": meas_pos_list,
                     "meas_vel_list": meas_vel_list,
                     "meas_tau_list": meas_tau_list,
                     "n": n,
                     "dt": dt,
                     "t": t}
        return data_dict

    def get_control_output(self, meas_pos, meas_vel, meas_tau, meas_time):
        g = -9.81                               # gravitational acceleration on earth
        m = 0.546                               # mass at the end of the pendulum
        l = 0.5                                 # length of the rod

        # Compute gravity torques
        des_tau = m*g*l*math.sin(meas_pos)

        # since this is a pure torque controller, set pos_des and vel_des to None
        des_pos = 0
        des_vel = 0

        return des_pos, des_vel, des_tau

