"""
Gravity Compensation Controller
===============================
"""


# Other imports
import math

# Local imports
from simple_pendulum.controllers.abstract_controller import AbstractController


class GravityCompController(AbstractController):
    def __init__(self, params):
        self.counter = 0
        self.u = 0
        self.g = params['gravity']        # gravitational acceleration on earth
        self.m = params['mass']               # mass at the end of the pendulum
        self.l = params['length']                           # length of the rod

    def get_control_output(self, meas_pos, meas_vel, meas_tau, meas_time):
        # compensate gravity with input torque
        des_tau = self.m * self.g * self.l * math.sin(meas_pos)

        # since this is a pure torque controller, set pos_des and vel_des to 0
        des_pos = 0
        des_vel = 0

        return des_pos, des_vel, des_tau
