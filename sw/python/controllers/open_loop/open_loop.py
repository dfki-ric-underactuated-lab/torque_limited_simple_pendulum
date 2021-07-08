# Set path for local imports
import site
site.addsitedir('../..')

# Local imports
from controllers.abstract_controller import *
from controllers.lqr.lqr_controller import LQRController


class OpenLoopController(AbstractOpenLoopController):
    def __init__(self, data_dict):
        self.counter = 0
        self.u = 0
        self.data_dict = data_dict
        self.traj_time = data_dict["des_time_list"]
        self.traj_pos = data_dict["des_pos_list"]
        self.traj_vel = data_dict["des_vel_list"]
        self.traj_tau = data_dict["des_tau_list"]

    def set_goal(self, x):
        pass

    def get_control_output(self):
        des_pos = None
        des_vel = None
        des_tau = None

        if self.counter < len(self.traj_time):
            des_pos = self.traj_pos[self.counter]
            des_vel = self.traj_vel[self.counter]
            des_tau = self.traj_tau[self.counter]
            self.counter += 1
        return des_pos, des_vel, des_tau


class OpenLoopAndLQRController(AbstractClosedLoopController):
    def __init__(self, params, data_dict):
        self.open_loop_controller = OpenLoopController(data_dict)
        self.lqr_controller = LQRController(params)
        self.active_controller = "none"

    def set_goal(self, x):
        pass

    def get_control_output(self, meas_pos, meas_vel,
                           meas_tau=0, meas_time=0, i=0):
        des_pos, des_vel, des_tau = (self.lqr_controller.
                                     get_control_output(meas_pos, meas_vel))
        if des_tau is not None:
            if self.active_controller != "lqr":
                self.active_controller = "lqr"
                print("Switching to lqr control")
        else:
            if self.active_controller != "OpenLoop":
                self.active_controller = "OpenLoop"
                print("Switching to csv trajectory")
            des_pos, des_vel, des_tau = (self.open_loop_controller.
                                         get_control_output(i=i))

        return des_pos, des_vel, des_tau
