# Other imports
import numpy as np

# Set path for local imports
import site
site.addsitedir('../..')

# Local imports
from controllers.abstract_controller import AbstractController
from controllers.lqr.lqr_controller import LQRController


class OpenLoopController(AbstractController):
    def __init__(self):
        self.counter = 0
        self.u = 0

    def prepare(self, csv_path, n):
        # load trajectories from csv file
        trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
        des_time_list = trajectory.T[0].T       # desired time in s
        des_pos_list = trajectory.T[1].T        # desired position in radian
        des_vel_list = trajectory.T[2].T        # desired velocity in radian/s
        des_tau_list = trajectory.T[3].T        # desired torque in Nm

        # create 4 empty numpy array, where measured data can be stored
        meas_pos_list = np.zeros(n)
        meas_vel_list = np.zeros(n)
        meas_tau_list = np.zeros(n)
        meas_time_list = np.zeros(n)

        # avg desired dt
        dt = (des_time_list[n-1] - des_time_list[0])/n
        return des_time_list, des_pos_list, des_vel_list, des_tau_list, \
            meas_pos_list, meas_vel_list, meas_tau_list, meas_time_list, dt

    def set_goal(self, x):
        pass

    def get_control_output(self, des_time_list, des_pos_list, des_vel_list,
                           des_tau_list, meas_tau=0, meas_time=0):
        if self.counter < len(des_time_list):
            des_pos = des_pos_list[self.counter]
            des_vel = des_vel_list[self.counter]
            des_tau = des_tau_list[self.counter]
            self.counter += 1
        return des_pos, des_vel, des_tau


class OpenLoopAndLQRController(AbstractController):
    def __init__(self, csv_path="", mass=1.0, length=0.5,
                 damping=0.1, gravity=9.81, torque_limit=np.inf):

        self.open_loop_controller = OpenLoopController(csv_path)
        self.lqr_controller = LQRController(mass,
                                            length,
                                            damping,
                                            gravity,
                                            torque_limit)

        self.active_controller = "none"

    def set_goal(self, x):
        pass

    def get_control_output(self, meas_pos, meas_vel,
                           meas_tau=0, meas_time=0, i=0):
        des_pos, des_vel, u = (self.lqr_controller.
                               get_control_output(meas_pos, meas_vel))
        if u is not None:
            if self.active_controller != "lqr":
                self.active_controller = "lqr"
                print("Switching to lqr control")
        else:
            if self.active_controller != "OpenLoop":
                self.active_controller = "OpenLoop"
                print("Switching to csv trajectory")
            des_pos, des_vel, u = (self.open_loop_controller.
                                   get_control_output(i=i))

        return des_pos, des_vel, u
