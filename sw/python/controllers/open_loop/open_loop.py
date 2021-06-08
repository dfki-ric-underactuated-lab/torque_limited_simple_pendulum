import numpy as np

from utilities.abstract_controller import AbstractController
from controllers.LQR.lqr_controller import LQRController


class OpenLoopController(AbstractController):
    def __init__(self, csv_path):

        trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
        self.pos_traj = trajectory.T[1].T
        self.vel_traj = trajectory.T[2].T
        self.u_traj = trajectory.T[3].T

        self.counter = 0
        self.u = 0

    def set_goal(self, x):
        pass

    def get_control_output(self, meas_pos=0, meas_vel=0,
                           meas_tau=0, meas_time=0):
        if self.counter < len(self.u_traj):
            self.pos = self.u_traj[self.counter]
            self.vel = self.u_traj[self.counter]
            self.u = self.u_traj[self.counter]
            self.counter += 1
        return self.pos, self.vel, self.u


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
            if self.active_controller != "LQR":
                self.active_controller = "LQR"
                print("Switching to LQR control")
        else:
            if self.active_controller != "OpenLoop":
                self.active_controller = "OpenLoop"
                print("Switching to csv trajectory")
            des_pos, des_vel, u = (self.open_loop_controller.
                                   get_control_output(i=i))

        return des_pos, des_vel, u
