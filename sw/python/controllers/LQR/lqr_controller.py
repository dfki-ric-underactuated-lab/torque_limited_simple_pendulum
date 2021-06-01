import numpy as np

from controllers.LQR.lqr import lqr
from utilities.abstract_controller import AbstractController


class LQRController(AbstractController):
    def __init__(self, mass=1.0, length=0.5, damping=0.1,
                 gravity=9.81, torque_limit=np.inf):
        self.m = mass
        self.len = length
        self.b = damping
        self.g = gravity
        self.torque_limit = torque_limit

        self.A = np.array([[0, 1],
                           [self.g/self.len, -self.b/self.m/self.len**2.0]])
        self.B = np.array([[0, 0], [0, 1]])
        self.Q = np.diag((10, 1))
        self.R = np.array([[100, 0], [0, 1]])

        self.K, self.S, _ = lqr(self.A, self.B, self.Q, self.R)

    def set_goal(self, x):
        pass

    def get_control_output(self, meas_pos, meas_vel,
                           meas_tau=0, meas_time=0, i=0):
        th = meas_pos[0] + np.pi
        th = (th + np.pi) % (2*np.pi) - np.pi
        y = np.asarray([th, meas_vel[0]])

        u = np.asarray(-self.K.dot(y))[0][1]

        if np.abs(u) > self.torque_limit:
            u = None

        # since this is a pure torque controller,
        # set pos_des and vel_des to None
        des_pos = None
        des_vel = None

        return des_pos, des_vel, u
