"""
LQR Controller
==============
"""


# Other imports
import numpy as np

# Local imports
from simple_pendulum.controllers.lqr.lqr import lqr
from simple_pendulum.controllers.abstract_controller import AbstractController


class LQRController(AbstractController):
    """
    Controller which stabilizes the pendulum at its instable fixpoint.
    """
    def __init__(self, mass=1.0, length=0.5, damping=0.1,
                 gravity=9.81, torque_limit=np.inf):
        """
        Controller which stabilizes the pendulum at its instable fixpoint.

        Parameters
        ----------
        mass : float, default=1.0
            mass of the pendulum [kg]
        length : float, default=0.5
            length of the pendulum [m]
        damping : float, default=0.1
            damping factor of the pendulum [kg m/s]
        gravity : float, default=9.81
            gravity (positive direction points down) [m/s^2]
        torque_limit : float, default=np.inf
            the torque_limit of the pendulum actuator
        """
        self.m = mass
        self.len = length
        self.b = damping
        self.g = gravity
        self.torque_limit = torque_limit

        self.A = np.array([[0, 1],
                           [self.g/self.len, -self.b/(self.m*self.len**2.0)]])
        self.B = np.array([[0, 1./(self.m*self.len**2.0)]]).T
        self.Q = np.diag((10, 1))
        self.R = np.array([[1]])

        self.K, self.S, _ = lqr(self.A, self.B, self.Q, self.R)

    def set_goal(self, x):
        pass

    def get_control_output(self, meas_pos, meas_vel,
                           meas_tau=0, meas_time=0):
        """
        The function to compute the control input for the pendulum actuator

        Parameters
        ----------
        meas_pos : float
            the position of the pendulum [rad]
        meas_vel : float
            the velocity of the pendulum [rad/s]
        meas_tau : float, default=0
            the meastured torque of the pendulum [Nm]
            (not used)
        meas_time : float, default=0
            the collapsed time [s]
            (not used)

        Returns
        -------
        des_pos : float
            the desired position of the pendulum [rad]
            (not used, returns None)
        des_vel : float
            the desired velocity of the pendulum [rad/s]
            (not used, returns None)
        des_tau : float
            the torque supposed to be applied by the actuator [Nm]
        """

        pos = float(np.squeeze(meas_pos))
        vel = float(np.squeeze(meas_vel))

        th = pos + np.pi
        th = (th + np.pi) % (2*np.pi) - np.pi

        y = np.asarray([th, vel])

        u = np.asarray(-self.K.dot(y))[0]

        if np.abs(u) > self.torque_limit:
            u = None

        # since this is a pure torque controller,
        # set des_pos and des_pos to None
        des_pos = None
        des_vel = None

        return des_pos, des_vel, u
