"""
LQR Controller
==============
"""


# Other imports
import numpy as np

# Local imports
from simple_pendulum.controllers.lqr.lqr import lqr
from simple_pendulum.controllers.abstract_controller import AbstractController
from simple_pendulum.model.pendulum_plant import PendulumPlant
try:
    from simple_pendulum.controllers.lqr.roa.sos import SOSequalityConstrained, SOSlineSearch
except ModuleNotFoundError:
    # drake not installed
    pass


class LQRController(AbstractController):
    """
    Controller which stabilizes the pendulum at its instable fixpoint.
    """
    def __init__(self, mass=1.0, length=0.5, damping=0.1, coulomb_fric=0.0,
                 gravity=9.81, torque_limit=np.inf, Q=np.diag((10, 1)), R=np.array([[1]]),
                 compute_RoA=False):
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
        coulomb_fric : float, default=0.0
            friction term, (independent of magnitude of velocity), unit: Nm
        gravity : float, default=9.81
            gravity (positive direction points down) [m/s^2]
        torque_limit : float, default=np.inf
            the torque_limit of the pendulum actuator
        Q : array-like, default=np.diag(10, 1)
            the state cost matrix, np.shape(Q) = (2,2)
        R : array-like, default=np.array([[1]])
            the control cost matrix, np.shape(R) = (1,1)
        compute_RoA : bool, default=False
            whether to compute the region of attraction of the LQR controller
            (requires drake)
        """
        self.m = mass
        self.len = length
        self.b = damping
        self.cf = coulomb_fric
        self.g = gravity
        self.torque_limit = torque_limit

        self.A = np.array([[0, 1],
                           [self.g/self.len, -self.b/(self.m*self.len**2.0)]])
        self.B = np.array([[0, 1./(self.m*self.len**2.0)]]).T
        self.Q = Q
        self.R = R

        self.K, self.S, _ = lqr(self.A, self.B, self.Q, self.R)

        # RoA calculation
        if compute_RoA:
            pendulum = PendulumPlant(mass=self.m,
                             length=self.len,
                             damping=self.b,
                             gravity=self.g,
                             coulomb_fric=self.cf,
                             inertia=self.m*self.len**2.0,
                             torque_limit=self.torque_limit)

            #self.rho, _ = SOSequalityConstrained(pendulum, self)
            self.rho, _ = SOSlineSearch(pendulum, self)
        else:
            self.rho = None

        self.clip_out = False

    def set_goal(self, x):
        pass

    def set_clip(self):
        self.clip_out = True

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
        u += np.sign(vel)*self.cf

        if not self.clip_out:
            if self.rho is not None:
                if y.dot(self.S.dot(y)) > self.rho:
                    u = None
            else:
                if np.abs(u) > self.torque_limit:
                    u = None

        else:
            u = np.clip(u, -self.torque_limit, self.torque_limit)

        # since this is a pure torque controller,
        # set des_pos and des_pos to None
        des_pos = None
        des_vel = None

        return des_pos, des_vel, u
