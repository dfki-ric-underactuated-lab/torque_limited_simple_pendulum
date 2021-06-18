# Other imports
import numpy as np

# Set path for local imports
import site
site.addsitedir('../..')

# Local imports
from controllers.LQR.lqr import lqr
from utilities.abstract import AbstractController

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
                           meas_tau=0, meas_time=0):

        if isinstance(meas_pos, (list, tuple, np.ndarray)):
            pos = meas_pos[0]
        else:
            pos = meas_pos

        if isinstance(meas_vel, (list, tuple, np.ndarray)):
            vel = meas_vel[0]
        else:
            vel = meas_vel

        th = pos + np.pi
        th = (th + np.pi) % (2*np.pi) - np.pi

        y = np.asarray([th, vel])

        # u = np.asarray(-self.K.dot(y))[0][1]

        if y.dot(np.asarray(self.S.dot(y))[0]) < 2.0: # old value:0.1
            u = -self.K.dot(y)
            u = np.asarray(u)[0][1]
        else:
            u = None

        # if np.abs(u) > self.torque_limit:
        #    u = None
#        else:
#            u = u * 1.2

        # since this is a pure torque controller,
        # set des_pos and des_pos to None
        des_pos = None
        des_vel = None

        return des_pos, des_vel, u
