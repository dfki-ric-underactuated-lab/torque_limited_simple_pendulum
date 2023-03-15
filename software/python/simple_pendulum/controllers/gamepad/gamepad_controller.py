"""
Gamepad Controller
==============
"""

# Other imports
import numpy as np

# Local imports
from simple_pendulum.controllers.abstract_controller import AbstractController
from simple_pendulum.controllers.lqr.lqr_controller import LQRController
from simple_pendulum.controllers.gamepad.gamepad import GamePad


class GamepadController(AbstractController):
    """
    Controller actuates the pendulum with a gamepad
    """
    def __init__(self, torque_limit=1.0,
                 gamepad_name="Logitech Logitech RumblePad 2 USB",
                 mass=1.0, length=0.5, damping=0.1, coulomb_fric=0.0,
                 gravity=9.81, lqr_torque_limit=2.0, Q=np.diag((10, 1)), R=np.array([[1]]),
                 dt=0.005,
                 rumble=False, max_vel=10.0, eps=[0.15, 0.05]):
        """
        Controller actuates the pendulum with a gamepad

        Parameters
        ----------
        torque_limit : float, default=1.0
            the torque_limit of the pendulum actuator
        """
        self.torque_limit = torque_limit
        self.GP = GamePad(gamepad_name, dt)

        self.lqr_con = LQRController(mass=mass,
                                     length=length,
                                     damping=damping,
                                     coulomb_fric=coulomb_fric,
                                     gravity=gravity,
                                     torque_limit=torque_limit,
                                     Q=Q,
                                     R=R)
        self.lqr_con.set_goal([np.pi, 0.0])

        self.swingup_time = None

        self.dt = dt
        self.rumble = rumble
        self.max_vel = max_vel
        self.eps = eps

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

        th = meas_pos + np.pi
        th = (th + np.pi) % (2*np.pi) - np.pi

        if (self.swingup_time is None and
            np.abs(th) < self.eps[0] and
            np.abs(meas_vel) < self.eps[1]):
            self.swingup_time = meas_time
            print("Swingup successful! Time: ", self.swingup_time)


        cmd = self.GP.read()
        u = cmd[0]*self.torque_limit

        u_lqr = self.lqr_con.get_control_output(meas_pos, meas_vel,
                                                meas_tau, meas_time)[2]

        if cmd[1] and cmd[2]:
            if u_lqr is None:
                u = 0.0
            else:
                u = u_lqr
        elif self.rumble:
            self.GP.stop_rumble()
            if u_lqr is not None:
                self.GP.rumble()

        if np.abs(meas_vel) > self.max_vel:
            u = 0.0

        # since this is a pure torque controller,
        # set des_pos and des_pos to None
        des_pos = None
        des_vel = None

        return des_pos, des_vel, u

    def get_swingup_time(self):
        return self.swingup_time
