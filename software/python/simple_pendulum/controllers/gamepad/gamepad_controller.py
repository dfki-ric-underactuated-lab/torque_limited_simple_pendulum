"""
Gamepad Controller
==============
"""

# Other imports
import numpy as np

# Local imports
from simple_pendulum.controllers.abstract_controller import AbstractController
from simple_pendulum.controllers.gamepad.gamepad import GamePad


class GamepadController(AbstractController):
    """
    Controller actuates the pendulum with a gamepad
    """
    def __init__(self, torque_limit=5.0,
                 gamepad_name="Logitech Logitech RumblePad 2 USB"):
        """
        Controller actuates the pendulum with a gamepad

        Parameters
        ----------
        torque_limit : float, default=np.inf
            the torque_limit of the pendulum actuator
        """
        self.torque_limit = torque_limit
        self.GP = GamePad(gamepad_name)

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

        u = self.GP.read()*self.torque_limit

        # since this is a pure torque controller,
        # set des_pos and des_pos to None
        des_pos = None
        des_vel = None

        return des_pos, des_vel, u
