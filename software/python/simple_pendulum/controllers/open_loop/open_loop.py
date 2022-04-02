"""
Open Loop Controller
====================
"""


# Global imports
import numpy as np

# Local imports
from simple_pendulum.controllers.abstract_controller import AbstractController
from simple_pendulum.controllers.lqr.lqr_controller import LQRController


class OpenLoopController(AbstractController):
    """
    Controller acts on a predefined trajectory.
    """
    def __init__(self, data_dict):
        """
        Controller acts on a predefined trajectory.

        Parameters
        ----------
        data_dict : dictionary
            a dictionary containing the trajectory to follow
            should have the entries:
            data_dict["des_time_list"] : desired timesteps
            data_dict["des_pos_list"] : desired positions
            data_dict["des_vel_list"] : desired velocities
            data_dict["des_tau_list"] : desired torques
        """
        self.counter = 0
        self.traj_time = data_dict["des_time_list"]
        self.traj_pos = data_dict["des_pos_list"]
        self.traj_vel = data_dict["des_vel_list"]
        self.traj_tau = data_dict["des_tau_list"]

    def init(self, x0):
        self.counter = 0

    def set_goal(self, x):
        pass

    def get_control_output(self, meas_pos=None, meas_vel=None, meas_tau=None,
                           meas_time=None):
        """
        The function to read and send the entries of the loaded trajectory
        as control input to the simulator/real pendulum.

        Parameters
        ----------
        meas_pos : float, deault=None
            the position of the pendulum [rad]
        meas_vel : float, deault=None
            the velocity of the pendulum [rad/s]
        meas_tau : float, deault=None
            the meastured torque of the pendulum [Nm]
        meas_time : float, deault=None
            the collapsed time [s]

        Returns
        -------
        des_pos : float
            the desired position of the pendulum [rad]
        des_vel : float
            the desired velocity of the pendulum [rad/s]
        des_tau : float
            the torque supposed to be applied by the actuator [Nm]
        """

        des_pos = None
        des_vel = None
        des_tau = 0

        if self.counter < len(self.traj_time):
            des_pos = self.traj_pos[self.counter]
            des_vel = self.traj_vel[self.counter]
            des_tau = self.traj_tau[self.counter]

        self.counter += 1

        return des_pos, des_vel, des_tau


class OpenLoopAndLQRController(AbstractController):
    def __init__(self, data_dict, mass=1.0, length=0.5, damping=0.1,
                 gravity=9.81, torque_limit=np.inf):
        """
        Controller acts on a predefined trajectory. Switches to lqr control
        when in its region of attraction.

        Parameters
        ----------
        data_dict : dictionary
            a dictionary containing the trajectory to follow
            should have the entries:
            data_dict["des_time_list"] : desired timesteps
            data_dict["des_pos_list"] : desired positions
            data_dict["des_vel_list"] : desired velocities
            data_dict["des_tau_list"] : desired torques
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
        self.open_loop_controller = OpenLoopController(data_dict)
        self.lqr_controller = LQRController(mass=mass,
                                            length=length,
                                            damping=damping,
                                            gravity=gravity,
                                            torque_limit=torque_limit)
        self.active_controller = "none"

    def init(self, x0):
        self.lqr_controller.init(x0)
        self.open_loop_controller.init(x0)

    def set_goal(self, x):
        pass

    def get_control_output(self, meas_pos, meas_vel,
                           meas_tau=0, meas_time=0, verbose=False):
        """
        The function to read and send the entries of the loaded trajectory
        as control input to the simulator/real pendulum. Switches to lqr
        control when in its region of attraction.

        Parameters
        ----------
        meas_pos : float
            the position of the pendulum [rad]
        meas_vel : float
            the velocity of the pendulum [rad/s]
        meas_tau : float, deault=0
            the meastured torque of the pendulum [Nm]
        meas_time : float, deault=0
            the collapsed time [s]
        verbose : bool, default=False
            whether to print when the controller switches between
            preloaded trajectory and lqr

        Returns
        -------
        des_pos : float
            the desired position of the pendulum [rad]
        des_vel : float
            the desired velocity of the pendulum [rad/s]
        des_tau : float
            the torque supposed to be applied by the actuator [Nm]
        """
        des_pos, des_vel, des_tau = (self.lqr_controller.
                                     get_control_output(meas_pos, meas_vel))
        if des_tau is not None:
            if self.active_controller != "lqr":
                self.active_controller = "lqr"
                if verbose:
                    print("Switching to lqr control")
        else:
            if self.active_controller != "OpenLoop":
                self.active_controller = "OpenLoop"
                if verbose:
                    print("Switching to csv trajectory")
            des_pos, des_vel, des_tau = (self.open_loop_controller.
                                         get_control_output())

        return des_pos, des_vel, des_tau
