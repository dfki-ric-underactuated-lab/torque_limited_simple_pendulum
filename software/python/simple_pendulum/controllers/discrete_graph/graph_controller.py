"""
Graph Controller
====================
"""

# Global imports
import numpy as np

# Local imports
from simple_pendulum.controllers.abstract_controller import AbstractController
from simple_pendulum.controllers.discrete_graph.utils import (
    discretize_pos_vel,
    pos_vel_to_index,
    index_to_pos_vel,
)


class GraphController(AbstractController):
    """
    Controller acts on a predefined trajectory.
    """

    def __init__(self, graph_csv_path, pos_res, vel_res, vel_max):
        """
        Controller acts on a predefined trajectory.

        Parameters
        ----------
        data_dict : dictionary
            a dictionary containing the trajectory to follow
            should have the entries:
            data_dict["des_time"] : desired timesteps
            data_dict["des_pos"] : desired positions
            data_dict["des_vel"] : desired velocities
            data_dict["des_tau"] : desired torques
        """
        self.policy_index_map = np.loadtxt(graph_csv_path, delimiter=",", skiprows=1)
        self.pos_res = pos_res
        self.vel_res = vel_res
        self.vel_max = vel_max

    def init(self, x0):
        pass

    def set_goal(self, x):
        pass

    def get_control_output(
        self, meas_pos=None, meas_vel=None, meas_tau=None, meas_time=None
    ):
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
        # print(meas_time, meas_pos, meas_vel)
        p, v = discretize_pos_vel(meas_pos, meas_vel, self.pos_res, self.vel_res)
        p = np.clip(p, -np.pi, np.pi)
        v = np.clip(v, -self.vel_max + self.vel_res, self.vel_max - self.vel_res)
        ind = pos_vel_to_index(
            p,
            v,
            self.pos_res,
            -np.pi,
            np.pi,
            self.vel_res,
            -self.vel_max,
            self.vel_max,
        )
        if ind < (self.policy_index_map.shape[0] - 1):
            des_tau = self.policy_index_map[ind][1]
            des_ind = int(self.policy_index_map[ind][0])
            des_pos, des_vel = index_to_pos_vel(
                des_ind,
                self.pos_res,
                -np.pi,
                np.pi,
                self.vel_res,
                -self.vel_max,
                self.vel_max,
            )
        else:
            des_tau = 0.0
            des_pos = meas_pos
            des_vel = meas_vel

        # des_pos = None
        # des_vel = None
        # print(meas_pos, meas_vel, ind, des_ind, des_pos, des_vel, des_tau)
        # des_tau *= 2.0

        return des_pos, des_vel, des_tau
