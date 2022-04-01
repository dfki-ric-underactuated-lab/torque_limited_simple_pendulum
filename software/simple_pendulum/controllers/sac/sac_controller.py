"""
SAC Controller
==============
"""


# Other imports
import numpy as np
from stable_baselines3 import SAC

# Local imports
from simple_pendulum.controllers.abstract_controller import AbstractController


class SacController(AbstractController):
    """
    Controller which acts on a policy which has been learned with sac.
    """
    def __init__(self,
                 model_path,
                 torque_limit,
                 use_symmetry=True,
                 state_representation=2):
        """
        Controller which acts on a policy which has been learned with sac.

        Parameters
        ----------
        model_path : string
            path to the trained model in zip format
        torque_limit : float
            torque limit of the pendulum. The output of the model will be
            scaled with this number
        use_symmetry : bool
            whether to use the left/right symmetry of the pendulum
        """
        self.model = SAC.load(model_path)
        self.torque_limit = float(torque_limit)
        self.use_symmetry = bool(use_symmetry)
        self.state_representation = state_representation

        if state_representation == 2:
            # state is [th, th, vel]
            self.low = np.array([-6*2*np.pi, -20])
            self.high = np.array([6*2*np.pi, 20])
        elif state_representation == 3:
            # state is [cos(th), sin(th), vel]
            self.low = np.array([-1., -1., -8.])
            self.high = np.array([1., 1., 8.])

    def get_control_output(self, meas_pos, meas_vel, meas_tau=0, meas_time=0):
        """
        The function to compute the control input for the pendulum actuator

        Parameters
        ----------
        meas_pos : float
            the position of the pendulum [rad]
        meas_vel : float
            the velocity of the pendulum [rad/s]
        meas_tau : float
            the meastured torque of the pendulum [Nm]
            (not used)
        meas_time : float
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

        # map meas pos to [-np.pi, np.pi]
        meas_pos_mod = np.mod(pos + np.pi, 2 * np.pi) - np.pi
        # observation = np.squeeze(np.array([meas_pos_mod, vel]))
        observation = self.get_observation([meas_pos_mod, vel])

        if self.use_symmetry:
            observation[0] *= np.sign(meas_pos_mod)
            observation[1] *= np.sign(meas_pos_mod)
            des_tau, _states = self.model.predict(observation)
            des_tau *= np.sign(meas_pos_mod)
        else:
            des_tau, _states = self.model.predict(observation)
        des_tau *= self.torque_limit

        # since this is a pure torque controller,
        # set pos_des and vel_des to None
        des_pos = None
        des_vel = None

        return des_pos, des_vel, des_tau

    def get_observation(self, state):
        st = np.copy(state)
        st[1] = np.clip(st[1], self.low[-1], self.high[-1])
        if self.state_representation == 2:
            observation = np.array([obs for obs in st], dtype=np.float32)
        elif self.state_representation == 3:
            observation = np.array([np.cos(st[0]),
                                    np.sin(st[0]),
                                    st[1]],
                                   dtype=np.float32)

        return observation
