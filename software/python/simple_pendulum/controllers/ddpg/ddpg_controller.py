"""
DDPG Controller
===============
"""

import numpy as np
import tensorflow as tf
from tensorflow.keras.models import load_model

from simple_pendulum.controllers.abstract_controller import AbstractController


class ddpg_controller(AbstractController):
    """
    DDPG controller class
    """

    def __init__(
        self,
        model_path,
        torque_limit,
        state_representation=3,
        damping=0.0,
        coulomb_friction=0.0,
    ):
        self.model = load_model(model_path)
        self._predict_fn = tf.function(lambda x: self.model(x, training=False))
        self.torque_limit = torque_limit
        self.state_representation = state_representation

        if state_representation == 2:
            # state is [th, th, vel]
            self.low = np.array([-6 * 2 * np.pi, -20])
            self.high = np.array([6 * 2 * np.pi, 20])
        elif state_representation == 3:
            # state is [cos(th), sin(th), vel]
            self.low = np.array([-1.0, -1.0, -8.0])
            self.high = np.array([1.0, 1.0, 8.0])

        self.damping = damping
        self.coulomb_friction = coulomb_friction

    def get_control_output(self, meas_pos, meas_vel, meas_tau=0, meas_time=0):

        pos = float(np.squeeze(meas_pos))
        vel = float(np.squeeze(meas_vel))

        state = np.array([pos, vel])
        observation = self.get_observation(state)
        control_output = self._predict_fn(np.atleast_2d(observation)).numpy()

        control_output *= self.torque_limit

        control_output += self.damping * float(np.squeeze(meas_vel))
        control_output += self.coulomb_friction * np.sign(float(np.squeeze(meas_vel)))

        control_output = np.clip(control_output, -self.torque_limit, self.torque_limit)

        return None, None, float(control_output)

    def get_observation(self, state):
        st = np.copy(state)
        st[1] = np.clip(st[1], self.low[-1], self.high[-1])
        if self.state_representation == 2:
            observation = np.array([obs for obs in st], dtype=np.float32)
        elif self.state_representation == 3:
            observation = np.array(
                [np.cos(st[0]), np.sin(st[0]), st[1]], dtype=np.float32
            )

        return observation
