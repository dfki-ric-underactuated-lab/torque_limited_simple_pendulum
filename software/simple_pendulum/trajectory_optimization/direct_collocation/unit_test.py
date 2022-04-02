"""
Unit Tests
==========
"""


import unittest
import numpy as np

from simple_pendulum.trajectory_optimization.direct_collocation.direct_collocation import DirectCollocationCalculator


class Test(unittest.TestCase):

    epsilon = 0.2

    def test_0_direct_collocation(self):
        # pendulum parameters
        mass = 0.57288
        length = 0.5
        damping = 0.15
        gravity = 9.81
        coulomb_fric = 0.0
        torque_limit = 2.0

        # swingup parameters
        x0 = [0.0, 0.0]
        goal = [np.pi, 0.0]

        # direct collocation parameters
        N = 21
        max_dt = 0.5

        dircal = DirectCollocationCalculator()
        dircal.init_pendulum(mass=mass,
                             length=length,
                             damping=damping,
                             gravity=gravity,
                             torque_limit=torque_limit)
        x_trajectory, dircol, result = dircal.compute_trajectory(
                                                         N=N,
                                                         max_dt=max_dt,
                                                         start_state=x0,
                                                         goal_state=goal)
        T, X, XD, U = dircal.extract_trajectory(x_trajectory,
                                                dircol,
                                                result,
                                                N=1000)

        self.assertIsInstance(T, np.ndarray)
        self.assertIsInstance(X, np.ndarray)
        self.assertIsInstance(XD, np.ndarray)
        self.assertIsInstance(U, np.ndarray)

        swingup_success = True
        if np.abs((X[-1] % (2*np.pi)) - np.pi) > self.epsilon:
            if np.abs(XD[-1]) > self.epsilon:
                swingup_success = False
                print("Direct Collocation Computation did not swingup",
                      "final state: ", X[-1], " ", XD[-1])

        self.assertTrue(swingup_success)
