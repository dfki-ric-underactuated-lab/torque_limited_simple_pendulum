"""
Unit Tests
==========
"""


import unittest
import numpy as np

from simple_pendulum.model.pendulum_plant import PendulumPlant


class Test(unittest.TestCase):
    MASSES = [0.01, 0.1, 1.0, 10.0, 100.0]
    LENGTHS = [0.01, 0.1, 1.0, 10.0, 100.0]
    DAMPINGS = [0.0, 0.01, 0.1, 1.0]
    GRAVITY = [0.0, 9.81]
    CFRICS = [0.0, 0.01, 0.1, 1.0]
    TLIMITS = [1.0, 2.0, 3.0, 5.0, np.inf]

    iterations = 10
    epsilon = 1e-4

    max_angle = 5.0
    max_vel = 5.0
    max_tau = 5.0

    def test_0_kinematics(self):
        """
        Unit test for pendulum kinematics
        """
        for length in self.LENGTHS:
            pendulum = PendulumPlant(length=length)
            for _ in range(self.iterations):
                angle = (np.random.rand() - 0.5) * 2*self.max_angle
                self.assertIsInstance(angle, float)
                ee_pos = pendulum.forward_kinematics(angle)[0]
                self.assertIsInstance(ee_pos, list)
                ret_angle = pendulum.inverse_kinematics(ee_pos)
                self.assertIsInstance(ret_angle, float)
                diff = np.abs((angle % (2*np.pi)) - (ret_angle % (2*np.pi)))
                kinematics_test_passed = True
                if diff > self.epsilon:
                    kinematics_test_passed = False
                    print("Kinematics test failed. \
                          length {}, \
                          angle {}, \
                          ee_pos {}, \
                          returned angle {}".format(length,
                                                    angle,
                                                    ee_pos,
                                                    ret_angle))
                self.assertTrue(kinematics_test_passed)

    def test_1_dynamics(self):
        """
        Unit test for pendulum dynamics
        """
        for mass in self.MASSES:
            for length in self.LENGTHS:
                for damping in self.DAMPINGS:
                    for gravity in self.GRAVITY:
                        for coulomb_fric in self.CFRICS:
                            for torque_limit in self.TLIMITS:
                                inertia = mass*length*length
                                pendulum = PendulumPlant(
                                            mass=mass,
                                            length=length,
                                            damping=damping,
                                            gravity=gravity,
                                            coulomb_fric=coulomb_fric,
                                            inertia=inertia,
                                            torque_limit=torque_limit)
                                for _ in range(self.iterations):
                                    angle = (np.random.rand() - 0.5) * 2*self.max_angle
                                    vel = (np.random.rand() - 0.5) * 2*self.max_vel
                                    tau = (np.random.rand() - 0.5) * 2*self.max_tau
                                    state = [angle, vel]

                                    accn = pendulum.forward_dynamics(state, tau)
                                    ret_tau = pendulum.inverse_dynamics(state, accn)
                                    clipped_tau = np.clip(tau, -torque_limit, torque_limit)

                                    dynamics_test_passed = True
                                    if np.abs(clipped_tau - ret_tau) > self.epsilon:
                                        dynamics_test_passed = False
                                        print("Dynamics test failed.\
                                              mass {},\
                                              length {}, \
                                              damping {}, \
                                              gravity {}, \
                                              coulomb friction {}, \
                                              state {}, \
                                              torque {}, \
                                              returned torque {}".format(mass,
                                                                         length,
                                                                         damping,
                                                                         gravity,
                                                                         coulomb_fric,
                                                                         state,
                                                                         tau,
                                                                         ret_tau))
                                    self.assertTrue(dynamics_test_passed)

if __name__ == '__main__':
    unittest.main()
