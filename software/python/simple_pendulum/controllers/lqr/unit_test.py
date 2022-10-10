"""
Unit Tests
==========
"""


import unittest
import numpy as np

from simple_pendulum.model.pendulum_plant import PendulumPlant
from simple_pendulum.simulation.simulation import Simulator
from simple_pendulum.controllers.lqr.lqr_controller import LQRController


class Test(unittest.TestCase):

    epsilon = 0.01

    def test_0_LQR_stabilization(self):
        mass = 0.57288
        length = 0.5
        damping = 0.15
        gravity = 9.81
        coulomb_fric = 0.0
        torque_limit = 2.0
        inertia = mass*length*length

        pendulum = PendulumPlant(mass=mass,
                                 length=length,
                                 damping=damping,
                                 gravity=gravity,
                                 coulomb_fric=coulomb_fric,
                                 inertia=inertia,
                                 torque_limit=torque_limit)

        controller = LQRController(mass=mass,
                                   length=length,
                                   damping=damping,
                                   coulomb_fric=coulomb_fric,
                                   gravity=gravity,
                                   torque_limit=torque_limit,
                                   Q=np.diag([10, 1]),
                                   R=np.array([[1]]),
                                   compute_RoA=True)

        controller.set_goal([np.pi, 0])

        sim = Simulator(plant=pendulum)

        dt = 0.01
        t_final = 10.0

        T, X, U = sim.simulate(t0=0.0,
                               x0=[3.1, 0.0],
                               tf=t_final,
                               dt=dt,
                               controller=controller,
                               integrator="runge_kutta")

        self.assertIsInstance(T, list)
        self.assertIsInstance(X, list)
        self.assertIsInstance(U, list)

        stabilization_success = True
        if np.abs((X[-1][0] % (2*np.pi)) - np.pi) > self.epsilon:
            if np.abs(X[-1][1]) > self.epsilon:
                stabilization_success = False
                print("lqr Controller did not stabilize",
                      "final state: ", X[-1])

        self.assertTrue(stabilization_success)
