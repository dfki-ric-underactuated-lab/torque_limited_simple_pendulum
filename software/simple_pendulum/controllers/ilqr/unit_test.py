"""
Unit Tests
==========
"""


import unittest
import numpy as np

from simple_pendulum.model.pendulum_plant import PendulumPlant
from simple_pendulum.simulation.simulation import Simulator
from simple_pendulum.controllers.ilqr.iLQR_MPC_controller import iLQRMPCController


class Test(unittest.TestCase):

    epsilon = 0.2

    def test_0_iLQR_MPC_swingup_nx2(self):
        mass = 0.57288
        length = 0.5
        damping = 0.15
        gravity = 9.81
        coulomb_fric = 0.0
        torque_limit = 10.0
        inertia = mass*length*length

        pendulum = PendulumPlant(mass=mass,
                                 length=length,
                                 damping=damping,
                                 gravity=gravity,
                                 coulomb_fric=coulomb_fric,
                                 inertia=inertia,
                                 torque_limit=torque_limit)

        sim = Simulator(plant=pendulum)
        n_x = 2

        dt = 0.02
        t_final = 10.0
        x0 = np.array([0.0, 0.0])
        x0_sim = x0.copy()
        goal = np.array([np.pi, 0])

        controller = iLQRMPCController(mass=mass,
                                       length=length,
                                       damping=damping,
                                       coulomb_friction=coulomb_fric,
                                       gravity=gravity,
                                       inertia=inertia,
                                       dt=dt,
                                       n=50,  # horizon size
                                       max_iter=1,
                                       break_cost_redu=1e-1,
                                       sCu=30.0,
                                       sCp=10.0,
                                       sCv=1.0,
                                       sCen=1.0,
                                       fCp=10.0,
                                       fCv=1.0,
                                       fCen=80.0,
                                       dynamics="runge_kutta",
                                       n_x=n_x)

        controller.set_goal(goal)
        controller.init(x0=x0)

        T, X, U = sim.simulate(t0=0.0,
                               x0=x0_sim,
                               tf=t_final,
                               dt=dt,
                               controller=controller,
                               integrator="runge_kutta")

        self.assertIsInstance(T, list)
        self.assertIsInstance(X, list)
        self.assertIsInstance(U, list)

        swingup_success = True
        if np.abs((X[-1][0] % (2*np.pi)) - np.pi) > self.epsilon:
            if np.abs(X[-1][1]) > self.epsilon:
                swingup_success = False
                print("ilqr MPC Controller did not swingup",
                      "final state: ", X[-1])

        self.assertTrue(swingup_success)

    def test_1_iLQR_MPC_swingup_nx3(self):
        mass = 0.57288
        length = 0.5
        damping = 0.15
        gravity = 9.81
        coulomb_fric = 0.0
        torque_limit = 10.0
        inertia = mass*length*length

        pendulum = PendulumPlant(mass=mass,
                                 length=length,
                                 damping=damping,
                                 gravity=gravity,
                                 coulomb_fric=coulomb_fric,
                                 inertia=inertia,
                                 torque_limit=torque_limit)

        sim = Simulator(plant=pendulum)
        n_x = 3

        dt = 0.02
        t_final = 10.0
        x0 = np.array([0.0, 0.0])
        x0_sim = x0.copy()
        goal = np.array([np.pi, 0])

        controller = iLQRMPCController(mass=mass,
                                       length=length,
                                       damping=damping,
                                       coulomb_friction=coulomb_fric,
                                       gravity=gravity,
                                       inertia=inertia,
                                       dt=dt,
                                       n=50,  # horizon size
                                       max_iter=1,
                                       break_cost_redu=1e-1,
                                       sCu=1.0,
                                       sCp=10.0,
                                       sCv=1.0,
                                       sCen=1.0,
                                       fCp=10.0,
                                       fCv=1.0,
                                       fCen=80.0,
                                       dynamics="runge_kutta",
                                       n_x=n_x)

        controller.set_goal(goal)
        controller.init(x0=x0)

        T, X, U = sim.simulate(t0=0.0,
                               x0=x0_sim,
                               tf=t_final,
                               dt=dt,
                               controller=controller,
                               integrator="runge_kutta")

        self.assertIsInstance(T, list)
        self.assertIsInstance(X, list)
        self.assertIsInstance(U, list)

        swingup_success = True
        if np.abs((X[-1][0] % (2*np.pi)) - np.pi) > self.epsilon:
            if np.abs(X[-1][1]) > self.epsilon:
                swingup_success = False
                print("ilqr MPC Controller did not swingup",
                      "final state: ", X[-1])

        self.assertTrue(swingup_success)
