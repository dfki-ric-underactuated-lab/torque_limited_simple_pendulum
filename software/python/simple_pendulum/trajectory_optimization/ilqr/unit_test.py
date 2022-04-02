"""
Unit Tests
==========
"""


import unittest
import numpy as np
from functools import partial

from simple_pendulum.trajectory_optimization.ilqr.ilqr import iLQR_Calculator
from simple_pendulum.trajectory_optimization.ilqr.ilqr_sympy import iLQR_Calculator as iLQR_Calculator_sympy

from simple_pendulum.trajectory_optimization.ilqr.pendulum import (
                                    pendulum_discrete_dynamics_rungekutta,
                                    pendulum_swingup_stage_cost,
                                    pendulum_swingup_final_cost,
                                    pendulum3_discrete_dynamics_rungekutta,
                                    pendulum3_swingup_stage_cost,
                                    pendulum3_swingup_final_cost
                                    )


class Test(unittest.TestCase):

    epsilon = 0.2

    def test_0_iLQR_computation_nx2(self):
        # pendulum parameters
        mass = 0.57288
        length = 0.5
        damping = 0.15
        gravity = 9.81
        coulomb_friction = 0.0
        inertia = mass*length*length
        n_x = 2
        n_u = 1

        # swingup parameters
        x0 = np.array([0.0, 0.0])
        dt = 0.01
        goal = np.array([np.pi, 0])

        # ilqr parameters
        N = 1000
        max_iter = 100
        regu_init = 100

        # cost function weights
        sCu = 0.1
        sCp = 0.0
        sCv = 0.0
        sCen = 0.0
        fCp = 1000.0
        fCv = 1.0
        fCen = 0.0

        iLQR = iLQR_Calculator(n_x=n_x, n_u=n_u)

        # set dynamics
        # dyn_func = pendulum_discrete_dynamics_euler
        dyn_func = pendulum_discrete_dynamics_rungekutta
        dyn = partial(dyn_func,
                      dt=dt,
                      m=mass,
                      l=length,
                      b=damping,
                      cf=coulomb_friction,
                      g=gravity,
                      inertia=inertia)
        iLQR.set_discrete_dynamics(dyn)

        # set costs
        s_cost_func = pendulum_swingup_stage_cost
        f_cost_func = pendulum_swingup_final_cost
        s_cost = partial(s_cost_func,
                         goal=goal,
                         Cu=sCu,
                         Cp=sCp,
                         Cv=sCv,
                         Cen=sCen,
                         m=mass,
                         l=length,
                         b=damping,
                         cf=coulomb_friction,
                         g=gravity)
        f_cost = partial(f_cost_func,
                         goal=goal,
                         Cp=fCp,
                         Cv=fCv,
                         Cen=fCen,
                         m=mass,
                         l=length,
                         b=damping,
                         cf=coulomb_friction,
                         g=gravity)
        iLQR.set_stage_cost(s_cost)
        iLQR.set_final_cost(f_cost)

        iLQR.init_derivatives()
        iLQR.set_start(x0)

        # computation
        (X, U, cost_trace, regu_trace,
         redu_ratio_trace, redu_trace) = iLQR.run_ilqr(N=N,
                                                       init_u_trj=None,
                                                       init_x_trj=None,
                                                       max_iter=max_iter,
                                                       regu_init=regu_init,
                                                       break_cost_redu=1e-6)

        self.assertIsInstance(X, np.ndarray)
        self.assertIsInstance(U, np.ndarray)

        stabilization_success = True
        if np.abs((X[-1][0] % (2*np.pi)) - np.pi) > self.epsilon:
            if np.abs(X[-1][1]) > self.epsilon:
                stabilization_success = False
                print("ilqr Computation (n_x=2) did not swingup",
                      "final state: ", X[-1])

        self.assertTrue(stabilization_success)

    def test_0_iLQR_computation_nx3(self):
        # pendulum parameters
        mass = 0.57288
        length = 0.5
        damping = 0.15
        gravity = 9.81
        coulomb_friction = 0.0
        inertia = mass*length*length
        n_x = 3
        n_u = 1

        # swingup parameters
        dt = 0.01
        x0 = np.array([0.0, 0.0])
        goal = np.array([np.pi, 0])
        x0 = np.array([np.cos(x0[0]), np.sin(x0[0]), x0[1]])
        goal = np.array([np.cos(goal[0]), np.sin(goal[0]), goal[1]])

        # ilqr parameters
        N = 300
        max_iter = 100
        regu_init = 100

        # cost function weights
        sCu = 10.0
        sCp = 10.0
        sCv = 10.0
        sCen = 0.0
        fCp = 100000.0
        fCv = 100000.0
        fCen = 0.0

        iLQR = iLQR_Calculator(n_x=n_x, n_u=n_u)

        # set dynamics
        # dyn_func = pendulum3_discrete_dynamics_euler
        dyn_func = pendulum3_discrete_dynamics_rungekutta
        dyn = partial(dyn_func,
                      dt=dt,
                      m=mass,
                      l=length,
                      b=damping,
                      cf=coulomb_friction,
                      g=gravity,
                      inertia=inertia)
        iLQR.set_discrete_dynamics(dyn)

        # set costs
        s_cost_func = pendulum3_swingup_stage_cost
        f_cost_func = pendulum3_swingup_final_cost
        s_cost = partial(s_cost_func,
                         goal=goal,
                         Cu=sCu,
                         Cp=sCp,
                         Cv=sCv,
                         Cen=sCen,
                         m=mass,
                         l=length,
                         b=damping,
                         cf=coulomb_friction,
                         g=gravity)
        f_cost = partial(f_cost_func,
                         goal=goal,
                         Cp=fCp,
                         Cv=fCv,
                         Cen=fCen,
                         m=mass,
                         l=length,
                         b=damping,
                         cf=coulomb_friction,
                         g=gravity)
        iLQR.set_stage_cost(s_cost)
        iLQR.set_final_cost(f_cost)

        iLQR.init_derivatives()
        iLQR.set_start(x0)

        # computation
        (X, U, cost_trace, regu_trace,
         redu_ratio_trace, redu_trace) = iLQR.run_ilqr(N=N,
                                                       init_u_trj=None,
                                                       init_x_trj=None,
                                                       max_iter=max_iter,
                                                       regu_init=regu_init,
                                                       break_cost_redu=1e-6)

        self.assertIsInstance(X, np.ndarray)
        self.assertIsInstance(U, np.ndarray)

        stabilization_success = True
        if np.abs((X[-1][0] % (2*np.pi)) - np.pi) > self.epsilon:
            if np.abs(X[-1][1]) > self.epsilon:
                stabilization_success = False
                print("ilqr Computation (n_x=3) did not swingup",
                      "final state: ", X[-1])

        self.assertTrue(stabilization_success)

    def test_0_iLQR_computation_nx2_sympy(self):
        # pendulum parameters
        mass = 0.57288
        length = 0.5
        damping = 0.15
        gravity = 9.81
        coulomb_friction = 0.0
        inertia = mass*length*length
        n_x = 2
        n_u = 1

        # swingup parameters
        x0 = np.array([0.0, 0.0])
        dt = 0.01
        goal = np.array([np.pi, 0])

        # ilqr parameters
        N = 1000
        max_iter = 100
        regu_init = 100

        # cost function weights
        sCu = 0.1
        sCp = 0.0
        sCv = 0.0
        sCen = 0.0
        fCp = 1000.0
        fCv = 1.0
        fCen = 0.0

        iLQR = iLQR_Calculator_sympy(n_x=n_x, n_u=n_u)

        # set dynamics
        # dyn_func = pendulum_discrete_dynamics_euler
        dyn_func = pendulum_discrete_dynamics_rungekutta
        dyn = partial(dyn_func,
                      dt=dt,
                      m=mass,
                      l=length,
                      b=damping,
                      cf=coulomb_friction,
                      g=gravity,
                      inertia=inertia)
        iLQR.set_discrete_dynamics(dyn)

        # set costs
        s_cost_func = pendulum_swingup_stage_cost
        f_cost_func = pendulum_swingup_final_cost
        s_cost = partial(s_cost_func,
                         goal=goal,
                         Cu=sCu,
                         Cp=sCp,
                         Cv=sCv,
                         Cen=sCen,
                         m=mass,
                         l=length,
                         b=damping,
                         cf=coulomb_friction,
                         g=gravity)
        f_cost = partial(f_cost_func,
                         goal=goal,
                         Cp=fCp,
                         Cv=fCv,
                         Cen=fCen,
                         m=mass,
                         l=length,
                         b=damping,
                         cf=coulomb_friction,
                         g=gravity)
        iLQR.set_stage_cost(s_cost)
        iLQR.set_final_cost(f_cost)

        iLQR.init_derivatives()
        iLQR.set_start(x0)

        # computation
        (X, U, cost_trace, regu_trace,
         redu_ratio_trace, redu_trace) = iLQR.run_ilqr(N=N,
                                                       init_u_trj=None,
                                                       init_x_trj=None,
                                                       max_iter=max_iter,
                                                       regu_init=regu_init,
                                                       break_cost_redu=1e-6)

        self.assertIsInstance(X, np.ndarray)
        self.assertIsInstance(U, np.ndarray)

        stabilization_success = True
        if np.abs((X[-1][0] % (2*np.pi)) - np.pi) > self.epsilon:
            if np.abs(X[-1][1]) > self.epsilon:
                stabilization_success = False
                print("ilqr Computation (n_x=2) did not swingup",
                      "final state: ", X[-1])

        self.assertTrue(stabilization_success)

    def test_0_iLQR_computation_nx3_sympy(self):
        # pendulum parameters
        mass = 0.57288
        length = 0.5
        damping = 0.15
        gravity = 9.81
        coulomb_friction = 0.0
        inertia = mass*length*length
        n_x = 3
        n_u = 1

        # swingup parameters
        dt = 0.01
        x0 = np.array([0.0, 0.0])
        goal = np.array([np.pi, 0])
        x0 = np.array([np.cos(x0[0]), np.sin(x0[0]), x0[1]])
        goal = np.array([np.cos(goal[0]), np.sin(goal[0]), goal[1]])

        # ilqr parameters
        N = 300
        max_iter = 100
        regu_init = 100

        # cost function weights
        sCu = 10.0
        sCp = 10.0
        sCv = 10.0
        sCen = 0.0
        fCp = 100000.0
        fCv = 100000.0
        fCen = 0.0

        iLQR = iLQR_Calculator_sympy(n_x=n_x, n_u=n_u)

        # set dynamics
        # dyn_func = pendulum3_discrete_dynamics_euler
        dyn_func = pendulum3_discrete_dynamics_rungekutta
        dyn = partial(dyn_func,
                      dt=dt,
                      m=mass,
                      l=length,
                      b=damping,
                      cf=coulomb_friction,
                      g=gravity,
                      inertia=inertia)
        iLQR.set_discrete_dynamics(dyn)

        # set costs
        s_cost_func = pendulum3_swingup_stage_cost
        f_cost_func = pendulum3_swingup_final_cost
        s_cost = partial(s_cost_func,
                         goal=goal,
                         Cu=sCu,
                         Cp=sCp,
                         Cv=sCv,
                         Cen=sCen,
                         m=mass,
                         l=length,
                         b=damping,
                         cf=coulomb_friction,
                         g=gravity)
        f_cost = partial(f_cost_func,
                         goal=goal,
                         Cp=fCp,
                         Cv=fCv,
                         Cen=fCen,
                         m=mass,
                         l=length,
                         b=damping,
                         cf=coulomb_friction,
                         g=gravity)
        iLQR.set_stage_cost(s_cost)
        iLQR.set_final_cost(f_cost)

        iLQR.init_derivatives()
        iLQR.set_start(x0)

        # computation
        (X, U, cost_trace, regu_trace,
         redu_ratio_trace, redu_trace) = iLQR.run_ilqr(N=N,
                                                       init_u_trj=None,
                                                       init_x_trj=None,
                                                       max_iter=max_iter,
                                                       regu_init=regu_init,
                                                       break_cost_redu=1e-6)

        self.assertIsInstance(X, np.ndarray)
        self.assertIsInstance(U, np.ndarray)

        stabilization_success = True
        if np.abs((X[-1][0] % (2*np.pi)) - np.pi) > self.epsilon:
            if np.abs(X[-1][1]) > self.epsilon:
                stabilization_success = False
                print("ilqr Computation (n_x=3) did not swingup",
                      "final state: ", X[-1])

        self.assertTrue(stabilization_success)
