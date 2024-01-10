"""
Unit Tests
==========
"""

import os
from pathlib import Path
import unittest
import numpy as np

from simple_pendulum.trajectory_optimization.ddp.boxfddp import boxfddp_calculator


class Test(unittest.TestCase):
    epsilon = 0.2

    def test_0_DDP_trajOPtimization(self):
        # Loading the double pendulum model
        log_dir = "./"

        # pendulum parameters
        mass = 0.57288
        length = 0.5
        inertia = mass * length * length
        damping = 0.10
        gravity = 9.81
        coulomb_fric = 0.0
        torque_limit = 2.0
        WORK_DIR = Path(Path(os.path.abspath(__file__)).parents[5])
        urdf_path = os.path.join(WORK_DIR, "data/urdf/simplependul_dfki_pino_Modi.urdf")

        # swingup parameters
        x0 = np.array([0.0, 0.0])
        goal = np.array([np.pi, 0.0])
        dt = 4e-2
        N = 150

        # ddp parameters
        running_cost_state = 1e-5
        running_cost_torque = 1e-4
        final_cost_state = 1e10

        ddp = boxfddp_calculator(
            urdf_path=urdf_path,
            log_dir=log_dir,
        )
        ddp.init_pendulum(
            mass=mass,
            length=length,
            inertia=inertia,
            damping=damping,
            coulomb_friction=coulomb_fric,
            torque_limit=torque_limit,
        )

        T, X, U = ddp.compute_trajectory(
            start_state=x0,
            goal_state=goal,
            dt=dt,
            T=N,
            running_cost_state=running_cost_state,
            running_cost_torque=running_cost_torque,
            final_cost_state=final_cost_state,
        )
        xT = X[-1]

        swingup_success = True

        if np.abs((xT[0] % (2 * np.pi)) - np.pi) > self.epsilon:
            if np.abs(xT[1]) > self.epsilon:
                swingup_success = False
                print(
                    "FDDP Trajectory optimization did not swingup", "final state: ", xT
                )
        else:
            print("FDDP Trajectory optimization did swingup", "final state: ", xT)

        os.remove(os.path.join(log_dir, "temp.urdf"))

        self.assertTrue(swingup_success)
