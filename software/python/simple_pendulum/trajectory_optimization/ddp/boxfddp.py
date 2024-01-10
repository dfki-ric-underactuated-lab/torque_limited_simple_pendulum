"""
FDDP Calculator
===============
"""


import os
import time
import numpy as np
import crocoddyl
from pinocchio.robot_wrapper import RobotWrapper

from simple_pendulum.utilities.urdfs import set_parameters_in_urdf


class boxfddp_calculator:
    def __init__(self, urdf_path, log_dir="ddp_data"):
        self.log_dir = log_dir
        self.urdf_load_path = urdf_path
        self.urdf_work_path = os.path.join(log_dir, "temp.urdf")

        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        # self.enable_gui = enable_gui
        # if self.enable_gui:
        #     # call the gepetto viewer server
        #     _ = subprocess.Popen(["./gepetto-viewer.sh", "&"])
        #     print("Loading the viewer ...")
        #     time.sleep(1)

    def init_pendulum(
        self, mass, length, inertia, damping, coulomb_friction, torque_limit
    ):
        set_parameters_in_urdf(
            self.urdf_load_path,
            self.urdf_work_path,
            mass=mass,
            length=length,
            inertia=inertia,
            damping=damping,
            coulomb_friction=coulomb_friction,
            torque_limit=torque_limit,
        )

    def compute_trajectory(
        self,
        start_state=np.array([0.0, 0.0]),
        goal_state=np.array([np.pi, 0.0]),
        dt=4e-2,
        T=150,
        running_cost_state=1e-5,
        running_cost_torque=1e-4,
        final_cost_state=1e10,
    ):
        self.robot = RobotWrapper.BuildFromURDF(self.urdf_work_path)

        # definning the state ad actuation model
        state = crocoddyl.StateMultibody(self.robot.model)
        actuation = crocoddyl.ActuationModelFull(state)

        # defining the initial state
        self.robot.model.defaultState = start_state

        # defining the running cost model as a sun of cost
        runningCostModel = crocoddyl.CostModelSum(state)

        # defining the terminal cost model as a sun of cost
        terminalCostModel = crocoddyl.CostModelSum(state)

        xResidual = crocoddyl.ResidualModelState(state, goal_state)
        uResidual = crocoddyl.ResidualModelControl(state, actuation.nu)
        uRegCost = crocoddyl.CostModelResidual(state, uResidual)
        xRegCost = crocoddyl.CostModelResidual(state, xResidual)
        goalCost = crocoddyl.CostModelResidual(state, xResidual)

        runningCostModel.addCost("xReg", xRegCost, running_cost_state)
        runningCostModel.addCost("uReg", uRegCost, running_cost_torque)
        terminalCostModel.addCost("goal", goalCost, final_cost_state)

        runningModel = crocoddyl.IntegratedActionModelEuler(
            crocoddyl.DifferentialActionModelFreeFwdDynamics(
                state, actuation, runningCostModel
            ),
            dt,
        )
        terminalModel = crocoddyl.IntegratedActionModelEuler(
            crocoddyl.DifferentialActionModelFreeFwdDynamics(
                state, actuation, terminalCostModel
            ),
            0.0,
        )

        problem = crocoddyl.ShootingProblem(
            start_state, [runningModel] * T, terminalModel
        )
        self.fddp = crocoddyl.SolverFDDP(problem)

        # computation time
        startdtTest = time.time()
        # Solving the problem with the FDDP solver
        self.fddp.solve()
        print("computation time: %s s" % (time.time() - startdtTest))
        # print(self.fddp.xs[-1])

        # return data
        self.time_traj = np.linspace(0.0, T * dt, T)
        self.torque_traj = np.asarray(self.fddp.us).reshape(T)
        self.state_traj = np.asarray(self.fddp.xs[:T]).reshape(T, 2)

        return self.time_traj, self.state_traj, self.torque_traj
