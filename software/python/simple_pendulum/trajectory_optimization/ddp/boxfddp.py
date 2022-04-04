"""
FDDP Calculator
===============
"""


import os
import time
import subprocess
import re
import xml.etree.ElementTree as ET
from lxml import etree
import eigenpy
import numpy as np

# crocoddyl imports
import crocoddyl
from crocoddyl.utils.pendulum import ActuationModelDoublePendulum

# pinocchio imports
from pinocchio.robot_wrapper import RobotWrapper

# Matrix deprecated -> this should ensure that everything
# is treated/converted in array
eigenpy.switchToNumpyArray()


def remove_namespaces(tree):
    for el in tree.getiterator():
        match = re.match("^(?:\{.*?\})?(.*)$", el.tag)
        if match:
            el.tag = match.group(1)


def set_parameters_in_urdf(urdf_in,
                           urdf_out,
                           mass,
                           inertia,
                           length,
                           damping,
                           coulomb_friction,
                           torque_limit):

    # ET.register_namespace("xsi", "http://www.w3.org/2001/XMLSchema-instance")
    tree = ET.parse(urdf_in, parser=etree.XMLParser())
    remove_namespaces(tree)
    root = tree.getroot()

    for joint in root.iter('joint'):
        if joint.attrib['name'] == "shoulder":
            for a in joint.iter('dynamics'):
                a.attrib['damping'] = str(damping)
                a.attrib['friction'] = str(coulomb_friction)
            for a in joint.iter('limit'):
                a.attrib['effort'] = str(torque_limit)
            for a in joint.iter('origin'):
                a.attrib['xyz'] = "0 "+str(length)+" 0"

    for link in root.iter('link'):
        if link.attrib['name'] == "upper_link":
            for a in link.iter('inertial'):
                for aa in a.iter('mass'):
                    aa.attrib['value'] = str(mass)
                for aa in a.iter('origin'):
                    aa.attrib['xyz'] = "0 0 "+str(-length)
                for aa in a.iter('inertia'):
                    aa.attrib['iyy'] = str(inertia)
            for a in link.iter('visual'):
                for aa in a.iter('geometry'):
                    for aaa in aa.iter('cylinder'):
                        aaa.attrib['length'] = str(length)

    tree.write(urdf_out)


class boxfddp_calculator():
    def __init__(self,
                 urdf_path,
                 enable_gui=False,
                 log_dir="ddp_data"):

        self.log_dir = log_dir
        self.urdf_load_path = urdf_path
        self.urdf_work_path = os.path.join(log_dir, "temp.urdf")

        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        self.enable_gui = enable_gui
        if self.enable_gui:
            # call the gepetto viewer server
            _ = subprocess.Popen(["./gepetto-viewer.sh", "&"])
            print('Loading the viewer ...')
            time.sleep(1)

    def init_pendulum(self,
                      mass,
                      length,
                      inertia,
                      damping,
                      coulomb_friction,
                      torque_limit):

        set_parameters_in_urdf(self.urdf_load_path,
                               self.urdf_work_path,
                               mass=mass,
                               length=length,
                               inertia=inertia,
                               damping=damping,
                               coulomb_friction=coulomb_friction,
                               torque_limit=torque_limit)

    def compute_trajectory(self,
                           start_state=np.array([0., 0.]),
                           goal_state=np.array([np.pi, 0.]),
                           weights=np.array([1] + [0.1] * 1),
                           dt=4e-2,
                           T=150,
                           running_cost_state=1e-5,
                           running_cost_torque=1e-4,
                           final_cost_state=1e10):

        self.robot = RobotWrapper.BuildFromURDF(self.urdf_work_path)

        # creating the robot model and data in Crocoddyl
        robot_model = self.robot.model
        rmodel = self.robot.model
        # rdata = robot_model.createData()

        # adding the toques and velocities limites
        # lims = rmodel.effortLimit

        # definning the state ad actuation model
        state = crocoddyl.StateMultibody(robot_model)
        actModel = ActuationModelDoublePendulum(state, actLink=1)
        # the actuated link is the 1

        # defining the initial state
        rmodel.defaultState = start_state

        # defining the running cost model as a sun of cost
        runningCostModel = crocoddyl.CostModelSum(state, actModel.nu)

        # defining the terminal cost model as a sun of cost
        terminalCostModel = crocoddyl.CostModelSum(state, actModel.nu)

        # defining the costs for the running and the terminal model
        xPendCost = crocoddyl.CostModelState(
                             state,
                             crocoddyl.ActivationModelWeightedQuad(weights),
                             goal_state,
                             actModel.nu)

        uRegCost = crocoddyl.CostModelControl(state,
                                              crocoddyl.ActivationModelQuad(1),
                                              actModel.nu)

        # Extract Time
        time_traj = np.linspace(0.0, T*dt, T)
        time_traj = time_traj.reshape(T, 1).T
        print("Time Array Shape: {}".format(time_traj.shape))

        # adding costs for the running model
        runningCostModel.addCost("xGoal", xPendCost, running_cost_state)
        runningCostModel.addCost("uReg", uRegCost, running_cost_torque)

        # adding costs for the terminal model
        terminalCostModel.addCost("xGoalTerminal", xPendCost, final_cost_state)

        # models
        runningModel = crocoddyl.IntegratedActionModelEuler(
            crocoddyl.DifferentialActionModelFreeFwdDynamics(state,
                                                             actModel,
                                                             runningCostModel), dt)

        terminalModel = crocoddyl.IntegratedActionModelEuler(
            crocoddyl.DifferentialActionModelFreeFwdDynamics(state,
                                                             actModel,
                                                             terminalCostModel), 0)

        problem = crocoddyl.ShootingProblem(start_state,
                                            [runningModel] * T,
                                            terminalModel)
        self.fddp = crocoddyl.SolverFDDP(problem)

        if self.enable_gui:
            # display defining the position and orientation of the camera
            #  by a quaternion
            cameraTF = [1.4, 0., 0.2, 0.5, 0.5, 0.5, 0.5]
            self.display = crocoddyl.GepettoDisplay(self.robot,
                                                    4,
                                                    4,
                                                    cameraTF,
                                                    True)

            self.fddp.setCallbacks([crocoddyl.CallbackLogger(),
                                    crocoddyl.CallbackVerbose(),
                                    crocoddyl.CallbackDisplay(self.display)])

        else:
            self.fddp.setCallbacks([crocoddyl.CallbackLogger(),
                                    crocoddyl.CallbackVerbose()])

        # to test computational time
        startdtTest = time.time()
        # Solving the problem with the FDDP solver
        self.fddp.solve()
        print("time taken--- %s seconds ---" % (time.time() - startdtTest))
        # xT = self.fddp.xs[-1]

        self.log = self.fddp.getCallbacks()[0]

        # reshape the output trajectory
        torque = []
        Q = []
        V = []

        for i in range(T):
            torque.extend([self.fddp.us[i]])
            Q.extend([self.fddp.xs[i][:1]])
            V.extend([self.fddp.xs[i][1:]])

        TOR = np.asarray(torque)
        self.torque_traj = TOR[:].reshape(T, 1).T

        ROT = np.asarray(Q)
        self.theta = ROT[:].reshape(T, 1).T

        VEL = np.asarray(V)
        self.theta_dot = VEL[:].reshape(T, 1).T

        return time_traj, self.theta, self.theta_dot, self.torque_traj

    def plot_trajectory(self):
        # Plotting the entire motion

        crocoddyl.plotOCSolution(self.log.xs,
                                 self.log.us,
                                 figIndex=1,
                                 show=True)
        crocoddyl.plotConvergence(self.log.costs,
                                  self.log.u_regs,
                                  self.log.x_regs,
                                  self.log.grads,
                                  self.log.stops,
                                  self.log.steps,
                                  figIndex=2)

    def simulate_trajectory_gepetto(self):
        # Display the entire motion

        if self.enable_gui:
            self.display = crocoddyl.GepettoDisplay(self.robot, floor=False)
            self.display.displayFromSolver(self.fddp)
        else:
            print("Gui was disabled in class initialization." +
                  "Simulation not possible.")
