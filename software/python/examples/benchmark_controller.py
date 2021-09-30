import os
import numpy as np

from simple_pendulum.utilities.process_data import prepare_trajectory
from simple_pendulum.analysis.benchmark import benchmarker
from simple_pendulum.controllers.energy_shaping.energy_shaping_controller import EnergyShapingController, \
                                                                                 EnergyShapingAndLQRController
from simple_pendulum.controllers.ilqr.iLQR_MPC_controller import iLQRMPCController
from simple_pendulum.controllers.sac.sac_controller import SacController
from simple_pendulum.controllers.ddpg.ddpg_controller import ddpg_controller
from simple_pendulum.controllers.open_loop.open_loop import OpenLoopController, \
                                                            OpenLoopAndLQRController
from simple_pendulum.controllers.pid.pid import PIDController
from simple_pendulum.controllers.tvlqr.tvlqr import TVLQRController


cons = ["ilqr_open", "ilqr_pid", "ilqr_tvlqr",
        "dircol_open", "dircol_pid", "dircol_tvlqr",
        "ddp_open", "ddp_pid", "ddp_tvlqr",
        "energy_shaping", "ilqrMPC", "sac", "ddpg"]

save_dir_pre = "log_data"

for con in cons:

    mass = 0.57288
    length = 0.5
    damping = 0.10
    gravity = 9.81
    coulomb_fric = 0.0
    torque_limit = 2.5
    inertia = mass*length*length

    # simulation parameters
    dt = 0.01
    max_time = 10.0
    integrator = "runge_kutta"
    benchmark_iterations = 100

    if con == "ilqr_open":
        csv_path = "../../../data/trajectories/ilqr/trajectory.csv"
        data_dict = prepare_trajectory(csv_path)

        controller = OpenLoopController(data_dict)

        trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
        dt = trajectory[1][0] - trajectory[0][0]
        save_dir = os.path.join(save_dir_pre, "ilqr_open")

    if con == "ilqr_pid":
        csv_path = "../../../data/trajectories/ilqr/trajectory.csv"
        data_dict = prepare_trajectory(csv_path)

        controller = PIDController(data_dict=data_dict, Kp=1.0, Ki=1.0, Kd=1.0)

        trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
        dt = trajectory[1][0] - trajectory[0][0]
        save_dir = os.path.join(save_dir_pre, "ilqr_pid")

    if con == "ilqr_tvlqr":
        csv_path = "../../../data/trajectories/ilqr/trajectory.csv"
        data_dict = prepare_trajectory(csv_path)

        controller = TVLQRController(data_dict=data_dict, mass=mass,
                                     length=length, damping=damping,
                                     gravity=gravity,
                                     torque_limit=torque_limit)

        trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
        dt = trajectory[1][0] - trajectory[0][0]
        save_dir = os.path.join(save_dir_pre, "ilqr_tvlqr")

    if con == "dircol_open":
        csv_path = "../../../data/trajectories/direct_collocation/trajectory.csv"
        data_dict = prepare_trajectory(csv_path)

        controller = OpenLoopController(data_dict)

        # integrator = "euler"
        trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
        dt = trajectory[1][0] - trajectory[0][0]
        save_dir = os.path.join(save_dir_pre, "dircol_open")

    if con == "dircol_pid":
        csv_path = "../../../data/trajectories/direct_collocation/trajectory.csv"
        data_dict = prepare_trajectory(csv_path)

        controller = PIDController(data_dict=data_dict, Kp=1.0, Ki=1.0, Kd=1.0)

        # integrator = "euler"
        trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
        dt = trajectory[1][0] - trajectory[0][0]
        save_dir = os.path.join(save_dir_pre, "dircol_pid")

    if con == "dircol_tvlqr":
        csv_path = "../../../data/trajectories/direct_collocation/trajectory.csv"
        data_dict = prepare_trajectory(csv_path)

        controller = TVLQRController(data_dict=data_dict, mass=mass,
                                     length=length, damping=damping,
                                     gravity=gravity,
                                     torque_limit=torque_limit)

        # integrator = "euler"
        trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
        dt = trajectory[1][0] - trajectory[0][0]
        save_dir = os.path.join(save_dir_pre, "dircol_tvlqr")

    if con == "ddp_open":
        csv_path = "../../../data/trajectories/ddp/trajectory.csv"
        data_dict = prepare_trajectory(csv_path)

        controller = OpenLoopController(data_dict)

        trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
        dt = trajectory[1][0] - trajectory[0][0]
        save_dir = os.path.join(save_dir_pre, "ddp_open")

    if con == "ddp_pid":
        csv_path = "../../../data/trajectories/ddp/trajectory.csv"
        data_dict = prepare_trajectory(csv_path)

        controller = PIDController(data_dict=data_dict, Kp=5.0, Ki=1.0, Kd=1.0)

        trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
        dt = trajectory[1][0] - trajectory[0][0]
        save_dir = os.path.join(save_dir_pre, "ddp_pid")

    if con == "ddp_tvlqr":
        csv_path = "../../../data/trajectories/ddp/trajectory.csv"
        data_dict = prepare_trajectory(csv_path)

        controller = TVLQRController(data_dict=data_dict, mass=mass,
                                     length=length, damping=damping,
                                     gravity=gravity,
                                     torque_limit=torque_limit)

        trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
        dt = trajectory[1][0] - trajectory[0][0]
        save_dir = os.path.join(save_dir_pre, "ddp_tvlqr")

    if con == "energy_shaping":
        controller = EnergyShapingAndLQRController(mass,
                                                   length,
                                                   damping,
                                                   gravity,
                                                   torque_limit)
        controller.set_goal([np.pi, 0])
        save_dir = os.path.join(save_dir_pre, "energy_shaping")

    if con == "ilqrMPC":
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
        save_dir = os.path.join(save_dir_pre, "ilqr_mpc")

    if con == "sac":

        model_path = "../../../data/models/sac_model.zip"

        controller = SacController(model_path=model_path,
                                   torque_limit=torque_limit,
                                   use_symmetry=True,
                                   state_representation=2)
        save_dir = os.path.join(save_dir_pre, "sac")

    elif con == "ddpg":

        model_path = "../../../data/models/ddpg_model/actor"
        tl = 1.5
        state_rep = 3

        controller = ddpg_controller(model_path=model_path,
                                     torque_limit=tl,
                                     state_representation=state_rep)
        save_dir = os.path.join(save_dir_pre, "ddpg")

    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    ben = benchmarker(dt=dt,
                      max_time=max_time,
                      integrator=integrator,
                      benchmark_iterations=benchmark_iterations)

    ben.init_pendulum(mass=mass,
                      length=length,
                      inertia=inertia,
                      damping=damping,
                      coulomb_friction=coulomb_fric,
                      gravity=gravity,
                      torque_limit=torque_limit)

    ben.set_controller(controller)

    ben.benchmark(check_speed=True,
                  check_energy=True,
                  check_time=True,
                  check_smoothness=True,
                  check_consistency=True,
                  check_robustness=True,
                  check_sensitivity=True,
                  check_torque_limit=True,
                  save_path=os.path.join(save_dir, "benchmark.yml"))
