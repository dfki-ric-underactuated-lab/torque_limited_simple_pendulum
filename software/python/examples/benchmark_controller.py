import os
import sys
import subprocess
import numpy as np

from simple_pendulum.utilities.process_data import load_trajectory
from simple_pendulum.analysis.benchmark import benchmarker

cons = [
    # "ilqr_open",
    "ilqr_pid",
    "ilqr_tvlqr",
    # "dircol_open",
    "dircol_pid",
    "dircol_tvlqr",
    # "ddp_open",
    "ddp_pid",
    "ddp_tvlqr",
    "energy_shaping",
    # "ilqrMPC",
    "ilqrMPCcpp",
    # "sac",
    "ddpg",
    "discrete_graph",
]

save_dir_pre = "log_data/benchmarks_new_new_new"

mass = 0.57288
length = 0.5
damping = 0.10
gravity = 9.81
coulomb_fric = 0.0
torque_limit = 2.0
inertia = mass * length * length

# simulation parameters
dt = 0.01
max_time = 10.0
integrator = "runge_kutta"
benchmark_iterations = 100

# When called without arguments, re-invoke this script once per controller in a
# fresh subprocess so that incompatible libraries (e.g. pydrake vs. PyTorch/Triton)
# never share the same process.
if len(sys.argv) == 1:
    script = os.path.abspath(__file__)
    for con in cons:
        print(f"Spawning subprocess for: {con}")
        result = subprocess.run([sys.executable, script, con])
        if result.returncode != 0:
            print(
                f"WARNING: benchmark for '{con}' exited with code {result.returncode}"
            )
    sys.exit(0)

# Single-controller mode: called as `python benchmark_controller.py <con_name>`
cons = [sys.argv[1]]

for con in cons:
    print(con)

    if con == "ilqr_open":
        from simple_pendulum.controllers.open_loop.open_loop import OpenLoopController

        csv_path = "../../../data/trajectories/ilqr/ilqr_trajectory.csv"
        data_dict = load_trajectory(csv_path)

        controller = OpenLoopController(data_dict)

        trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
        dt = trajectory[1][0] - trajectory[0][0]
        save_dir = os.path.join(save_dir_pre, "ilqr_open")

    if con == "ilqr_pid":
        from simple_pendulum.controllers.pid.pid import PIDController

        csv_path = "../../../data/trajectories/ilqr/ilqr_trajectory.csv"
        data_dict = load_trajectory(csv_path)

        controller = PIDController(data_dict=data_dict, Kp=5.0, Ki=1.0, Kd=1.0)

        trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
        dt = trajectory[1][0] - trajectory[0][0]
        save_dir = os.path.join(save_dir_pre, "ilqr_pid")

    if con == "ilqr_tvlqr":
        from simple_pendulum.controllers.tvlqr.tvlqr import TVLQRController

        csv_path = "../../../data/trajectories/ilqr/ilqr_trajectory.csv"
        data_dict = load_trajectory(csv_path)

        controller = TVLQRController(
            data_dict=data_dict,
            mass=mass,
            length=length,
            damping=damping,
            gravity=gravity,
            torque_limit=torque_limit,
        )

        trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
        dt = trajectory[1][0] - trajectory[0][0]
        save_dir = os.path.join(save_dir_pre, "ilqr_tvlqr")

    if con == "dircol_open":
        from simple_pendulum.controllers.open_loop.open_loop import OpenLoopController

        csv_path = "../../../data/trajectories/direct_collocation/dircol_trajectory.csv"
        data_dict = load_trajectory(csv_path)

        controller = OpenLoopController(data_dict)

        # integrator = "euler"
        trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
        dt = trajectory[1][0] - trajectory[0][0]
        save_dir = os.path.join(save_dir_pre, "dircol_open")

    if con == "dircol_pid":
        from simple_pendulum.controllers.pid.pid import PIDController

        csv_path = "../../../data/trajectories/direct_collocation/dircol_trajectory.csv"
        data_dict = load_trajectory(csv_path)

        controller = PIDController(data_dict=data_dict, Kp=5.0, Ki=1.0, Kd=1.0)

        # integrator = "euler"
        trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
        dt = trajectory[1][0] - trajectory[0][0]
        save_dir = os.path.join(save_dir_pre, "dircol_pid")

    if con == "dircol_tvlqr":
        from simple_pendulum.controllers.tvlqr.tvlqr import TVLQRController

        csv_path = "../../../data/trajectories/direct_collocation/dircol_trajectory.csv"
        data_dict = load_trajectory(csv_path)

        controller = TVLQRController(
            data_dict=data_dict,
            mass=mass,
            length=length,
            damping=damping,
            gravity=gravity,
            torque_limit=torque_limit,
        )

        # integrator = "euler"
        trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
        dt = trajectory[1][0] - trajectory[0][0]
        save_dir = os.path.join(save_dir_pre, "dircol_tvlqr")

    if con == "ddp_open":
        from simple_pendulum.controllers.open_loop.open_loop import OpenLoopController

        csv_path = "../../../data/trajectories/ddp/ddp_trajectory.csv"
        data_dict = load_trajectory(csv_path)

        controller = OpenLoopController(data_dict)

        trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
        dt = trajectory[1][0] - trajectory[0][0]
        save_dir = os.path.join(save_dir_pre, "ddp_open")

    if con == "ddp_pid":
        from simple_pendulum.controllers.pid.pid import PIDController

        csv_path = "../../../data/trajectories/ddp/ddp_trajectory.csv"
        data_dict = load_trajectory(csv_path)

        controller = PIDController(data_dict=data_dict, Kp=5.0, Ki=1.0, Kd=1.0)

        trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
        dt = trajectory[1][0] - trajectory[0][0]
        save_dir = os.path.join(save_dir_pre, "ddp_pid")

    if con == "ddp_tvlqr":
        from simple_pendulum.controllers.tvlqr.tvlqr import TVLQRController

        csv_path = "../../../data/trajectories/ddp/ddp_trajectory.csv"
        data_dict = load_trajectory(csv_path)

        controller = TVLQRController(
            data_dict=data_dict,
            mass=mass,
            length=length,
            damping=damping,
            gravity=gravity,
            torque_limit=torque_limit,
        )

        trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
        dt = trajectory[1][0] - trajectory[0][0]
        save_dir = os.path.join(save_dir_pre, "ddp_tvlqr")

    if con == "energy_shaping":
        from simple_pendulum.controllers.energy_shaping.energy_shaping_controller import (
            EnergyShapingAndLQRController,
        )

        controller = EnergyShapingAndLQRController(
            mass=mass,
            length=length,
            damping=damping,
            coulomb_fric=coulomb_fric,
            gravity=gravity,
            torque_limit=torque_limit,
            k=1.0,
            Q=np.diag((10, 1)),
            R=np.array([[1]]),
            compute_RoA=False,
        )
        controller.set_goal([np.pi, 0])
        save_dir = os.path.join(save_dir_pre, "energy_shaping")

    if con == "ilqrMPC":
        from simple_pendulum.controllers.ilqr.iLQR_MPC_controller import (
            iLQRMPCController,
        )

        controller = iLQRMPCController(
            mass=mass,
            length=length,
            damping=damping,
            coulomb_friction=coulomb_fric,
            gravity=gravity,
            inertia=inertia,
            dt=0.02,
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
            n_x=2,
        )
        controller.set_goal([np.pi, 0])
        save_dir = os.path.join(save_dir_pre, "ilqr_mpc")

    if con == "ilqrMPCcpp":
        from simple_pendulum.controllers.ilqr.iLQR_MPC_controller_cpp import (
            iLQRMPCcppController,
        )

        controller = iLQRMPCcppController(
            mass=mass,
            length=length,
            damping=damping,
            coulomb_friction=coulomb_fric,
            gravity=gravity,
            inertia=inertia,
            dt=dt,
            n=200,  # horizon size
            max_iter=3,
            break_cost_redu=1e-1,
            sCu=50.0,
            sCp=10.0,
            sCv=1.0,
            sCen=1.0,
            fCp=10.0,
            fCv=1.0,
            fCen=80.0,
            dynamics="runge_kutta",
        )
        controller.set_goal([np.pi, 0])
        save_dir = os.path.join(save_dir_pre, "ilqr_mpc_cpp")

    if con == "sac":
        from simple_pendulum.controllers.sac.sac_controller import SacController

        model_path = "../../../data/models/sac_model.zip"

        controller = SacController(
            model_path=model_path,
            torque_limit=1.5,  # torque_limit from training
            use_symmetry=True,
            state_representation=3,
        )
        save_dir = os.path.join(save_dir_pre, "sac")

    elif con == "ddpg":
        from simple_pendulum.controllers.ddpg.ddpg_controller import ddpg_controller

        model_path = "../../../data/models/ddpg_model/actor.keras"

        controller = ddpg_controller(
            model_path=model_path, torque_limit=1.5, state_representation=3
        )
        save_dir = os.path.join(save_dir_pre, "ddpg")

    if con == "discrete_graph":
        from simple_pendulum.controllers.discrete_graph.graph_controller import (
            GraphController,
        )

        # policy_path = "../../../data/models/dijkstra_18360.csv"
        policy_path = "../../../data/models/graph/L0.5/dijkstra_36360.csv"
        graph_dt = 0.1
        graph_pos_res = np.deg2rad(1.0)
        graph_vel_res = graph_pos_res / graph_dt
        graph_vel_max = 50 * graph_vel_res
        controller = GraphController(
            graph_csv_path=policy_path,
            pos_res=graph_pos_res,
            vel_res=graph_vel_res,
            vel_max=graph_vel_max,
        )
        save_dir = os.path.join(save_dir_pre, "discrete_graph")

    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    ben = benchmarker(
        dt=dt,
        max_time=max_time,
        integrator=integrator,
        benchmark_iterations=benchmark_iterations,
    )

    ben.init_pendulum(
        mass=mass,
        length=length,
        inertia=inertia,
        damping=damping,
        coulomb_friction=coulomb_fric,
        gravity=gravity,
        torque_limit=torque_limit,
    )

    ben.set_controller(controller)

    ben.benchmark(
        check_speed=True,
        check_energy=True,
        check_time=True,
        check_smoothness=True,
        check_consistency=True,
        check_robustness=True,
        check_sensitivity=True,
        check_torque_limit=True,
        save_path=os.path.join(save_dir, "benchmark.yml"),
    )
