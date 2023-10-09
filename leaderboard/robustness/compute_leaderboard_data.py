# Other imports
import os
import importlib
import argparse

# Local imports
from simple_pendulum.analysis.benchmark import benchmarker

from sim_parameters import (
    mass,
    length,
    damping,
    gravity,
    coulomb_fric,
    torque_limit,
    inertia,
    dt,
    t_final,
    integrator,
    benchmark_iterations,
)


def compute_leaderboard_data(data_dir, con_filename):
    controller_arg = con_filename[:-3]
    controller_name = controller_arg[4:]

    save_dir = f"{data_dir}/{controller_name}"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    imp = importlib.import_module(controller_arg)

    controller = imp.controller

    ben = benchmarker(
        dt=dt,
        max_time=t_final,
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
        check_speed=False,
        check_energy=False,
        check_time=False,
        check_smoothness=False,
        check_consistency=True,
        check_robustness=True,
        check_sensitivity=True,
        check_torque_limit=True,
        save_path=os.path.join(save_dir, "benchmark.yml"),
    )

    if os.path.exists(f"readmes/{controller_name}.md"):
        os.system(f"cp readmes/{controller_name}.md {save_dir}/README.md")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--controller",
        dest="con_filename",
        help="Controller file containing the initialization of the controller.",
        default="con_energyshaping_lqr.py",
        required=True,
    )

    con_filename = parser.parse_args().con_filename

    data_dir = "data"
    if not os.path.exists(data_dir):
        os.makedirs(data_dir)

    print(f"Simulating new controller {con_filename}")
    compute_leaderboard_data(data_dir, con_filename)
