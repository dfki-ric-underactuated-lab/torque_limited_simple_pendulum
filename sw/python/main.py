# global imports
import sys
import os
import numpy as np
from datetime import datetime
from pathlib import Path

# local imports
from model.parameters import get_params
from utilities import parse, plot, process_data, looptime
from controllers import motor_control_loop
from controllers.gravity_compensation.gravity_compensation import *
from controllers.open_loop.open_loop import *
from controllers.energy_shaping.energy_shaping_controller import *
try:
    from controllers.ilqr.iLQR_MPC_controller import *
except ModuleNotFoundError:
    pass


# run syntax parser
args, unknown = parse.syntax()

# set your workspace
WORK_DIR = Path(Path(os.path.abspath(__file__)).parents[2])
print("Workspace is set to:", WORK_DIR)
sys.path.append(f'{WORK_DIR}/sw/python')  # add parent folder to system path

# get a timestamp
TIMESTAMP = datetime.now().strftime("%Y%m%d-%I%M%S-%p")

# select control method
if args.openloop:
    if args.pd:
        name = "Proportional-Derivative Control"
        folder_name = "pd_control"
        attribute = "open_loop"

    if args.fft:
        name = "Feedforward Torque"
        folder_name = "torque_control"
        attribute = "open_loop"

    # get parameters
    params_file = "sp_parameters_openloop.yaml"
    params_path = os.path.join(Path(__file__).parents[4], 'data/parameters/' +
                               params_file)
    params = get_params(params_path)
    ## alternatively from an urdf file
    # urdf_file = dfki_simple_pendulum.urdf
    # urdf_path = os.path.join(Path(__file__).parents[4], 'data/urdf/' +
    # urdf_file )

    # load precomputed trajectory
    csv_file = "swingup_300Hz.csv"
    csv_path = os.path.join(Path(__file__).parents[4], 'data/trajectories/' +
                            csv_file)
    data_dict = process_data.prepare_trajectory(csv_path)

    control_method = OpenLoopController(data_dict)

if args.gravity:
    name = "Gravity Compensation"
    folder_name = "gravity_compensation"
    attribute = "closed_loop"

    # get parameters
    params_file = "sp_parameters_gravity.yaml"
    params_path = os.path.join(Path(__file__).parents[4], 'data/parameters/' +
                               params_file)
    params = get_params(params_path)
    data_dict = process_data.prepare_empty(params)

    control_method = GravityCompController(params)

if args.sac:
    from controllers.sac.sac_controller import *
    name = "Soft Actor Critic"
    folder_name = "sac"
    attribute = "closed_loop"

    # get parameters
    params_file = "sp_parameters_sac.yaml"
    params_path = os.path.join(Path(__file__).parents[4], 'data/parameters/' +
                               params_file)
    params = get_params(params_path)
    data_dict = process_data.prepare_empty(params)

    control_method = SacController()  # cm = control_method.return_all()

if args.energy:
    name = "Energy Shaping"
    folder_name = "energy_shaping"
    attribute = "closed_loop"

    # get parameters
    params_file = "sp_parameters_energy.yaml"
    params_path = os.path.join(Path(__file__).parents[4], 'data/parameters/' +
                               params_file)
    params = get_params(params_path)
    data_dict = process_data.prepare_empty(params)

    control_method = EnergyShapingAndLQRController(params)
    control_method.set_goal([np.pi, 0])

if args.ilqr:
    name = "Iterative Linear Quadratic Regulator"
    folder_name = "ilqr"
    attribute = "closed_loop"

    # get parameters
    params_file = "sp_parameters_ilqr.yaml"
    params_path = os.path.join(Path(__file__).parents[4], 'data/parameters/' +
                               params_file)
    params = get_params(params_path)
    data_dict = process_data.prepare_empty(params)

    mass = params['mass']
    length = params['length']
    damping = params['damping']
    gravity = params['gravity']
    coulomb_fric = params['coulomb_fric']
    n_horizon = params["n_horizon"]
    n_x = params["n_x"]
    dt = params["dt"]
    t_final = params["t_final"]
    x0 = np.array(params["x0"])
    sCu = params["sCu"]
    sCp = params["sCp"]
    sCv = params["sCv"]
    sCen = params["sCen"]
    fCp = params["fCu"]
    fCv = params["fCv"]
    fCen = params["fCen"]
    dynamics = str(params["dynamics"])
    max_iter = int(params["max_iter"])
    break_cost_redu = params["break_cost_redu"]

    goal = np.array([np.pi, 0])
    if n_x == 3:
        x0 = np.array([np.cos(x0[0]), np.sin(x0[0]), x0[1]])
        goal = np.array([np.cos(goal[0]), np.sin(goal[0]), goal[1]])

    control_method = iLQRMPCController(# parameter,
                                   mass=mass,
                                   length=length,
                                   damping=damping,
                                   coulomb_friction=coulomb_fric,
                                   gravity=gravity,
                                   x0=x0,
                                   dt=dt,
                                   n=n_horizon,  # horizon size
                                   max_iter=max_iter,
                                   break_cost_redu=break_cost_redu,
                                   sCu=sCu,
                                   sCp=sCp,
                                   sCv=sCv,
                                   sCen=sCen,
                                   fCp=fCp,
                                   fCv=fCv,
                                   fCen=fCen,
                                   dynamics=dynamics,
                                   n_x=n_x)

    control_method.set_goal(goal)
    control_method.compute_initial_guess()


""""
if args.ddp:
    name = "ddp"
    params_file = "sp_parameters_01.yaml"
"""

# start control loop for ak80_6
start, end, meas_dt, data_dict = motor_control_loop.ak80_6(control_method,
                                                           name, attribute,
                                                           params, data_dict)

# performance profiler
looptime.profiler(data_dict, start, end, meas_dt)

output_folder = str(WORK_DIR) + f'/results/{TIMESTAMP}_' + folder_name

# save measurements
if args.save:
    process_data.save(output_folder, data_dict)

# plot data
plot.swingup(args, output_folder, data_dict)
