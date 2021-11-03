# global imports
import sys
import os
from datetime import datetime
from pathlib import Path
import numpy as np

# local imports
from simple_pendulum.model.parameters import get_params
from simple_pendulum.utilities import parse, plot, process_data
from simple_pendulum.utilities.performance_profiler import profiler
from simple_pendulum.utilities.process_data import prepare_trajectory
from simple_pendulum.controllers import motor_control_loop
from simple_pendulum.controllers.open_loop.open_loop import OpenLoopController
from simple_pendulum.controllers.gravity_compensation.gravity_compensation import GravityCompController
from simple_pendulum.controllers.energy_shaping.energy_shaping_controller import EnergyShapingAndLQRController
from simple_pendulum.controllers.ilqr.iLQR_MPC_controller import iLQRMPCController

try:
    from simple_pendulum.controllers.tvlqr.tvlqr import TVLQRController
except ModuleNotFoundError:
    # drake not installed
    pass

try:
    from simple_pendulum.controllers.sac.sac_controller import SacController
except ModuleNotFoundError:
    # pytorch not installed
    pass

try:
    from simple_pendulum.controllers.ddpg.ddpg_controller import ddpg_controller
except ModuleNotFoundError:
    # tensorflow not installed
    pass

"""
All parameters of the controllers in this script for the real simple pendulum
are stored in a .yaml file. The yaml files for the different controllers
can be found in the /data/parameters directory.
Some parameters like (mass, length) can be measured directly, others 
are obtained from system identification (damping, coulomb friction, 
inertia) or depend on actuator properties (torque limits, gear ratio, 
kp, kd).
"""

# set motor parameters
motor_id = "0x02"
can_port = 'can0'

# run syntax parser
args, unknown = parse.syntax()

# set your workspace
WORK_DIR = Path(Path(os.path.abspath(__file__)).parents[3])
print("Workspace is set to:", WORK_DIR)

# get a timestamp
TIMESTAMP = datetime.now().strftime("%Y%m%d-%I%M%S-%p")

# select control method

if args.ddp or args.dircol or args.ilqr:
    """
        Open loop control methods replay a precomputed trajectory. The 
        trajectories are derived offline from one out of two trajectory 
        optimization techniques:
            - Feasibility-Driven Dynamic Programming (within the open source 
              software crocoddyl)
            - Direct Collocation (within the open source software pyDrake)
            - iterative linear quadratic regulator
        
        A trajectory is split into time steps and stored as csv file in the 
        form of position, velocity and torque data for every time step. It is 
        important to ensure that the time step size acquired from trajectory 
        optimization matches with the frequency in which the time steps are 
        executed on the real system. We achieve this with a while loop 
        
        "while time.time() - start_loop < dt:
                pass

        that runs until the control loop run time matches with the desired time
        step size of the precomputed trajectory and a error print out that tells 
        us, if the control loop is slower then the desired time step size.
    """
    if args.ddp:
        """
        This option uses a trajectory obtained via Feasibility-Driven 
        Differential Dynamic Programming with crocoddyl. 
        """
        csv_file = "trajectory.csv"
        csv_path = os.path.join(WORK_DIR, 'data', 'trajectories', 'ddp', csv_file)
        name = "Differential Dynamic Programming"
        folder_name = "ddp"

    if args.dircol:
        """
        This option uses a trajectory obtained via Direct Collocation
        with pydrake. 
        """
        csv_file = "trajectory.csv"
        csv_path = os.path.join(WORK_DIR, 'data', 'trajectories', 'direct_collocation', csv_file)
        name = "Direct Collocation"
        folder_name = "dircol"

    if args.ilqr:
        """
        This option uses a trajectory obtained via iterative linear quadratic regulator.
        """
        csv_file = "trajectory.csv"
        csv_path = os.path.join(WORK_DIR, 'data', 'trajectories', 'ilqr', csv_file)
        name = "Iterative Linear Quadratic Regulator"
        folder_name = "ilqr"

    csv_data = prepare_trajectory(csv_path)

    params_file = "sp_parameters_trajectory.yaml"
    params_path = os.path.join(WORK_DIR, 'data', 'parameters', params_file)
    params = get_params(params_path)
    data_dict = process_data.prepare_trajectory(csv_path)

    #if args.pd: # is realized on motor level (see below)
    #    control_method = PIDController(data_dict=csv_data, Kp=3.0, Ki=1.0, Kd=1.0)
    if args.tvlqr:
        control_method = TVLQRController(data_dict=csv_data,
                                         mass=params["mass"],
                                         length=params["length"],
                                         damping=params["damping"],
                                         gravity=params["gravity"],
                                         torque_limit=params["torque_limit"])
        control_method.init([csv_data["des_pos_list"][0],
                             csv_data["des_pos_list"][0]])
        control_method.set_goal([np.pi, 0.0])  # final point must be stable point
        name += " + tvlqr"
        folder_name += "_tvlqr"
    else:  # args.fft or args.pd
        control_method = OpenLoopController(data_dict=csv_data)

    if args.pd:
        """
        Trajectory following controllers act on a precomputed trajectory and 
        ensure that the system follows the trajectory properly. The 
        proportional-derivative controller is composed of a proportional term,
        gaining torque proportional to the position error and a derivative term 
        gaining torque proportional to the derivative of the position error. 
        The controller can be seen as a spring-damper system where the 
        proportional gain contributes to the stiffness/springiness of the system 
        and the derivative term acts as a damper.
        """
        name += " + pd control"
        folder_name += "_pd"
        attribute = "pd_control"
    else:  # args.fft:
        """
        The feed-forward torque controller is simply forwarding the torque 
        control signal from a precomputed trajectory.
        """
        attribute = "motorfft"
        name += " + feedforward torque"
        folder_name += "_fft"

if args.gravity:
    """
        A controller compensating the gravitational force acting on the pendulum. 
        The pendulum can be moved as if it was in zero-g. The control input 
        torque is computed online and depends directly on the current position 
        of the pendulum.
    """
    name = "Gravity Compensation"
    folder_name = "gravity_compensation"
    attribute = "motorfft"

    # get parameters
    params_file = "sp_parameters_gravity.yaml"
    params_path = os.path.join(WORK_DIR, 'data', 'parameters', params_file)
    params = get_params(params_path)
    data_dict = process_data.prepare_empty(params)

    control_method = GravityCompController(params)

if args.sac:
    """
        Soft Actor Critic is an off-policy model free reinforcement learning 
        algorithm. It maximizes a trade-off between expected return of a reward 
        function and entropy, a measure of randomness in the policy. The 
        controller is trained via interaction with the system, such that a 
        mapping from state space to control command is learned. It generates 
        input torques online based on the learned control policy.
    """
    name = "Soft Actor Critic"
    folder_name = "sac"
    attribute = "motorfft"

    # get parameters
    params_file = "sp_parameters_sac.yaml"
    params_path = os.path.join(WORK_DIR, 'data', 'parameters', params_file)
    params = get_params(params_path)
    data_dict = process_data.prepare_empty(params)

    control_method = SacController(model_path=os.path.join(WORK_DIR, params['model_path']),
                                   torque_limit=params['torque_limit'],
                                   use_symmetry=params['use_symmetry'])

if args.ddpg:
    """
        The controller is trained via interaction with the system, such that a 
        mapping from state space to control command is learned. It generates 
        input torques online based on the learned control policy.
    """
    name = "Deep deterministic policy gradient"
    folder_name = "ddpg"
    attribute = "motorfft"

    # get parameters
    params_file = "sp_parameters_ddpg.yaml"
    params_path = os.path.join(WORK_DIR, 'data', 'parameters', params_file)
    params = get_params(params_path)
    data_dict = process_data.prepare_empty(params)

    control_method = ddpg_controller(model_path=os.path.join(WORK_DIR, params['model_path']),
                                   torque_limit=params['torque_limit'],
                                   state_representation=params['state_representation'])
if args.energy:
    """
        A controller regulating the energy of the pendulum. Drives the pendulum 
        into the upright position where the system has maximum potential and no 
        kinetic energy by controlling the desired energy level. The control 
        input torque is computed online and depends on current position and 
        velocity of the pendulum. Note however, that the controller does not 
        stabilize the upright position. For this task an additional LQR 
        controller is needed.
    """
    name = "Energy Shaping"
    folder_name = "energy_shaping"
    attribute = "motorfft"

    # get parameters
    params_file = "sp_parameters_energy.yaml"
    params_path = os.path.join(WORK_DIR, 'data', 'parameters', params_file)
    params = get_params(params_path)
    data_dict = process_data.prepare_empty(params)

    control_method = EnergyShapingAndLQRController(
                                        mass=params['mass'],
                                        length=params['length'],
                                        damping=params['damping'],
                                        gravity=params['gravity'],
                                        torque_limit=params['torque_limit'],
                                        k=params['k'])
    control_method.set_goal([np.pi, 0])

if args.ilqrmpc:
    """
        A controller which performs an iLQR optimization at every time step and 
        executes the first control signal of the computed optimal trajectory. 
        The iLQR optimization method has the ability to take the full system 
        dynamics into account and plan ahead by optimizing over a sequence of 
        control inputs. This means iLQR is used in an Model Predictive Control 
        (MPC) setting. New trajectories are generated online and therefore the 
        controller is able to equalize perturbations.
    """
    name = "Iterative Linear Quadratic Regulator"
    folder_name = "ilqrmpc"
    attribute = "motorfft"

    # get parameters
    params_file = "sp_parameters_ilqr.yaml"
    params_path = os.path.join(WORK_DIR, 'data', 'parameters', params_file)
    params = get_params(params_path)
    data_dict = process_data.prepare_empty(params)

    control_method = iLQRMPCController(
                                mass=params['mass'],
                                length=params['length'],
                                damping=params['damping'],
                                coulomb_friction=params['coulomb_fric'],
                                gravity=params['gravity'],
                                dt=params['dt'],
                                n=params['n_horizon'],
                                max_iter=int(params['max_iter']),
                                break_cost_redu=params['break_cost_redu'],
                                sCu=params['sCu'],
                                sCp=params['sCp'],
                                sCv=params['sCv'],
                                sCen=params['sCen'],
                                fCp=params['fCp'],
                                fCv=params['fCv'],
                                fCen=params['fCen'],
                                dynamics=str(params['dynamics']),
                                n_x=params['n_x'])

    control_method.set_goal(np.array([np.pi, 0]))
    control_method.init(x0=np.array(params["x0"]))

# start control loop for ak80_6
start, end, meas_dt, data_dict = motor_control_loop.ak80_6(control_method,
                                                           name, attribute,
                                                           params, data_dict,
                                                           motor_id, can_port)

# performance profiler
profiler(data_dict, start, end, meas_dt)

# save measurements
output_folder = str(WORK_DIR) + f'/results/{TIMESTAMP}_' + folder_name
if args.save:
    process_data.save(output_folder, data_dict)

# plot data
plot.swingup(args, output_folder, data_dict)
