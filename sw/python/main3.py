# Global imports
import sys
import os
import yaml
import numpy as np
# import asyncio
from pathlib import Path

# Local imports
from controllers import motor_control_loop2
from controllers.iLQR.iLQR_MPC_controller import iLQRMPCController
from utilities import data_process, looptime, parser, plot

# Set your workspace
WORK_DIR = Path(Path(os.path.abspath(__file__)).parents[2])
sys.path.append(f'{WORK_DIR}/sw/python')        # add parent folder system path

# run syntax parser
OUTPUT_FOLDER, args, unknown = parser.SyntaxParser(WORK_DIR)

# select control method


# select actuator
#params_actuator = params.ak80_6_01

#g = params.earth.gravity

# defining input files
#CSV_FILE = "swingup_300Hz.csv"
#URDF_FILE = "simple_pendulum.urdf"


filepath = "../../data/models/sp_parameters2.yaml"
with open(filepath, 'r') as yaml_file:
    params = yaml.safe_load(yaml_file)
mass = params["mass"]
length = params["length"]
damping = params["damping"]
gravity = params["gravity"]
coulomb_fric = params["coulomb_fric"]
inertia = None
torque_limit = 0.5      # params["torque_limit"]

n_x = 3
dt = 0.02
t_final = 10.0
x0 = np.array([0.0, 0.0])
x0_sim = x0.copy()
goal = np.array([np.pi, 0])
if n_x == 3:
    x0 = np.array([np.cos(x0[0]), np.sin(x0[0]), x0[1]])
    goal = np.array([np.cos(goal[0]), np.sin(goal[0]), goal[1]])



#controller = EnergyShapingController(mass,
#                                     length,
#                                     damping,
#                                     gravity,
#                                     0.3)
#controller.set_goal([np.pi, 0])



controller = iLQRMPCController(mass=mass,
                               length=length,
                               damping=damping,
                               coulomb_friction=coulomb_fric,
                               gravity=gravity,
                               x0=x0,
                               dt=dt,
                               N=50,  # horizon size
                               max_iter=1,
                               break_cost_redu=1e-1,
                               sCu=1.5,
                               sCp=50.0,
                               sCv=1.0,
                               sCen=0.0,
                               fCp=20.0,
                               fCv=1.0,
                               fCen=0.0,
                               dynamics="runge_kutta",
                               n_x=n_x)


###############################################################################
if __name__ == "__main__":                      
    # read data
    #CSV_PATH, URDF_PATH, data, n = data_process.read(WORK_DIR, CSV_FILE,
    #    URDF_FILE)

    # actuator parameters
    #gr = 6
    kp = 0
    kd = 0
    #dt = 0.01
    n = 1000


    # motor_id, gr, Kp, Kd = parameters.actuator()

    # system parameters
    # joints = parameters.robot(URDF_FILE)

    # prepare data
    #(dt, des_pos, des_vel, des_tau, des_time, meas_pos, meas_vel,
    # meas_tau, meas_time, des_pos_out, des_vel_out, des_tau_in,
     #rad2outputrev) = data_process.prepare(data, n, gr)

    # choose the desired controller
    # if args.tau:
    # if args.lqr:
    # if args.ddp:
    
    # execute a given trajectory on the system
    #if args.pd and args.qdd100:
    #    (start, end, meas_dt, meas_pos, meas_vel, meas_tau, meas_time) = \
    #        asyncio.run(motor_control_loop.qdd100(CSV_FILE, n, dt,
    #                                              des_pos_out, des_vel_out,
    #                                              des_tau_in, meas_pos,
    #                                             meas_vel, meas_tau,
    #                                              meas_time, gr,
    #                                              rad2outputrev))
    
    if args.energy and args.ak80_6:
        (start, end, meas_dt, meas_pos, meas_vel, meas_tau, meas_time,
         des_pos, des_vel, des_tau, des_time) = motor_control_loop2.ak80_6(
            controller, kp, kd, n, dt)

    if args.ilqr and args.ak80_6:
        (start, end, meas_dt, meas_pos, meas_vel, meas_tau, meas_time,
         des_pos, des_vel, des_tau, des_time) = motor_control_loop2.ak80_6(
            controller, kp, kd, n, dt)
   
    # performance profiler
    # looptime.profiler(n, dt, des_time, meas_time, start, end, meas_dt)

    # save measurements
    if args.save:
        data_process.save(OUTPUT_FOLDER, des_pos, des_vel, des_tau, des_time, 
                          meas_pos, meas_vel, meas_tau, meas_time)
    
    # plot data
    if args.energy:
        plot.swingup(OUTPUT_FOLDER, des_pos, des_vel, des_tau, des_time,
                     meas_pos, meas_vel, meas_tau, meas_time)
    #if args.tau:
    #    plot.swingup(OUTPUT_FOLDER, des_pos, des_vel, des_tau, des_time,
    #                 meas_pos, meas_vel, meas_tau, meas_time)

    # if args.lqr:
    
    # if args.ddp:




