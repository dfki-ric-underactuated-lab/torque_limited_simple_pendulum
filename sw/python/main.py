import sys
import os
from pathlib import Path
import asyncio
import pandas as pd

import argparse
from argparse import RawTextHelpFormatter

####################################################################################################
from model import parameters, pendulum_plant
from controllers import control_loop #, pd, gravity_compensation, lqr, ddp
from utilities import utils, profiler, plots
#from filters import fft, sg,
#from simulation import tlsp
####################################################################################################

can_port = 'can0'
motor_id = 0x01

####################################################################################################

CSV_FILE = "swingup_300Hz.csv"                                          # defining input files
URDF_FILE = "simple_pendulum.urdf"

WORK_DIR = Path(Path(os.path.abspath(__file__)).parents[2])             # setting the workspace
print("Workspace is set to:", WORK_DIR)
sys.path.append(f'{WORK_DIR}/sw/python')                                # adding the parent folder to your system path

OUTPUT_FOLDER, args, unknown = utils.syntax_parser(WORK_DIR)            # run the syntax parser

####################################################################################################
if __name__ == "__main__":                          
    # read data
    CSV_PATH, URDF_PATH, data, n = utils.read(WORK_DIR, CSV_FILE, URDF_FILE)

    # actuator parameters
    motor_id, gr, Kp, Kd = parameters.actuator()

    # system parameters
    joints = parameters.robot(URDF_FILE)

    # prepare data
    dt, des_pos, des_vel, des_tau, des_time, meas_pos, meas_vel, meas_tau, meas_time, des_pos_out, des_vel_out, des_tau_in, rad2outputrev = utils.prepare(data, n, gr)

    # choose the desired controller
    #if args.tau:
    #if args.lqr:
    #if args.ddp:
    
    # excecute a given trajectory on the system
    if (args.pd and args.qdd100):
        start, end, meas_dt, meas_pos, meas_vel, meas_tau, meas_time = asyncio.run(control_loop.qdd100(CSV_FILE, n, dt, des_pos_out, des_vel_out, des_tau_in, meas_pos, meas_vel, meas_tau, meas_time, gr, rad2outputrev))
    
    if (args.pd and args.ak80_6):
        start, end, meas_dt, meas_pos, meas_vel, meas_tau, meas_time = control_loop.ak80_6(CSV_FILE, data, can_port, motor_id, Kp, Kd, n, dt, des_pos, des_vel, des_tau, meas_pos, meas_vel, meas_tau, meas_time)
   
    # performance profiler
    profiler.performance(n, dt, des_time, meas_time, start, end, meas_dt)

    # save measurements
    if args.save:
        utils.save(OUTPUT_FOLDER, des_pos, des_vel, des_tau, des_time, meas_pos, meas_vel, meas_tau, meas_time)
    
    # plot data
    if args.pd:
        plots.swingup(OUTPUT_FOLDER, des_pos, des_vel, des_tau, des_time, meas_pos, meas_vel, meas_tau, meas_time)
    if args.tau:
        plots.swingup(OUTPUT_FOLDER, des_pos, des_vel, des_tau, des_time, meas_pos, meas_vel, meas_tau, meas_time)

    #if args.lqr:
    
    #if args.ddp:




