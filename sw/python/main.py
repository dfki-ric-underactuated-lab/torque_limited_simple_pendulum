# Global imports
import sys
import os
import asyncio
from pathlib import Path
#import argparse
#from argparse import RawTextHelpFormatter
#from  import CanMotorController
#import pandas as pd

# Local imports
from model import pendulum_plant, params
#from simulation import * 
#from trajectory_optimization import * 
from controllers import motor_control_loop
#from controllers.open_loop import *
#from controllers.energy_shaping import *
#from controllers.ilqr import *
#from controllers.lqr import *
#from controllers.sac import *
from utilities import data_process, looptime, parser, plot
#from filters import *

# workspace
WORK_DIR = Path(Path(os.path.abspath(__file__)).parents[2])  
# add parent folder system path
sys.path.append(f'{WORK_DIR}/sw/python')

# run syntax parser
OUTPUT_FOLDER, args, unknown = parser.SyntaxParser(WORK_DIR)

# select controller

# select actuator
params_actuator = params.ak80_6_01

g=params.earth.gravity

# defining input files
CSV_FILE = "swingup_300Hz.csv"                                          
URDF_FILE = "simple_pendulum.urdf"

###############################################################################
if __name__ == "__main__":                      
    # read data
    CSV_PATH, URDF_PATH, data, n = data_process.read(WORK_DIR, CSV_FILE, 
        URDF_FILE)

    # actuator parameters
    gr = 6
    Kp = 50
    Kd = 4
    #motor_id, gr, Kp, Kd = parameters.actuator()

    # system parameters
    #joints = parameters.robot(URDF_FILE)

    # prepare data
    (dt, des_pos, des_vel, des_tau, des_time, meas_pos, meas_vel,
     meas_tau, meas_time, des_pos_out, des_vel_out, des_tau_in, 
     rad2outputrev) = data_process.prepare(data, n, gr) 

    # choose the desired controller
    #if args.tau:
    #if args.lqr:
    #if args.ddp:
    
    # excecute a given trajectory on the system
    if (args.pd and args.qdd100):
        (start, end, meas_dt, meas_pos, meas_vel, meas_tau, 
         meas_time) = asyncio.run(motor_control_loop.qdd100(CSV_FILE, n, dt, 
         des_pos_out, des_vel_out, des_tau_in, meas_pos, meas_vel, meas_tau, 
         meas_time, gr, rad2outputrev))
    
    if (args.pd and args.ak80_6):
        (start, end, meas_dt, meas_pos, meas_vel, meas_tau,
         meas_time) = motor_control_loop.ak80_6(CSV_FILE, Kp, Kd, n, dt, 
         des_pos, des_vel, des_tau, meas_pos, meas_vel, meas_tau, meas_time)
   
    # performance profiler
    looptime.profiler(n, dt, des_time, meas_time, start, end, meas_dt)

    # save measurements
    if args.save:
        data_process.save(OUTPUT_FOLDER, des_pos, des_vel, des_tau, des_time, 
                          meas_pos, meas_vel, meas_tau, meas_time)
    
    # plot data
    if args.pd:
        plot.swingup(OUTPUT_FOLDER, des_pos, des_vel, des_tau, des_time, 
                     meas_pos, meas_vel, meas_tau, meas_time)
    if args.tau:
        plot.swingup(OUTPUT_FOLDER, des_pos, des_vel, des_tau, des_time,
                     meas_pos, meas_vel, meas_tau, meas_time)

    #if args.lqr:
    
    #if args.ddp:




