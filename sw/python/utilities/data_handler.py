import os
import pandas as pd
import numpy as np
from datetime import datetime

import argparse
from argparse import RawTextHelpFormatter

def syntax_parser(WORK_DIR):
    parser = argparse.ArgumentParser(
        description="Simulation of a torque limited simple pendulum powered by a quasi-direct drive", formatter_class=RawTextHelpFormatter)
    parser.add_argument("--pd", action='store_true',
                        help="Use position and velocity control mode")
    parser.add_argument("--tau", action='store_true',
                        help="Use torque control mode")
    parser.add_argument("--lqr", action='store_true',
                        help="Use a Linear Quadratic Regulator for control")
    parser.add_argument("--ddp", action='store_true',
                        help="Use Differential Dynamic Programming for control")
    parser.add_argument("--simulate", action='store_true',
                        help="Runs a physical simulation of the pendulum plant, instead of controlling the real system.")
    parser.add_argument("--save", action='store_true',
                        help="Save results to the results folder (../results)")

    # Execute the parse_args method
    args, unknown = parser.parse_known_args()    

    folder_name = ""
    if args.position_control:
        folder_name = "position_control"
    if args.velocity_control:
        folder_name = "velocity_control" 
    if args.torque_control:
        folder_name = "torque_control"
    if args.lqr:
        folder_name = "lqr"
    if args.ddp:
        folder_name = "ddp"

    TIMESTAMP = datetime.now().strftime(                # get timestamp              
        "%Y%m%d-%I%M%S-%p")
    OUTPUT_FOLDER = WORK_DIR + f'/results/{TIMESTAMP}_' + folder_name
    return OUTPUT_FOLDER, args, unknown

def read(WORK_DIR, CSV_FILE, URDF_FILE):
    CSV_PATH = WORK_DIR + "/data/trajectories/" + CSV_FILE
    URDF_PATH = WORK_DIR + "/data/" + URDF_FILE
    data = pd.read_csv(CSV_PATH)
    n = len(data)
    return CSV_PATH, URDF_PATH, data, n

def prepare(data, n, gr):
    # store trajectories for position, velocity and torque in a list
    des_pos = data["pos"]                                                               # desired position in radian
    des_vel = data["vel"]                                                               # desired velocity in radian/s
    des_tau = data["torque"]                                                            # desired torque in Nm
    des_time = data["time"]                                                             # desired torque in s
    meas_pos,  meas_vel, meas_tau, meas_time  = np.zeros(n)                             # create empty numpy array, where measured data can be stored
    
    dt = (data["time"][n-1] - data["time"][0])/n                                        # avg desired dt

    # convert commanded position (in rad) to revolutions at the outputshaft (after the gear transmission)
    rad2outputrev = (gr)/(2*np.pi)

    # converting the desired trajectories according to the gear ratio
    des_pos_out = [x * rad2outputrev for x in des_pos]                                  # desired position in revolutions at the output shaft
    des_vel_out = [x * rad2outputrev for x in des_vel]                                  # desired velocity in revolutions/s at the output shaft
    des_tau_in = [x * (1/gr) for x in des_tau]                                          # torque in Nm on the motor side before the gear transmission
    return dt, des_pos, des_vel, des_tau, des_time, meas_pos, meas_vel, meas_tau, meas_time, des_pos_out, des_vel_out, des_tau_in, rad2outputrev

def cut(data_measured, data_desired):
    nm = len(data_measured)                                                             # compare length of desired and measured data
    n = len(data_desired)
    cut_s = 930, 
    cut_e = 1570,

    data_measured = data_measured.drop(data_measured.index[range(cut_s)])               # cut data at the start of the measurement       
    data_desired = data_desired.drop(data_desired.index[range(cut_s)])
    nm_cs = len(data_measured)
    n_cs = len(data_desired)

    data_measured = data_measured.drop(data_measured.index[range(nm_cs-cut_e, nm_cs)])  # cut data at the end of the measurement
    data_desired = data_desired.drop(data_desired.index[range(n_cs-(cut_e-500), n_cs)])
    nm_cut = len(data_measured)
    n_cut = len(data_desired)
    data_measured = data_measured.reset_index(drop=True)                                # reset the row index, so that the first row of the clean data set has index 0 again
    data_desired = data_desired.reset_index(drop=True)
    print("'cut_data' function output")
    print("number of all data points:")
    print("desired =", n)
    print("measured =", nm)
    print("number of data points with clean start:")
    print("desired =", n_cs)
    print("measured =", nm_cs)
    print("number of data points with clean start + end:")
    print("desired =", n_cut)
    print("measured =", nm_cut)
    print()
    return data_measured, data_desired, n_cut

def save(OUTPUT_FOLDER, des_pos, des_vel, des_tau, des_time, meas_pos, meas_vel, meas_tau, meas_time):
    os.mkdir(OUTPUT_FOLDER)
    measured = np.array([np.array(meas_time),
                                  np.array(meas_pos),
                                  np.array(meas_vel),
                                  np.array(meas_tau)]).T
    np.savetxt(OUTPUT_FOLDER + '/data_measured.csv', measured,
               delimiter=',', header="time,position,velocity,torque", comments="")

    desired = np.array([np.array(des_time),
                                 np.array(des_pos),
                                 np.array(des_vel),
                                 np.array(des_tau)]).T
    np.savetxt(OUTPUT_FOLDER +'/data_desired.csv', desired,
               delimiter=',', header="time,position,velocity,torque", comments="")
    print(f'Saving .csv data to folder {OUTPUT_FOLDER}')

