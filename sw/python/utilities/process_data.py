import os
import pandas as pd
import numpy as np


def read(params_file, urdf_file, csv_file):
    csv_path = str(WORK_DIR) + "/data/trajectories/" + csv_file
    urdf_path = str(WORK_DIR) + "/data/urdf/" + urdf_file
    params_path = str(WORK_DIR) + "/data/parameters/" + params_file
    data = pd.read_csv(csv_path)
    n = len(data)
    return urdf_path, params_path, csv_path, data, n


def cut(data_measured, data_desired):
    nm = len(data_measured)       # compare length of desired and measured data
    n = len(data_desired)
    cut_s = 930, 
    cut_e = 1570,

    # cut data at the start of the measurement
    data_measured = data_measured.drop(data_measured.index[range(cut_s)])
    data_desired = data_desired.drop(data_desired.index[range(cut_s)])
    nm_cs = len(data_measured)
    n_cs = len(data_desired)

    # cut data at the end of the measurement
    data_measured = data_measured.drop(data_measured.index[range(nm_cs-cut_e,
                                                                 nm_cs)])

    data_desired = data_desired.drop(data_desired.index[range(n_cs-(cut_e-500),
                                                              n_cs)])
    nm_cut = len(data_measured)
    n_cut = len(data_desired)

    # reset the row index, so that the first row of the clean data set has
    # index 0 again
    data_measured = data_measured.reset_index(drop=True)
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


def save(OUTPUT_FOLDER, des_pos, des_vel, des_tau, des_time, meas_pos,
         meas_vel, meas_tau, meas_time):
    os.mkdir(OUTPUT_FOLDER)
    measured = np.array([np.array(meas_time),
                                  np.array(meas_pos),
                                  np.array(meas_vel),
                                  np.array(meas_tau)]).T
    np.savetxt(OUTPUT_FOLDER + '/data_measured.csv', measured,
               delimiter=',', header="time,position,velocity,torque",
               comments="")

    desired = np.array([np.array(des_time),
                                 np.array(des_pos),
                                 np.array(des_vel),
                                 np.array(des_tau)]).T
    np.savetxt(OUTPUT_FOLDER +'/data_desired.csv', desired,
               delimiter=',', header="time,position,velocity,torque",
               comments="")
    print(f'Saving .csv data to folder {OUTPUT_FOLDER}')

