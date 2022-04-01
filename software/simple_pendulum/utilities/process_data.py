"""
Data Processing
===============
"""


import os
import pandas as pd
import numpy as np


def read(WORK_DIR, params_file, urdf_file, csv_file):
    csv_path = str(WORK_DIR) + "/data/trajectories/" + csv_file
    urdf_path = str(WORK_DIR) + "/data/urdf/" + urdf_file
    params_path = str(WORK_DIR) + "/data/parameters/" + params_file
    data = pd.read_csv(csv_path)
    n = len(data)
    return urdf_path, params_path, csv_path, data, n


def prepare_empty(params):
    dt = params['dt']
    t = params['runtime']
    n = int(t/dt)

    # create 4 empty numpy array, where measured data can be stored
    des_time_list = np.zeros(n)
    des_pos_list = np.zeros(n)
    des_vel_list = np.zeros(n)
    des_tau_list = np.zeros(n)

    meas_time_list = np.zeros(n)
    meas_pos_list = np.zeros(n)
    meas_vel_list = np.zeros(n)
    meas_tau_list = np.zeros(n)
    vel_filt_list = np.zeros(n)

    data_dict = {"des_time_list": des_time_list,
                 "des_pos_list": des_pos_list,
                 "des_vel_list": des_vel_list,
                 "des_tau_list": des_tau_list,
                 "meas_time_list": meas_time_list,
                 "meas_pos_list": meas_pos_list,
                 "meas_vel_list": meas_vel_list,
                 "meas_tau_list": meas_tau_list,
                 "vel_filt_list": vel_filt_list,
                 "n": n,
                 "dt": dt,
                 "t": t}
    return data_dict


def prepare_trajectory(csv_path):
    """
    inputs:
        csv_path: string
            path to a csv file containing a trajectory in the
            below specified format

    The csv file should have 4 columns with values for
    [time, position, velocity, torque] respectively.
    The values shopuld be separated with a comma.
    Each row in the file is one timestep. The number of rows can vary.
    The values are assumed to be in SI units, i.e. time in s, position in rad,
    velocity in rad/s, torque in Nm.
    The first line in the csv file is reserved for comments
    and will be skipped during read out.

    Example:

        # time, position, velocity, torque
        0.00, 0.00, 0.00, 0.10
        0.01, 0.01, 0.01, -0.20
        0.02, ....

    """
    # load trajectories from csv file
    trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
    des_time_list = trajectory.T[0].T                       # desired time in s
    des_pos_list = trajectory.T[1].T               # desired position in radian
    des_vel_list = trajectory.T[2].T             # desired velocity in radian/s
    des_tau_list = trajectory.T[3].T                     # desired torque in Nm

    n = len(des_time_list)
    t = des_time_list[n - 1]
    dt = round((des_time_list[n - 1] - des_time_list[0]) / n, 3)

    # create 4 empty numpy array, where measured data can be stored
    meas_time_list = np.zeros(n)
    meas_pos_list = np.zeros(n)
    meas_vel_list = np.zeros(n)
    meas_tau_list = np.zeros(n)
    vel_filt_list = np.zeros(n)

    data_dict = {"des_time_list": des_time_list,
                 "des_pos_list": des_pos_list,
                 "des_vel_list": des_vel_list,
                 "des_tau_list": des_tau_list,
                 "meas_time_list": meas_time_list,
                 "meas_pos_list": meas_pos_list,
                 "meas_vel_list": meas_vel_list,
                 "meas_tau_list": meas_tau_list,
                 "vel_filt_list": vel_filt_list,
                 "n": n,
                 "dt": dt,
                 "t": t}
    return data_dict


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


def save(output_folder, data_dict):
    des_time = data_dict["des_time_list"]
    des_pos = data_dict["des_pos_list"]
    des_vel = data_dict["des_vel_list"]
    des_tau = data_dict["des_tau_list"]
    meas_time = data_dict["meas_time_list"]
    meas_pos = data_dict["meas_pos_list"]
    meas_vel = data_dict["meas_vel_list"]
    meas_tau = data_dict["meas_tau_list"]

    os.makedirs(output_folder)

    measured = np.array([np.array(meas_time),
                         np.array(meas_pos),
                         np.array(meas_vel),
                         np.array(meas_tau)]).T
    np.savetxt(output_folder + '/data_measured.csv', measured,
               delimiter=',', header="time,position,velocity,torque",
               comments="")

    desired = np.array([np.array(des_time),
                        np.array(des_pos),
                        np.array(des_vel),
                        np.array(des_tau)]).T
    np.savetxt(output_folder + '/data_desired.csv', desired,
               delimiter=',', header="time,position,velocity,torque",
               comments="")
    print(f'Saving .csv data to folder {output_folder}')
