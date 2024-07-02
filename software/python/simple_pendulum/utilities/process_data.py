"""
Data Processing
===============
"""

import os
import pandas as pd
import numpy as np
import copy


def load_trajectory(csv_path):
    """
    Load trajectory from csv file.
    The csv file should have 4 columns with values for
    [time, position, velocity, torque] respectively.
    The values shopuld be separated with a comma.
    Each row in the file is one timestep. The number of rows can vary.
    The values are assumed to be in SI units, i.e. time in s, position in rad,
    velocity in rad/s, torque in Nm.
    The first line in the csv file is reserved for comments
    and will be skipped during read out.

    Example:

        # des_time, despos, des_vel, des_tau
        0.00, 0.00, 0.00, 0.10
        0.01, 0.01, 0.01, -0.20
        0.02, ....

    Parameters
    ----------
        csv_path: string
            path to a csv file containing a trajectory in the
            below specified format

    Returns
    -------
        dict : dictionary with trajectory data
            the dictionary keys are:
                - "des_time"
                - "des_pos"
                - "des_vel"
                - "des_tau"
                - "meas_time"
                - "meas_pos"
                - "meas_vel"
                - "meas_tau"
                - "vel_filt"
            If the csv file contains no data for one of the keys they are
            filled with zeroes.
    """
    # load trajectories from csv file
    data = pd.read_csv(csv_path)

    n = len(data[data.keys()[1]])

    data_dict = {
        "des_time": np.zeros(n),
        "des_pos": np.zeros(n),
        "des_vel": np.zeros(n),
        "des_tau": np.zeros(n),
        "meas_time": np.zeros(n),
        "meas_pos": np.zeros(n),
        "meas_vel": np.zeros(n),
        "meas_tau": np.zeros(n),
        "vel_filt": np.zeros(n),
    }

    if "des_time" in data.keys():
        data_dict["des_time"] = np.asarray(data["des_time"])
    elif "time" in data.keys():
        data_dict["des_time"] = np.asarray(data["time"])

    if "des_pos" in data.keys():
        data_dict["des_pos"] = np.asarray(data["des_pos"])
    elif "position" in data.keys():
        data_dict["des_pos"] = np.asarray(data["position"])
    elif "pos" in data.keys():
        data_dict["des_pos"] = np.asarray(data["pos"])

    if "des_vel" in data.keys():
        data_dict["des_vel"] = np.asarray(data["des_vel"])
    elif "velocity" in data.keys():
        data_dict["des_vel"] = np.asarray(data["velocity"])
    elif "vel" in data.keys():
        data_dict["des_vel"] = np.asarray(data["vel"])

    if "des_tau" in data.keys():
        data_dict["des_tau"] = np.asarray(data["des_tau"])
    elif "torque" in data.keys():
        data_dict["des_tau"] = np.asarray(data["torque"])
    elif "tau" in data.keys():
        data_dict["des_tau"] = np.asarray(data["tau"])

    if "meas_time" in data.keys():
        data_dict["meas_time"] = np.asarray(data["meas_time"])

    if "meas_pos" in data.keys():
        data_dict["meas_pos"] = np.asarray(data["meas_pos"])

    if "meas_vel" in data.keys():
        data_dict["meas_vel"] = np.asarray(data["meas_vel"])

    if "meas_tau" in data.keys():
        data_dict["meas_tau"] = np.asarray(data["meas_tau"])

    if "vel_filt" in data.keys():
        data_dict["vel_filt"] = np.asarray(data["vel_filt"])

    return data_dict


def save_trajectory(csv_path, data_dict):
    """
    Save trajectory data to csv file.

    Parameters
    ----------
        csv_path: string
            path where a csv file containing the trajectory data
            will be stored
        data_dict : dict
            dictionary containing the trajectory data.
            expected dictionary keys:
                - "des_time"
                - "des_pos"
                - "des_vel"
                - "des_tau"
                - "meas_time"
                - "meas_pos"
                - "meas_vel"
                - "meas_tau"
                - "vel_filt"
    """
    if not os.path.exists(os.path.dirname(csv_path)):
        os.makedirs(os.path.dirname(csv_path))

    data = [
        data_dict["des_time"],
        data_dict["des_pos"],
        data_dict["des_vel"],
        data_dict["des_tau"],
        data_dict["meas_time"],
        data_dict["meas_pos"],
        data_dict["meas_vel"],
        data_dict["meas_tau"],
    ]

    data = np.asarray(data).T

    header = "des_time,des_pos,des_vel,des_tau,meas_time,meas_pos,meas_vel,meas_tau"

    np.savetxt(csv_path, data, delimiter=",", header=header, comments="")
    print(f"Saved .csv data to folder {csv_path}")


def prepare_empty_data_dict(dt, tf, n=None):
    """
    Prepare empty data/trajectory dictionary.
    Used in real time experiments.

    Parameters
    ----------
        dt : float
            timestep in [s]
        tf : float
            final time in [s]

    Returns
    -------
        dict : data dictionary
            all entries are filled with zeros
    """
    if n is None:
        n = int(tf / dt)

    # create 4 empty numpy array, where measured data can be stored
    des_time = np.zeros(n)
    des_pos = np.zeros(n)
    des_vel = np.zeros(n)
    des_tau = np.zeros(n)

    meas_time = np.zeros(n)
    meas_pos = np.zeros(n)
    meas_vel = np.zeros(n)
    meas_tau = np.zeros(n)
    vel_filt = np.zeros(n)

    data_dict = {
        "des_time": des_time,
        "des_pos": des_pos,
        "des_vel": des_vel,
        "des_tau": des_tau,
        "meas_time": meas_time,
        "meas_pos": meas_pos,
        "meas_vel": meas_vel,
        "meas_tau": meas_tau,
        "vel_filt": vel_filt,
    }
    return data_dict


def cut_trajectory(data_dict, key="meas_time"):
    n = np.nonzero(data_dict[key])[0][-1] + 1

    for k in data_dict.keys():
        data_dict[k] = data_dict[k][:n]

    return data_dict


def data_dict_from_TXU(T, X, U):
    dt = T[1] - T[0]
    tf = T[-1] + dt
    n = len(T)

    data_dict = prepare_empty_data_dict(dt, tf, n=n)
    data_dict["meas_time"] = T
    data_dict["meas_pos"] = np.asarray(X).T[0]
    data_dict["meas_vel"] = np.asarray(X).T[1]
    data_dict["meas_tau"] = U

    return data_dict


def TXU_from_data_dict(data_dict):
    T = data_dict["meas_time"]

    P = data_dict["meas_pos"]
    V = data_dict["meas_vel"]
    X = np.concatenate([[P], [V]], axis=0)

    U = data_dict["meas_tau"]
    return T, X, U


def saveFunnel(rho, S_t, time, max_dt, N, estMethod=""):
    S = S_t.value(time[0]).flatten()
    for i in range(1, len(time)):
        S = np.vstack((S, S_t.value(time[i]).flatten()))

    log_dir = "log_data/funnel"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    csv_data = np.vstack((rho, np.array(S).T))

    csv_path = os.path.join(log_dir, estMethod + f"funnel{max_dt}-{N}.csv")
    np.savetxt(csv_path, csv_data, delimiter=",", header="rho,S_t", comments="")


def getEllipseFromCsv(csv_path, index):
    data = np.loadtxt(csv_path, skiprows=1, delimiter=",")

    rho = data[0].T[index]

    S_t = data[1 : len(data)].T[index]
    state_dim = int(np.sqrt(len(data) - 1))
    S_t = np.reshape(S_t, (state_dim, state_dim))

    return rho, S_t


# def read(WORK_DIR, params_file, urdf_file, csv_file):
#     csv_path = str(WORK_DIR) + "/data/trajectories/" + csv_file
#     urdf_path = str(WORK_DIR) + "/data/urdf/" + urdf_file
#     params_path = str(WORK_DIR) + "/data/parameters/" + params_file
#     data = pd.read_csv(csv_path)
#     n = len(data)
#     return urdf_path, params_path, csv_path, data, n


# def cut(data_measured, data_desired):
#     nm = len(data_measured)       # compare length of desired and measured data
#     n = len(data_desired)
#     cut_s = 930,
#     cut_e = 1570,

#     # cut data at the start of the measurement
#     data_measured = data_measured.drop(data_measured.index[range(cut_s)])
#     data_desired = data_desired.drop(data_desired.index[range(cut_s)])
#     nm_cs = len(data_measured)
#     n_cs = len(data_desired)

#     # cut data at the end of the measurement
#     data_measured = data_measured.drop(data_measured.index[range(nm_cs-cut_e,
#                                                                  nm_cs)])

#     data_desired = data_desired.drop(data_desired.index[range(n_cs-(cut_e-500),
#                                                               n_cs)])
#     nm_cut = len(data_measured)
#     n_cut = len(data_desired)

#     # reset the row index, so that the first row of the clean data set has
#     # index 0 again
#     data_measured = data_measured.reset_index(drop=True)
#     data_desired = data_desired.reset_index(drop=True)
#     print("'cut_data' function output")
#     print("number of all data points:")
#     print("desired =", n)
#     print("measured =", nm)
#     print("number of data points with clean start:")
#     print("desired =", n_cs)
#     print("measured =", nm_cs)
#     print("number of data points with clean start + end:")
#     print("desired =", n_cut)
#     print("measured =", nm_cut)
#     print()
#     return data_measured, data_desired, n_cut
