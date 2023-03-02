import numpy as np

from simple_pendulum.utilities.process_data import load_trajectory


def leaderboard_scores(data_paths,
                       save_to,
                       weights={"swingup_time": 0.2,
                                "max_tau": 0.1,
                                "energy": 0.0,
                                "integ_tau": 0.1,
                                "tau_cost": 0.0,
                                "tau_smoothness": 0.6},
                       normalize={"swingup_time": 10.,
                                  "max_tau": 1.0,
                                  "energy": 1.0,
                                  "integ_tau": 10.0,
                                  "tau_cost": 10.0,
                                  "tau_smoothness": 1.0}):
    """leaderboard_scores.
    Compute leaderboard scores from data_dictionaries which will be loaded from
    data_paths.  Data can be either from simulation or experiments (but for
    comparability it should only be one).

    Parameters
    ----------
    data_paths : dict
        contains the names and paths to the trajectory data in the form:
        {controller1_name: {"csv_path": data_path1, "name": controller1_name, "username": username1},
         controller2_name: {"csv_path": data_path2, "name": controller2_name, "username": username2},
         ...}
    save_to : string
        path where the result will be saved as a csv file
    weights : dict
        dictionary containing the weights for the different criteria in the
        form:
        {"swingup_time": weight1,
         "max_tau": weight2,
         "energy": weight3,
         "integ_tau": weight4,
         "tau_cost": weight5,
         "tau_smoothness": weight6}
         The weights should sum up to 1 for the final score to be in the range
         [0, 1].
    normalize : dict
        dictionary containing normalization constants for the different
        criteria in the form:
        {"swingup_time": norm1,
         "max_tau": norm2,
         "energy": norm3,
         "integ_tau": norm4,
         "tau_cost": norm5,
         "tau_smoothness": norm6}
         The normalization constants should be the maximum values that can be
         achieved by the criteria so that after dividing by the norm the result
         is in the range [0, 1].
    """

    leaderboard_data = []

    for key in data_paths:
        d = data_paths[key]
        data_dict = load_trajectory(d["csv_path"])

        swingup_time = get_swingup_time(data_dict)
        max_tau = get_max_tau(data_dict)
        energy = get_energy(data_dict)
        integ_tau = get_integrated_torque(data_dict)
        tau_cost = get_torque_cost(data_dict)
        tau_smoothness = get_tau_smoothness(data_dict)

        score = weights["swingup_time"] * swingup_time / normalize["swingup_time"] + \
                weights["max_tau"] * max_tau / normalize["max_tau"] + \
                weights["energy"] * energy / normalize["energy"] + \
                weights["integ_tau"] * integ_tau / normalize["integ_tau"] + \
                weights["tau_cost"] * tau_cost / normalize["tau_cost"] + \
                weights["tau_smoothness"] * tau_smoothness / normalize["tau_smoothness"]

        score = 1 - score

        leaderboard_data.append([d["name"],
                                 str(swingup_time),
                                 str(energy),
                                 str(max_tau),
                                 str(integ_tau),
                                 str(tau_cost),
                                 str(tau_smoothness),
                                 str(score),
                                 d["username"]])

    np.savetxt(save_to,
               leaderboard_data,
               header="Controller,Swingup Time,Energy,Max. Torque,Integrated Torque,Torque Cost,Torque Smoothness,Real AI Score, Username",
               delimiter=",",
               fmt="%s",
               comments="")


def get_swingup_time(data_dict, eps=[2.e-2, 2e-1]):
    """get_swingup_time.
    get the swingup time from a data_dict.

    Parameters
    ----------
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
    eps : list
        list with len(eps) = 2. The thresholds for the swingup to be
        successfull ([position, velocity])
    """
    goal = np.array([np.pi, 0.])

    dist_pos = np.abs(np.mod(data_dict["meas_pos"], 2 * np.pi) - goal[0])
    ddist_pos = np.where(dist_pos < eps[0], 0., dist_pos)
    n_pos = np.nonzero(ddist_pos)[0][-1] + 1

    dist_vel = np.abs(data_dict["meas_vel"] - goal[1])
    ddist_vel = np.where(dist_vel < eps[1], 0., dist_vel)
    n_vel = np.nonzero(ddist_vel)[0][-1] + 1

    n = max(n_pos, n_vel)
    m = max(0, n - 1)
    time = data_dict["meas_time"][m]

    return time

def get_max_tau(data_dict):
    """get_max_tau.

    Get the maximum torqu used in the trajectory.

    Parameters
    ----------
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
    tau = np.max(data_dict["meas_tau"])
    return tau

def get_energy(data_dict):
    """get_energy.

    Get the mechanical energy used during the swingup.

    Parameters
    ----------
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
    delta_pos = np.diff(data_dict["meas_pos"])
    tau = data_dict["meas_tau"][:-1]
    energy = np.sum(np.abs(delta_pos * tau))
    return energy

def get_integrated_torque(data_dict):
    """get_integrated_torque.

    Get the (discrete) time integral over the torque.

    Parameters
    ----------
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
    delta_t = np.diff(data_dict["meas_time"])
    tau = np.abs(data_dict["meas_tau"][:-1])
    int_tau = np.sum(tau * delta_t)
    return int_tau

def get_torque_cost(data_dict, R=1.):
    """get_torque_cost.

    Get the running cost torque with cost parameter R.
    The cost is normalized with the timestep.

    Parameters
    ----------
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
    R : float
        running cost weight
    """
    delta_t = np.diff(data_dict["meas_time"])
    u = data_dict["meas_tau"][:-1]
    cost = np.sum(u * R * delta_t * u)
    return cost

def get_tau_smoothness(data_dict):
    """get_tau_smoothness.

    Get the standard deviation of the changes in the torque signal.

    Parameters
    ----------
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
    u = data_dict["meas_tau"][:-1]
    u_diff = np.diff(u)
    std = np.std(u_diff)
    return std
