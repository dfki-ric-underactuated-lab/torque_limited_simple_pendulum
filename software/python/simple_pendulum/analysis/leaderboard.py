import os
import numpy as np

from simple_pendulum.utilities.process_data import load_trajectory


def leaderboard_scores(
    data_paths,
    save_to,
    weights={
        "swingup_time": 0.2,
        "max_tau": 0.1,
        "energy": 0.0,
        "integ_tau": 0.1,
        "tau_cost": 0.0,
        "tau_smoothness": 0.6,
    },
    normalize={
        "swingup_time": 10.0,
        "max_tau": 1.0,
        "energy": 1.0,
        "integ_tau": 10.0,
        "tau_cost": 10.0,
        "tau_smoothness": 1.0,
    },
    link_base="",
    simulation=True,
):
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
        if type(d["csv_path"]) == str:
            csv_paths = [d["csv_path"]]
        else:
            csv_paths = d["csv_path"]

        swingup_times = []
        max_taus = []
        energies = []
        integ_taus = []
        tau_costs = []
        tau_smoothnesses = []
        velocity_costs = []
        successes = []
        scores = []

        for path in sorted(csv_paths):
            data_dict = load_trajectory(path)
            T = data_dict["meas_time"]
            # X = data_dict["meas_pos"]
            # U = data_dict["meas_vel"]

            swingup_times.append(
                get_swingup_time(
                    data_dict,
                    has_to_stay=True,
                )
            )
            max_taus.append(get_max_tau(data_dict))
            energies.append(get_energy(data_dict))
            integ_taus.append(get_integrated_torque(data_dict))
            tau_costs.append(get_torque_cost(data_dict))
            tau_smoothnesses.append(get_tau_smoothness(data_dict))
            velocity_costs.append(get_velocity_cost(data_dict))

            successes.append(int(swingup_times[-1] < T[-1]))

            score = successes[-1] * (
                1.0
                - (
                    weights["swingup_time"]
                    * swingup_times[-1]
                    / normalize["swingup_time"]
                    + weights["max_tau"] * max_taus[-1] / normalize["max_tau"]
                    + weights["energy"] * energies[-1] / normalize["energy"]
                    + weights["integ_tau"] * integ_taus[-1] / normalize["integ_tau"]
                    + weights["tau_cost"] * tau_costs[-1] / normalize["tau_cost"]
                    + weights["tau_smoothness"]
                    * tau_smoothnesses[-1]
                    / normalize["tau_smoothness"]
                    + weights["velocity_cost"]
                    * velocity_costs[-1]
                    / normalize["velocity_cost"]
                )
            )

            scores.append(score)

            results = np.array(
                [
                    [successes[-1]],
                    [swingup_times[-1]],
                    [energies[-1]],
                    [max_taus[-1]],
                    [integ_taus[-1]],
                    [tau_costs[-1]],
                    [tau_smoothnesses[-1]],
                    [velocity_costs[-1]],
                    [score],
                ]
            ).T

            np.savetxt(
                os.path.join(os.path.dirname(path), "scores.csv"),
                results,
                header="Swingup Success,Swingup Time [s],Energy [J],Max. Torque [Nm],Integrated Torque [Nms],Torque Cost[N²m²],Torque Smoothness [Nm],Velocity Cost [m²/s²],RealAI Score",
                delimiter=",",
                fmt="%s",
                comments="",
            )

        best = np.argmax(scores)
        swingup_time = swingup_times[best]
        max_tau = max_taus[best]
        energy = energies[best]
        integ_tau = integ_taus[best]
        tau_cost = tau_costs[best]
        tau_smoothness = tau_smoothnesses[best]
        velocity_cost = velocity_costs[best]
        success = np.sum(successes)
        score = np.mean(scores)
        best_score = np.max(scores)

        if link_base != "":
            if "simple_name" in d.keys():
                name_with_link = (
                    f"[{d['simple_name']}]({link_base}{d['name']}/README.md)"
                )
            else:
                name_with_link = f"[{d['name']}]({link_base}{d['name']}/README.md)"
        else:
            if "simple_name" in d.keys():
                name_with_link = d["simple_name"]
            else:
                name_with_link = d["name"]

        if simulation:
            append_data = [
                name_with_link,
                d["short_description"],
                str(int(success)) + "/" + str(len(csv_paths)),
                str(round(swingup_time, 2)),
                str(round(energy, 2)),
                str(round(max_tau, 2)),
                str(round(integ_tau, 2)),
                str(round(tau_cost, 2)),
                str(round(tau_smoothness, 3)),
                str(round(velocity_cost, 2)),
                str(round(score, 3)),
                d["username"],
            ]
        else:
            append_data = [
                name_with_link,
                d["short_description"],
                str(int(success)) + "/" + str(len(csv_paths)),
                str(round(swingup_time, 2)),
                str(round(energy, 2)),
                str(round(max_tau, 2)),
                str(round(integ_tau, 2)),
                str(round(tau_cost, 2)),
                str(round(tau_smoothness, 3)),
                str(round(velocity_cost, 2)),
                str(round(best_score, 3)),
                str(round(score, 3)),
                d["username"],
            ]

        if link_base != "":
            controller_link = link_base + d["name"]

            if simulation:
                data_link = "[data](" + controller_link + "/sim_swingup.csv)"
                plot_link = "[plot](" + controller_link + "/timeseries.png)"
                video_link = "[video](" + controller_link + "/sim_video.gif)"
                append_data.append(data_link + " " + plot_link + " " + video_link)
            else:
                data_link = (
                    "[data]("
                    + controller_link
                    + "/experiment"
                    + str(best + 1).zfill(2)
                    + "/trajectory.csv)"
                )
                plot_link = (
                    "[plot]("
                    + controller_link
                    + "/experiment"
                    + str(best + 1).zfill(2)
                    + "/timeseries.png)"
                )
                video_link = (
                    "[video]("
                    + controller_link
                    + "/experiment"
                    + str(best + 1).zfill(2)
                    + "/video.gif)"
                )
                # link = "[data plots videos](" + controller_link + ")"
                # append_data.append(link)
                append_data.append(data_link + " " + plot_link + " " + video_link)

        leaderboard_data.append(append_data)

    if simulation:
        header = "Controller,Short Controller Description,Swingup Success,Swingup Time [s],Energy [J],Max. Torque [Nm],Integrated Torque [Nms],Torque Cost[N²m²],Torque Smoothness [Nm],Velocity Cost [m²/s²],RealAI Score,Username"
    else:
        header = "Controller,Short Controller Description,Swingup Success,Swingup Time [s],Energy [J],Max. Torque [Nm],Integrated Torque [Nms],Torque Cost[N²m²],Torque Smoothness [Nm],Velocity Cost [m²/s²],Best RealAI Score,Average RealAI Score,Username"
    if link_base != "":
        header += ",Data"

    np.savetxt(
        save_to,
        leaderboard_data,
        header=header,
        delimiter=",",
        fmt="%s",
        comments="",
    )


def get_swingup_time(data_dict, eps=[2e-2, 2e-1], has_to_stay=True):
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
        default = [2e-2, 2e-1]
    has_to_stay : bool
        whether the pendulum has to stay upright until the end of the trajectory
        default=True

    Returns
    -------
    float
        swingup time
    """
    goal = np.array([np.pi, 0.0])

    dist_pos = np.abs(np.mod(data_dict["meas_pos"], 2 * np.pi) - goal[0])
    ddist_pos = np.where(dist_pos < eps[0], 0.0, dist_pos)
    # n_pos = np.nonzero(ddist_pos)[0][-1] + 1
    n_pos = np.argwhere(ddist_pos == 0.0)

    dist_vel = np.abs(data_dict["meas_vel"] - goal[1])
    ddist_vel = np.where(dist_vel < eps[1], 0.0, dist_vel)
    # n_vel = np.nonzero(ddist_vel)[0][-1] + 1
    n_vel = np.argwhere(ddist_vel == 0.0)

    n = np.intersect1d(n_pos, n_vel)

    time_index = len(data_dict["meas_time"]) - 1
    if has_to_stay:
        if len(n) > 0:
            for i in range(len(n) - 2, 0, -1):
                if n[i] + 1 == n[i + 1]:
                    time_index = n[i]
                else:
                    break
    else:
        if len(n) > 0:
            time_index = n[0]
    time = data_dict["meas_time"][time_index]

    return time


def get_max_tau(data_dict):
    """get_max_tau.

    Get the maximum torque used in the trajectory.

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

    Returns
    -------
    float
        maximum torque
    """
    tau = np.max(np.abs(data_dict["meas_tau"]))
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

    Returns
    -------
    float
        energy
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

    Returns
    -------
    float
        integrated torque
    """
    delta_t = np.diff(data_dict["meas_time"])
    tau = np.abs(data_dict["meas_tau"][:-1])
    int_tau = np.sum(tau * delta_t)
    return int_tau


def get_torque_cost(data_dict, R=1.0):
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

    Returns
    -------
    float
        torque cost
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

    Returns
    -------
    float
        torque smoothness (std of changes)
    """
    u = data_dict["meas_tau"][:-1]
    u_diff = np.diff(u)
    std = np.std(u_diff)
    return std


def get_velocity_cost(data_dict, Q=1.0):
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

    Returns
    -------
    float
        torque cost
    """
    delta_t = np.diff(data_dict["meas_time"])
    qd = data_dict["meas_vel"][:-1]
    cost = np.sum(qd * Q * delta_t * qd)
    return cost
