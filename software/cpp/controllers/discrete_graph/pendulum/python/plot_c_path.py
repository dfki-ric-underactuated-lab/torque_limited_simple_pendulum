import sys
import os
import numpy as np
import yaml

# sys.path.append("..")
from plot_pendulum import plotStatesAndTransitions, plotStatesAndTransitionsPhaseSpace
from utils import index_to_pos_vel


def plot_cmap(datapath):

    with open(os.path.join(datapath, "config.yml")) as f:
        config = yaml.safe_load(f)
    print("config ", config)
    vel_max = config["vmax"]
    deltaT = config["deltaT"]
    pos_res = (config["position_resolution"] / 360.0) * 2 * np.pi
    vel_res = (config["velocity_resolution"] / 360.0) * 2 * np.pi
    # dv = self.dp / self.dt
    veö_max = vel_max * vel_res

    # deltaT = 0.1
    # pos_res = 1.0 * 2*np.pi / 360.0
    # vel_res = pos_res / deltaT
    # vel_max = 50.0 * vel_res

    states_ind = np.loadtxt(os.path.join(datapath, "states_index.csv"), delimiter=",")
    transitions_ind = np.loadtxt(
        os.path.join(datapath, "transitions_index.csv"), delimiter=","
    )

    states = []
    for si in states_ind:
        pos, vel = index_to_pos_vel(
            si, pos_res, -np.pi, np.pi, vel_res, -vel_max, vel_max
        )
        states.append([pos, vel])

    transitions = []
    for ti in transitions_ind:
        pos1, vel1 = index_to_pos_vel(
            ti[0], pos_res, -np.pi, np.pi, vel_res, -vel_max, vel_max
        )
        pos2, vel2 = index_to_pos_vel(
            ti[1], pos_res, -np.pi, np.pi, vel_res, -vel_max, vel_max
        )
        transitions.append([pos1, vel1, pos2, vel2, ti[2], ti[3]])

    states = np.asarray(states)
    transitions = np.asarray(transitions)

    plotStatesAndTransitions(
        states,
        transitions,
        path=datapath,
        velocity_spread=0.5,
        save_string="state_space_graph.png",
        overlay=False,
    )


def plot_cpath(datapath):
    path = np.loadtxt(os.path.join(datapath, "path.csv"), delimiter=",", skiprows=1)
    path_vertices = []
    path_transitions = []
    print("shape of path", np.shape(path))
    for i in range(np.shape(path)[0] - 1):
        path_transitions.append(
            [path[i][1], path[i][2], path[i + 1][1], path[i + 1][2]]
        )
        path_vertices.append([path[i][1], path[i][2]])

    path_vertices.append([path[-1][1], path[-1][2]])
    path_vertices = np.asarray(path_vertices)
    plotStatesAndTransitions(
        path_vertices,
        path_transitions,
        path=datapath,
        velocity_spread=0.5,
        save_string="path.pdf",
        overlay=False,
    )


# datapath = "../Data/dp0.1_dt0.04_vmax200_ntau7/"
# datapath = "../Data/dp1.0_dt0.1_vmax50_ntau5/"
# datapath = "../Data/dp10.0_dt1.0_vmax50_ntau5/"
datapath = "/media/Volume_500GB/Work/DFKI/discrete-exploration/Data/pendulum/testbench_par/dp1.0_dt0.1_vmax50_ntau31"
if len(sys.argv) > 1:
    datapath = sys.argv[1]
# plot_cmap(datapath)
plot_cpath(datapath)
