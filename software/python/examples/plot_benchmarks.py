import numpy as np
import yaml
import os
import matplotlib as mpl
import matplotlib.pyplot as plt

font = {'family' : 'Arial',
        'weight' : 'normal',
        'size'   : 22}

mpl.rc('font', **font)

datadir = "../../../data/benchmarks"
savedir = "log_data"
alignment = "horizontal"

workdir = os.getcwd()
data = {}
labels = []

ilqr_open_path = os.path.join(workdir, datadir, "ilqr_open/benchmark.yml")
with open(ilqr_open_path, "r") as f:
    data["ilqr_open"] = yaml.safe_load(f)
labels.append("ilqr + ff")

ilqr_pid_path = os.path.join(workdir, datadir, "ilqr_pid/benchmark.yml")
with open(ilqr_pid_path, "r") as f:
    data["ilqr_pid"] = yaml.safe_load(f)
labels.append("ilqr + pid")

ilqr_tvlqr_path = os.path.join(workdir, datadir, "ilqr_tvlqr/benchmark.yml")
with open(ilqr_tvlqr_path, "r") as f:
    data["ilqr_tvlqr"] = yaml.safe_load(f)
labels.append("ilqr + tvlqr")

# dircol_open_path = os.path.join(workdir, datadir, "dircol_open/benchmark.yml")
# with open(dircol_open_path, "r") as f:
#     data["dircol_open"] = yaml.safe_load(f)

dircol_pid_path = os.path.join(workdir, datadir, "dircol_pid/benchmark.yml")
with open(dircol_pid_path, "r") as f:
    data["dircol_pid"] = yaml.safe_load(f)
labels.append("dircol + pid")

dircol_tvlqr_path = os.path.join(workdir, datadir, "dircol_tvlqr/benchmark.yml")
with open(dircol_tvlqr_path, "r") as f:
    data["dircol_tvlqr"] = yaml.safe_load(f)
labels.append("dircol + tvlqr")

# ddp_open_path = os.path.join(workdir, datadir, "ddp_open/benchmark.yml")
# with open(ddp_open_path, "r") as f:
#     data["ddp_open"] = yaml.safe_load(f)

ddp_pid_path = os.path.join(workdir, datadir, "ddp_pid/benchmark.yml")
with open(ddp_pid_path, "r") as f:
    data["ddp_pid"] = yaml.safe_load(f)
labels.append("ddp + pid")

ddp_tvlqr_path = os.path.join(workdir, datadir, "ddp_tvlqr/benchmark.yml")
with open(ddp_tvlqr_path, "r") as f:
    data["ddp_tvlqr"] = yaml.safe_load(f)
labels.append("ddp + tvlqr")

energy_path = os.path.join(workdir, datadir, "energy_shaping/benchmark.yml")
with open(energy_path, "r") as f:
    data["energy"] = yaml.safe_load(f)
labels.append("energy + lqr")

ilqrMPC_path = os.path.join(workdir, datadir, "ilqr_mpc/benchmark.yml")
with open(ilqrMPC_path, "r") as f:
    data["ilqrMPC"] = yaml.safe_load(f)
labels.append("mpc ilqr")

sac_path = os.path.join(workdir, datadir, "sac/benchmark.yml")
with open(sac_path, "r") as f:
    data["sac"] = yaml.safe_load(f)
labels.append("sac")

ddpg_path = os.path.join(workdir, datadir, "ddpg/benchmark.yml")
with open(ddpg_path, "r") as f:
    data["ddpg"] = yaml.safe_load(f)
labels.append("ddpg")

print(data.keys())

frequencies = []
times = []
energies = []
smoothnesses = []
consistencies = []
stabilities = []
sensitivities = []
min_torques = []

for k in data.keys():
    frequencies.append(data[k]["frequency"])
    times.append(data[k]["swingup_time"])
    energies.append(data[k]["energy"])
    smoothnesses.append(data[k]["smoothness"])
    consistencies.append(data[k]["consistency"] / data[k]["iterations"])
    stabilities.append(data[k]["robustness"] / data[k]["iterations"])
    sensitivities.append(data[k]["sensitivity"] / data[k]["iterations"])
    min_torques.append(data[k]["min_successful_torque"])


if alignment == "vertical":
    fig = plt.figure(figsize=(14, 16))

    # large values are good
    ax_freq = plt.subplot(421)
    ax_freq.bar(labels, frequencies, color="tab:green")
    ax_freq.set_yscale("log")
    ax_freq.set_ylabel("frequency [Hz]", fontsize=20)
    plt.setp(ax_freq.get_xticklabels(), visible=False)

    ax_cons = plt.subplot(423, sharex=ax_freq)
    ax_cons.bar(labels, np.asarray(consistencies)*100, color="tab:green")
    ax_cons.set_ylabel("consistency [%]", fontsize=20)
    ax_cons.set_ylim(0, 100)
    plt.setp(ax_cons.get_xticklabels(), visible=False)

    ax_stab = plt.subplot(425, sharex=ax_freq)
    ax_stab.bar(labels, np.asarray(stabilities)*100, color="tab:green")
    ax_stab.set_ylabel("robustness [%]", fontsize=20)
    ax_stab.set_ylim(0, 100)
    plt.setp(ax_stab.get_xticklabels(), visible=False)

    ax_sens = plt.subplot(427, sharex=ax_freq)
    ax_sens.bar(labels, np.asarray(sensitivities)*100, color="tab:green")
    ax_sens.set_ylabel("insensitivity [%]", fontsize=20)
    ax_sens.set_ylim(0, 100)
    plt.xticks(rotation=45, ha="right", rotation_mode="anchor")

    # small values are good
    ax_time = plt.subplot(422)
    ax_time.bar(labels, times, color="tab:red")
    ax_time.set_ylabel("swingup time [s]", fontsize=20)
    plt.setp(ax_time.get_xticklabels(), visible=False)

    ax_energy = plt.subplot(424, sharex=ax_time)
    ax_energy.bar(labels, energies, color="tab:red")
    ax_energy.set_ylabel("energy [J]", fontsize=20)
    plt.setp(ax_energy.get_xticklabels(), visible=False)

    ax_smooth = plt.subplot(426, sharex=ax_time)
    ax_smooth.bar(labels, smoothnesses, color="tab:red")
    ax_smooth.set_ylabel("smoothness [Nm]", fontsize=20)
    ax_smooth.set_ylim(0, 0.8)
    plt.setp(ax_smooth.get_xticklabels(), visible=False)

    ax_mint = plt.subplot(428, sharex=ax_time)
    ax_mint.bar(labels, min_torques, color="tab:red")
    ax_mint.set_ylabel(r"Reduced $\tau_{max}$ [Nm]", fontsize=20)
    plt.xticks(rotation=45, ha="right", rotation_mode="anchor")

    ax_freq.get_shared_x_axes().join(ax_freq, ax_time)

    plt.savefig(os.path.join(savedir, "benchmark_barplot"),
                bbox_inches="tight")
    plt.show()

if alignment == "horizontal":
    fig = plt.figure(figsize=(16, 10))

    # large values are good
    ax_freq = plt.subplot(241)
    ax_freq.barh(labels, frequencies, color="tab:green")
    ax_freq.set_xscale("log")
    ax_freq.set_xlabel("frequency [Hz]", fontsize=20)
    ax_freq.xaxis.labelpad = 10
    ax_freq.xaxis.set_label_position("top")
    ax_freq.xaxis.tick_top()
    ax_freq.invert_yaxis()
    # plt.yticks(rotation=45, ha="right", rotation_mode="anchor")

    ax_cons = plt.subplot(242, sharey=ax_freq)
    ax_cons.barh(labels, np.asarray(consistencies)*100, color="tab:green")
    ax_cons.set_xlabel("consistency [%]", fontsize=20)
    ax_cons.xaxis.labelpad = 10
    ax_cons.set_xlim(0, 100)
    ax_cons.set_xticks([25, 50, 75, 100])
    ax_cons.xaxis.set_label_position("top")
    ax_cons.xaxis.tick_top()
    plt.setp(ax_cons.get_yticklabels(), visible=False)

    ax_stab = plt.subplot(243, sharey=ax_freq)
    ax_stab.barh(labels, np.asarray(stabilities)*100, color="tab:green")
    ax_stab.set_xlabel("robustness [%]", fontsize=20)
    ax_stab.xaxis.labelpad = 10
    ax_stab.set_xlim(0, 100)
    ax_stab.set_xticks([25, 50, 75, 100])
    ax_stab.xaxis.set_label_position("top")
    ax_stab.xaxis.tick_top()
    plt.setp(ax_stab.get_yticklabels(), visible=False)

    ax_sens = plt.subplot(244, sharey=ax_freq)
    ax_sens.barh(labels, np.asarray(sensitivities)*100, color="tab:green")
    ax_sens.set_xlabel("insensitivity [%]", fontsize=20)
    ax_sens.xaxis.labelpad = 10
    ax_sens.set_xlim(0, 100)
    ax_sens.set_xticks([25, 50, 75, 100])
    ax_sens.xaxis.set_label_position("top")
    ax_sens.xaxis.tick_top()
    plt.setp(ax_sens.get_yticklabels(), visible=False)

    # small values are good
    ax_time = plt.subplot(245)
    ax_time.barh(labels, times, color="tab:red")
    ax_time.set_xlabel("swingup time [s]", fontsize=20)
    ax_time.set_xticks([0, 2, 4, 6, 8])
    ax_time.invert_yaxis()
    # plt.yticks(rotation=45, ha="right", rotation_mode="anchor")

    ax_energy = plt.subplot(246, sharey=ax_time)
    ax_energy.barh(labels, energies, color="tab:red")
    ax_energy.set_xlabel("energy [J]", fontsize=20)
    ax_energy.set_xticks([0, 2, 4, 6, 8, 10])
    plt.setp(ax_energy.get_yticklabels(), visible=False)

    ax_smooth = plt.subplot(247, sharey=ax_time)
    ax_smooth.barh(labels, smoothnesses, color="tab:red")
    ax_smooth.set_xlim(0, 0.8)
    ax_smooth.set_xlabel("smoothness [Nm]", fontsize=20)
    plt.setp(ax_smooth.get_yticklabels(), visible=False)

    ax_mint = plt.subplot(248, sharey=ax_time)
    ax_mint.barh(labels, min_torques, color="tab:red")
    ax_mint.set_xlabel(r"Reduced $\tau_{max}$ [Nm]", fontsize=20)
    plt.setp(ax_mint.get_yticklabels(), visible=False)

    ax_freq.get_shared_y_axes().join(ax_freq, ax_time)

    plt.subplots_adjust(wspace=0.1, hspace=0.1)
    plt.savefig(os.path.join(savedir, "benchmark_barplot"),
                bbox_inches="tight")
    plt.show()
