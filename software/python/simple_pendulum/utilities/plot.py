"""
Plotting
========
"""


# global imports
import os
import numpy as np
import matplotlib.pyplot as plt

# local imports
from simple_pendulum.utilities.filters import running_mean as rm


def swingup(save, output_folder, data_dict):
    print("Making data plots.")

    des_time = data_dict["des_time_list"]
    des_pos = data_dict["des_pos_list"]
    des_vel = data_dict["des_vel_list"]
    des_tau = data_dict["des_tau_list"]
    meas_time = data_dict["meas_time_list"]
    meas_pos = data_dict["meas_pos_list"]
    meas_vel = data_dict["meas_vel_list"]
    # meas_vel_filt = data_dict["vel_filt_list"]
    meas_tau = data_dict["meas_tau_list"]

    plt.figure()
    plt.plot(meas_time, meas_pos)
    plt.plot(des_time, des_pos)
    plt.xlabel("Time (s)")
    plt.ylabel("Position (rad)")
    plt.title("Position (rad) vs Time (s)")
    plt.legend(['position_measured', 'position_desired'])
    plt.draw()
    if save:
        plt.savefig(output_folder + '/swingup_pos.pdf')
    plt.show()

    plt.figure()
    plt.plot(meas_time, meas_vel)
    # plt.plot(meas_time, meas_vel_filt)
    plt.plot(des_time, des_vel)
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (rad/s)")
    plt.legend(['velocity_measured', 'velocity_desired'])
    plt.title("Velocity (rad/s) vs Time (s)")
    plt.draw()
    if save:
        plt.savefig(output_folder + '/swingup_vel.pdf')
    plt.show()

    plt.figure()
    plt.plot(meas_time, meas_tau)
    plt.plot(des_time, des_tau)
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (Nm)")
    plt.title("Torque (Nm) vs Time (s)")
    plt.legend(['Measured Torque', 'Desired Torque'])
    plt.draw()
    if save:
        plt.savefig(output_folder + '/swingup_tau.pdf')
    plt.show()


def grav_comp(args, output_folder, data_dict):
    print("Making data plots.")

    des_time = data_dict["des_time_list"]
    # des_pos = data_dict["des_pos_list"]
    # des_vel = data_dict["des_vel_list"]
    des_tau = data_dict["des_tau_list"]
    meas_time = data_dict["meas_time_list"]
    meas_pos = data_dict["meas_pos_list"]
    meas_vel = data_dict["meas_vel_list"]
    meas_tau = data_dict["meas_tau_list"]

    plt.figure()
    plt.plot(meas_time, meas_pos)
    plt.xlabel("Time (s)")
    plt.ylabel("Position (rad)")
    plt.title("Position (rad) vs Time (s)")
    plt.draw()
    if args.save:
        plt.savefig(output_folder + '/grav_comp_pos.pdf')
    plt.show()

    plt.figure()
    plt.plot(meas_time, meas_vel)
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (rad/s)")
    plt.title("Velocity (rad/s) vs Time (s)")
    plt.draw()
    if args.save:
        plt.savefig(output_folder + '/grav_comp_vel.pdf')
    plt.show()

    plt.figure()
    plt.plot(meas_time, meas_tau)
    plt.plot(des_time, des_tau)
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (Nm)")
    plt.title("Torque (Nm) vs Time (s)")
    plt.legend(['Measured', 'Desired'])
    plt.draw()
    if args.save:
        plt.savefig(output_folder + '/grav_comp_tau.pdf')
    plt.show()

    time_vec_filtered = rm.data_filter(np.array(meas_time), 10)
    filtered_torque = rm.data_filter(np.array(meas_tau), 10)
    filtered_desired_torque = rm.data_filter(np.array(des_tau), 10)

    plt.figure()
    plt.plot(time_vec_filtered, filtered_torque)
    plt.plot(time_vec_filtered, filtered_desired_torque)
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (Nm)")
    plt.title(
        "Filtered Torque (Nm) vs Time (s) with" +
        " moving average filter (window = 100)")
    plt.legend(['Measured', 'Desired'])
    plt.draw()
    if args.save:
        plt.savefig(os.path.join(output_folder, 'grav_comp_tau_filt.pdf'))
    plt.show()


def sys_id_unified(output_folder, meas_time=None, meas_pos=None, meas_vel=None,
                   meas_tau=None, acc=None):
    plt.figure()
    if meas_pos is not None:
        plt.plot(meas_time, meas_pos, label="position")
    if meas_vel is not None:
        plt.plot(meas_time, meas_vel, label="velocity")
    if acc is not None:
        plt.plot(meas_time, acc, label="acceleration")
    if meas_tau is not None:
        plt.plot(meas_time, meas_tau, label="torque")
    plt.legend(loc="best")
    plt.draw()
    plt.savefig(os.path.join(output_folder, 'grav_comp_tau_filt.pdf'))
    plt.show()


def sys_id_comparison(output_folder, meas_time, vel_dict, tau_dict, acc_dict):
    t = meas_time
    vel_raw = vel_dict["vel_raw"]
    vel_fft = vel_dict["vel_fft"]
    vel_butter = vel_dict["vel_butter"]
    vel_grad = vel_dict["vel_grad"]
    vel_grad_butter = vel_dict["vel_grad_butter"]
    tau_raw = tau_dict["tau_raw"]
    tau_fft = tau_dict["tau_fft"]
    tau_butter = tau_dict["tau_butter"]
    acc_raw = acc_dict["acc_raw"]
    acc_butter = acc_dict["acc_butter"]
    acc_grad_butter = acc_dict["acc_grad_butter"]
    acc_grad_2butter = acc_dict["acc_grad_2butter"]

    plt.figure("velocity filter comparison", clear=True)
    plt.plot(t, vel_raw)
    plt.plot(t, vel_fft)
    plt.plot(t, vel_butter)
    plt.plot(t, vel_grad)
    plt.plot(t, vel_grad_butter)
    plt.legend(["vel raw",
                "vel fft",
                "vel butterworth",
                "pos/dt",
                "pos/dt + butterworth"])
    plt.savefig(os.path.join(output_folder, 'sys_id_vel_filt_comparison.pdf'))
    plt.show()

    plt.figure("torque filter comparison", clear=True)
    plt.plot(t, tau_raw)
    plt.plot(t, tau_fft)
    plt.plot(t, tau_butter)
    plt.legend(["torque raw", "torque fft", "torque butter"])

    plt.figure("accelereation filter comparison", clear=True)
    plt.plot(t, acc_raw)
    plt.plot(t, acc_butter)
    plt.plot(t, acc_grad_butter)
    plt.plot(t, acc_grad_2butter)
    plt.legend(["acc raw (vel/dt)",
                "acc (vel/dt + butterworth)",
                "acc (vel + butterworth/dt)",
                "acc 2x butterworth"])
    plt.savefig(os.path.join(output_folder, 'sys_id_tau_filt_comparison.pdf'))
    plt.show()


def sys_id_result(output_folder, t, ref_trq, est_trq):
    plt.figure("Measured torque vs. parameter estimation", clear=True)
    plt.plot(t, ref_trq)
    plt.plot(t, est_trq)
    plt.legend(["measured torque", "estimated torque"])
    plt.savefig(os.path.join(output_folder, 'sys_id_result.pdf'))
    plt.show()
