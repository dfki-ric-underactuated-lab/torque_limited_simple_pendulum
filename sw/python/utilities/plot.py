# global imports
import numpy as np
import matplotlib.pyplot as plt

# local imports
from main import data_dict
from filters.running_mean import rm_filter

des_time = data_dict["des_time_list"]
des_pos = data_dict["des_pos_list"]
des_vel = data_dict["des_vel_list"]
des_tau = data_dict["des_tau_list"]
meas_time = data_dict["meas_time_list"]
meas_pos = data_dict["meas_pos_list"]
meas_vel = data_dict["meas_vel_list"]
meas_tau = data_dict["meas_tau_list"]
vel_filt = data_dict["vel_filt_list"]


def swingup(args, output_folder):
    print("Making data plots.")

    plt.figure()
    plt.plot(meas_time, meas_pos)
    plt.plot(des_time, des_pos)
    plt.xlabel("Time (s)")
    plt.ylabel("Position (rad)")
    plt.title("Position (rad) vs Time (s)")
    plt.legend(['position_measured', 'position_desired'])
    plt.draw()
    if args.save:
        plt.savefig(output_folder + '/swingup_pos.pdf')
    plt.show()

    plt.figure()
    plt.plot(meas_time, meas_vel)
    plt.plot(des_time, des_vel)
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (rad/s)")
    plt.legend(['velocity_measured', 'velocity_desired'])
    plt.title("Velocity (rad/s) vs Time (s)")
    plt.draw()
    if args.save:
        plt.savefig(output_folder + '/swingup_vel.pdf')
    plt.show()

    plt.figure()
    plt.plot(meas_time, meas_tau)
    plt.plot(des_time, des_tau)
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (Nm)")
    plt.title("Torque (Nm) vs Time (s)")
    plt.legend(['Measured Torque', 'Estimated Torque'])
    plt.draw()
    if args.save:
        plt.savefig(output_folder + '/swingup_tau.pdf')
    plt.show()


def grav_comp(args, output_folder):
    print("Making data plots.")

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

    time_vec_filtered = rm_filter(np.array(meas_time), 10)
    filtered_torque = rm_filter(np.array(meas_tau), 10)
    filtered_desired_torque = rm_filter(np.array(des_tau), 10)

    plt.figure()
    plt.plot(time_vec_filtered, filtered_torque)
    plt.plot(time_vec_filtered, filtered_desired_torque)
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (Nm)")
    plt.title(
        "Filtered Torque (Nm) vs Time (s) with moving average filter (window = 100)")
    plt.legend(['Measured', 'Desired'])
    plt.draw()
    if args.save:
        plt.savefig(output_folder + '/grav_comp_tau_filt.pdf')
    plt.show()

