import matplotlib.pyplot as plt


def swingup(output_folder, data_dict):
    print("Making data plots.")

    des_time = data_dict["des_time_list"]
    des_pos = data_dict["des_pos_list"]
    des_vel = data_dict["des_vel_list"]
    des_tau = data_dict["des_tau_list"]
    meas_time = data_dict["meas_time_list"]
    meas_pos = data_dict["meas_pos_list"]
    meas_vel = data_dict["meas_vel_list"]
    meas_tau = data_dict["meas_tau_list"]

    plt.figure()
    plt.plot(meas_time, meas_pos)
    plt.plot(des_time, des_pos)
    plt.xlabel("Time (s)")
    plt.ylabel("Position (rad)")
    plt.title("Position (rad) vs Time (s)")
    plt.legend(['position_measured', 'position_desired'])
    plt.draw()
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
    plt.savefig(output_folder + '/swingup_tau.pdf')
    plt.show()

