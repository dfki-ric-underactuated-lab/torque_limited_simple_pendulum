import matplotlib.pyplot as plt

def swingup(OUTPUT_FOLDER, des_pos, des_vel, des_tau, des_time, meas_pos, meas_vel, meas_tau, meas_time):
    print("Making data plots.")

    plt.figure()
    plt.plot(meas_time, meas_pos)
    plt.plot(des_time, des_pos)
    plt.xlabel("Time (s)")
    plt.ylabel("Position (rad)")
    plt.title("Position (rad) vs Time (s)")
    plt.legend(['position_measured', 'position_desired'])
    plt.draw()
    plt.savefig(OUTPUT_FOLDER + '/swingup_pos.pdf')
    plt.show()

    plt.figure()
    plt.plot(meas_time, meas_vel)
    plt.plot(des_time, des_vel)
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (rad/s)")
    plt.legend(['velocity_measured', 'velocity_desired'])
    plt.title("Velocity (rad/s) vs Time (s)")
    plt.draw()
    plt.savefig(OUTPUT_FOLDER + '/swingup_vel.pdf')
    plt.show()

    plt.figure()
    plt.plot(meas_time, meas_tau)
    plt.plot(des_time, des_tau)
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (Nm)")
    plt.title("Torque (Nm) vs Time (s)")
    plt.legend(['Measured Torque', 'Estimated Torque'])
    plt.draw()
    plt.savefig(OUTPUT_FOLDER + '/swingup_tau.pdf'))
    plt.show()
    return