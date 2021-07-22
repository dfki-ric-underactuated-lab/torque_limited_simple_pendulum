import numpy as np
import matplotlib.pyplot as plt


def plot_policy(controller,
                position_range=[-np.pi, np.pi],
                velocity_range=[-8, 8],
                samples_per_dim=100,
                plotstyle="2d"):

    positions = np.linspace(position_range[0],
                            position_range[1],
                            samples_per_dim)

    velocities = np.linspace(velocity_range[0],
                             velocity_range[1],
                             samples_per_dim)

    torques = np.zeros((samples_per_dim, samples_per_dim))

    for p, pos in enumerate(positions):
        for v, vel in enumerate(velocities):
            _, _, u = controller.get_control_output(meas_pos=pos,
                                                    meas_vel=vel)
            if u is None:
                u = 0
            torques[p, v] = u

    if plotstyle == "3d":
        fig = plt.figure(figsize=(30, 30))
        ax = fig.add_subplot(111, projection='3d')

        x_data, y_data = np.meshgrid(np.arange(samples_per_dim),
                                     np.arange(samples_per_dim))

        ax.plot_surface(x_data,
                        y_data,
                        torques.T,
                        cmap="viridis",
                        edgecolor="none")

        tau_min = np.floor(np.min(torques))
        tau_max = np.ceil(np.max(torques))

        ax.set_xticks(np.linspace(0, samples_per_dim, 17))
        ax.set_yticks(np.linspace(-1, samples_per_dim, 17))
        ax.set_zticks(np.linspace(tau_min, tau_max, 23))

        x_ticks = np.linspace(position_range[0],
                              position_range[1],
                              17)
        x_ticks = np.around(x_ticks, 2)
        ax.set_xticklabels(x_ticks, fontsize=20)

        y_ticks = np.linspace(velocity_range[0],
                              velocity_range[1],
                              17)
        y_ticks = np.around(y_ticks, 1)
        ax.set_yticklabels(y_ticks, fontsize=20)

        z_ticks = np.linspace(tau_min,
                              tau_max,
                              23)
        z_ticks = np.around(z_ticks, 1)
        ax.set_zticklabels(z_ticks, fontsize=20)

        ax.set_xlabel("\nangle [rad]", fontsize=30, linespacing=4)
        ax.set_ylabel("\nangular velocity [rad/s]", fontsize=30, linespacing=4)
        ax.set_zlabel("\ntorque [Nm]", fontsize=30, linespacing=4)
        plt.show()

    elif plotstyle == "2d":
        fig = plt.figure(figsize=(30, 30))
        ax = fig.add_subplot(111)

        ax.tick_params(axis="both", which="major", labelsize=20)
        ax.tick_params(axis="both", which="minor", labelsize=20)

        img = ax.imshow(torques.T, extent=[-1, 1, -1, 1])

        ax.set_xticks(np.linspace(-1, 1, 17))
        ax.set_yticks(np.linspace(-1, 1, 17))

        x_ticks = np.linspace(position_range[0],
                              position_range[1],
                              17)
        x_ticks = np.around(x_ticks, 2)
        ax.set_xticklabels(x_ticks)

        y_ticks = np.linspace(velocity_range[0],
                              velocity_range[1],
                              17)
        y_ticks = np.around(y_ticks, 1)
        ax.set_yticklabels(y_ticks)

        cbar = fig.colorbar(img)
        cbar.ax.tick_params(labelsize=20)
        cbar.set_label("torque [Nm]", rotation=270, fontsize=30)
        plt.xlabel("angle [rad]", fontsize=30)
        plt.ylabel("angular velocity [rad/s]", fontsize=30)

        plt.show()

    else:
        print("Plotstyle ", plotstyle, " is not implemented",
              "Please set plotytyle to 2d or 3d")
