import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os

c_blue = (0.11, 0.23, 0.56)
c_orange = (0.96, 0.65, 0.07)
c_red = (0.92, 0.38, 0.62)
c_green = (0.41, 0.75, 0.64)
c_purple = (0.52, 0.40, 0.83)
c_yellow = (0.99, 0.88, 0.35)
c_gray = (0.39, 0.39, 0.39)
c_light_blue = (0.68, 0.85, 0.90)
c_soft_gray = (0.85, 0.85, 0.85)
c_black = (0.023, 0.090, 0.109)
c_orange_edge = (0.39, 0.39, 0.0)


def plotStatesAndTransitions(
    s,
    t,
    path,
    velocity_spread=1.0,
    save_string="capability_map",
    overlay=True,
    highlight_nodes=[],
    highlight_transitions=[],
):
    # velocity_spread = 0.5
    fig, ax = plt.subplots(figsize=(4.65, 4.65))
    radius = 10
    # cm = plt.cm.get_cmap('RdYlBu')

    highlight_nodes = np.asarray(highlight_nodes)
    highlight_transitions = np.asarray(highlight_transitions)

    theta = np.linspace(-np.pi, np.pi, 100)
    ax.plot(
        np.sin(theta) * radius, -np.cos(theta) * radius, "--", color=c_gray, alpha=0.5
    )
    ax.text(-0.1, 0, "x", verticalalignment="center", color="black")

    ax.plot(
        np.sin(s.T[0]) * (velocity_spread * s.T[1] + radius),
        -np.cos(s.T[0]) * (velocity_spread * s.T[1] + radius),
        ".",
        markersize=4,
        color=c_blue,
    )
    ax.plot(
        np.sin(s.T[0][0]) * (velocity_spread * s.T[1][0] + radius),
        -np.cos(s.T[0][0]) * (velocity_spread * s.T[1][0] + radius),
        "o",
        color=c_green,
    )
    if len(highlight_nodes) > 0:
        ax.plot(
            np.sin(highlight_nodes.T[0])
            * (velocity_spread * highlight_nodes.T[1] + radius),
            -np.cos(highlight_nodes.T[0])
            * (velocity_spread * highlight_nodes.T[1] + radius),
            "o",
            color=c_green,
        )

    # for i in range(0, len(s.T[0])):
    for i, trans in enumerate(t):
        # if i < t.shape[0]:
        #    for a in t[i]:
        start_pos = trans[0]
        start_vel = trans[1]
        end_pos = trans[2]
        end_vel = trans[3]
        # deltaT = trans[4]

        d = np.asarray([start_pos, end_pos])
        x = np.asarray(
            [
                np.sin(start_pos) * (velocity_spread * start_vel + radius),
                np.sin(end_pos) * (velocity_spread * end_vel + radius),
            ]
        )
        y = np.asarray(
            [
                -np.cos(start_pos) * (velocity_spread * start_vel + radius),
                -np.cos(end_pos) * (velocity_spread * end_vel + radius),
            ]
        )
        dx = 0.75 * (x[1] - x[0])
        dy = 0.75 * (y[1] - y[0])
        color = c_red
        hw = 0.1
        lw = 1.0

        # if ((start_pos, start_vel), (end_pos, end_vel)) in highlight_transitions:
        # if [trans[:4]] in np.asarray(highlight_transitions):
        for ht in highlight_transitions:
            if (
                ht[0] == trans[0]
                and ht[1] == trans[1]
                and ht[2] == trans[2]
                and ht[3] == trans[3]
            ):
                color = c_green
                hw = 0.1
                lw = 2.0
        ax.arrow(x[0], y[0], dx, dy, lw=lw, color=color, head_width=hw)
        if i % 1000 == 0:
            print(i)

    if overlay:
        lin = np.linspace(0, 1, 1000)
        max_vel_range = s.T[1].max() * velocity_spread
        vel_legend_pos = np.pi * 1.2

        ax.annotate(
            s="",
            xy=(
                (radius + 1.2 * max_vel_range) * np.sin(vel_legend_pos),
                (radius + 1.2 * max_vel_range) * (-1) * np.cos(vel_legend_pos),
            ),
            xytext=(
                (radius - 1.2 * max_vel_range) * np.sin(vel_legend_pos),
                (radius - 1.2 * max_vel_range) * (-1) * np.cos(vel_legend_pos),
            ),
            arrowprops=dict(arrowstyle="<->"),
        )
        ax.text(
            (radius + 1.2 * max_vel_range) * np.sin(vel_legend_pos) + 0.3,
            (radius + 1.2 * max_vel_range) * (-1) * np.cos(vel_legend_pos) - 0.1,
            r"$\dot{q}^{max}$",
            size=20,
        )
        ax.text(
            (radius - 1.2 * max_vel_range) * np.sin(vel_legend_pos) + 0.3,
            (radius - 1.2 * max_vel_range) * (-1) * np.cos(vel_legend_pos) - 0.1,
            r"$-\dot{q}^{max}$",
            size=20,
        )

        # plot pendulum
        pendulum_pos = 0
        ax.annotate(
            s="",
            xy=(0, 0),
            xytext=(radius * np.sin(pendulum_pos), -radius * np.cos(pendulum_pos)),
            arrowprops=dict(headlength=0.4, headwidth=0.2, width=5, color=c_green),
        )
        ax.plot(
            np.sin(pendulum_pos) * (radius),
            -np.cos(pendulum_pos) * (radius),
            "o",
            markersize=25,
            color=c_green,
        )

        pendulum_pos = np.pi / 45
        ax.annotate(
            s="",
            xy=(0, 0),
            xytext=(radius * np.sin(pendulum_pos), -radius * np.cos(pendulum_pos)),
            arrowprops=dict(
                headlength=0.4, headwidth=0.2, width=5, color=c_green, alpha=0.4
            ),
        )
        ax.plot(
            np.sin(pendulum_pos) * (radius),
            -np.cos(pendulum_pos) * (radius),
            "o",
            markersize=25,
            color=c_green,
            alpha=0.4,
        )

        pendulum_pos = np.pi / 22.5
        ax.annotate(
            s="",
            xy=(0, 0),
            xytext=(radius * np.sin(pendulum_pos), -radius * np.cos(pendulum_pos)),
            arrowprops=dict(
                headlength=0.4, headwidth=0.2, width=5, color=c_green, alpha=0.2
            ),
        )
        ax.plot(
            np.sin(pendulum_pos) * (radius),
            -np.cos(pendulum_pos) * (radius),
            "o",
            markersize=25,
            color=c_green,
            alpha=0.2,
        )

        if velocity_spread > 0:
            # velocity axis
            ax.text(
                radius * np.sin(vel_legend_pos) + 0.3,
                -radius * np.cos(vel_legend_pos) - 0.3,
                r"$\dot{q}$",
                size=20,
            )
            # if velocity_spread == 1.0:
            # velocity direction arrows
            style = "Simple,tail_width=0.5,head_width=4,head_length=8"
            kw = dict(arrowstyle=style, color="black")

            arrow1_pos = np.pi * 0.75
            arrow1_end = np.pi * 0.85

            r_arrow = 1

            vel_arrow_left = patches.FancyArrowPatch(
                (
                    (radius + r_arrow * max_vel_range) * np.sin(arrow1_pos),
                    (radius + r_arrow * max_vel_range) * (-1) * np.cos(arrow1_pos),
                ),
                (
                    (radius + r_arrow * max_vel_range) * np.sin(arrow1_end),
                    (radius + r_arrow * max_vel_range) * (-1) * np.cos(arrow1_end),
                ),
                connectionstyle="arc3,rad=0.1",
                **kw
            )
            r_arrow = -1
            vel_arrow_right = patches.FancyArrowPatch(
                (
                    (radius + r_arrow * max_vel_range) * np.sin(arrow1_end),
                    (radius + r_arrow * max_vel_range) * (-1) * np.cos(arrow1_end),
                ),
                (
                    (radius + r_arrow * max_vel_range) * np.sin(arrow1_pos),
                    (radius + r_arrow * max_vel_range) * (-1) * np.cos(arrow1_pos),
                ),
                connectionstyle="arc3,rad=-0.1",
                **kw
            )

            arrow1_pos = np.pi * 0.75
            arrow1_end = np.pi * 0.80

            r_arrow = -2 / 3
            vel_arrow_right2 = patches.FancyArrowPatch(
                (
                    (radius + r_arrow * max_vel_range) * np.sin(arrow1_end),
                    (radius + r_arrow * max_vel_range) * (-1) * np.cos(arrow1_end),
                ),
                (
                    (radius + r_arrow * max_vel_range) * np.sin(arrow1_pos),
                    (radius + r_arrow * max_vel_range) * (-1) * np.cos(arrow1_pos),
                ),
                connectionstyle="arc3,rad=-0.1",
                **kw
            )

            r_arrow = 2 / 3
            vel_arrow_left2 = patches.FancyArrowPatch(
                (
                    (radius + r_arrow * max_vel_range) * np.sin(arrow1_pos),
                    (radius + r_arrow * max_vel_range) * (-1) * np.cos(arrow1_pos),
                ),
                (
                    (radius + r_arrow * max_vel_range) * np.sin(arrow1_end),
                    (radius + r_arrow * max_vel_range) * (-1) * np.cos(arrow1_end),
                ),
                connectionstyle="arc3,rad=0.1",
                **kw
            )

            arrow1_pos = np.pi * 0.75
            arrow1_end = np.pi * 0.775

            r_arrow = -1 / 3
            vel_arrow_right3 = patches.FancyArrowPatch(
                (
                    (radius + r_arrow * max_vel_range) * np.sin(arrow1_end),
                    (radius + r_arrow * max_vel_range) * (-1) * np.cos(arrow1_end),
                ),
                (
                    (radius + r_arrow * max_vel_range) * np.sin(arrow1_pos),
                    (radius + r_arrow * max_vel_range) * (-1) * np.cos(arrow1_pos),
                ),
                connectionstyle="arc3,rad=-0.1",
                **kw
            )

            r_arrow = 1 / 3
            vel_arrow_left3 = patches.FancyArrowPatch(
                (
                    (radius + r_arrow * max_vel_range) * np.sin(arrow1_pos),
                    (radius + r_arrow * max_vel_range) * (-1) * np.cos(arrow1_pos),
                ),
                (
                    (radius + r_arrow * max_vel_range) * np.sin(arrow1_end),
                    (radius + r_arrow * max_vel_range) * (-1) * np.cos(arrow1_end),
                ),
                connectionstyle="arc3,rad=0.1",
                **kw
            )

            plt.gca().add_patch(vel_arrow_left)
            plt.gca().add_patch(vel_arrow_right)
            plt.gca().add_patch(vel_arrow_left2)
            plt.gca().add_patch(vel_arrow_right2)
            plt.gca().add_patch(vel_arrow_left3)
            plt.gca().add_patch(vel_arrow_right3)

    # plt.colorbar(p)
    # plt.xlim(-1.3*radius, 1.3*radius)
    # plt.ylim(-1.3*radius, 1.3*radius)
    # plt.axes().set_aspect('equal', 'datalim')
    # plt.axes().set_aspect('equal')
    ax.axis("off")
    # plt.savefig(os.path.join(path, "Capability_maps/capability_map_" + str(int(10*round(velocity_spread, 2))).zfill(1)+".png"), bbox_inches='tight', dpi=600)
    # plt.savefig(os.path.join(path, "Capability_maps/capability_map_" + str(number).zfill(4)+".svg"), bbox_inches='tight', dpi=600)
    fig.savefig(os.path.join(path, save_string), bbox_inches="tight", dpi=600)
    # plt.savefig(os.path.join(path, "Capability_maps/capability_map_" + str(int(10*round(velocity_spread, 2))).zfill(1)+".eps"), bbox_inches='tight', format='eps')
    # plt.show()
    # plt.close()


def plotStatesAndTransitionsPhaseSpace(
    s,
    t,
    path,
    save_string="capability_phase_space",
    highlight_nodes=[],
    highlight_transitions=[],
    free_only=False,
):
    # velocity_spread = 0.5
    plt.figure(figsize=(20, 20))
    radius = 10
    # cm = plt.cm.get_cmap('RdYlBu')

    highlight_nodes = np.asarray(highlight_nodes)
    highlight_transitions = np.asarray(highlight_transitions)

    plt.plot(s.T[0], s.T[1], ".", markersize=1)
    plt.plot(s.T[0][0], s.T[1][0], "o", color="green", markersize=10)

    if len(highlight_nodes) > 0:
        plt.plot(
            highlight_nodes.T[0],
            highlight_nodes.T[1],
            "o",
            color="green",
            markersize=10,
        )

    for i, trans in enumerate(t):
        start_pos = trans[0]
        start_vel = trans[1]
        end_pos = trans[2]
        end_vel = trans[3]
        # deltaT = trans[4]

        dx = np.abs(start_vel) * (end_pos - start_pos)
        dy = np.abs(start_vel) * (end_vel - start_vel)

        color = "red"
        hw = 0.02
        lw = 0.2

        if trans in highlight_transitions:
            color = "green"
            hw = 0.1
            lw = 2.0

        plot = True
        if free_only:
            if trans[5] != 0:
                plot = False
        if plot:
            plt.arrow(start_pos, start_vel, dx, dy, lw=lw, color=color, head_width=hw)
        if i % 1000 == 0:
            print(i)

    # plt.xlim(-np.pi, np.pi)
    # plt.ylim(-np.pi, np.pi)
    # plt.axes().set_aspect('equal', 'datalim')
    # plt.axis('off')
    plt.xlabel("position [rad]")
    plt.ylabel("velocity [rad/s]")
    plt.savefig(os.path.join(path, save_string), bbox_inches="tight", dpi=600)
    # plt.show()
    plt.close()
