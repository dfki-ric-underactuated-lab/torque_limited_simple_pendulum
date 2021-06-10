import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.animation as mplanimation
import time


class Simulator:
    def __init__(self, plant):
        """
        Simulator class, can simulate and animate the pendulum
        Initialization parameters:
            - plant: plant object
                     (e.g. PendulumPlant from models.pendulum_plant.py)
        """
        self.plant = plant

        self.x = np.zeros(2*self.plant.dof)  # position, velocity
        self.t = 0.0  # time

    def set_state(self, time, x):
        """
        set the state of the pendulum plant
        input:
            - time: float, time, unit: s
            - x: type as self.plant expects a state,
                 state of the pendulum plant
        """
        self.x = x
        self.t = time

    def get_state(self):
        """
        Get current state of the plant
        returns:
            - time,     float, unit: s
            - plant state,    type as self.plant expects a state
        """
        return self.t, self.x

    def reset_data_recorder(self):
        """
        Reset the internal data recorder of the simulator
        """
        self.t_values = []
        self.x_values = []
        self.tau_values = []

    def record_data(self, time, x, tau):
        """
        Records data in the internal data recorder
        input:
            - time: float, unit: s
            - x:    type as self.plant expects a state
            - tau:  type as self.plant expects an actuation
        """
        self.t_values.append(time)
        self.x_values.append(x)
        self.tau_values.append(tau)

    def euler_integrator(self, t, y, dt, tau):
        """
        Euler integrator for the simulated plant
        input:
            - t: float, time, unit: s
            - y: type as self.plant expects a state
            - dt: float, time step, unit: s
            - tau:  type as self.plant expects an actuation
        returns:
            - the Euler integrand
        """
        return self.plant.rhs(t, y, tau)

    def runge_integrator(self, t, y, dt, tau):
        """
        Runge-Kutta integrator for the simulated plant
        input:
            - t: float, time, unit: s
            - y: type as self.plant expects a state
            - dt: float, time step, unit: s
            - tau:  type as self.plant expects an actuation
        returns:
            - the Runge-Kutta integrand
        """
        k1 = self.plant.rhs(t, y, tau)
        k2 = self.plant.rhs(t + 0.5 * dt, y + 0.5 * dt * k1, tau)
        k3 = self.plant.rhs(t + 0.5 * dt, y + 0.5 * dt * k2, tau)
        k4 = self.plant.rhs(t + dt, y + dt * k3, tau)
        return (k1 + 2 * (k2 + k3) + k4) / 6.0

    def step(self, tau, dt, integrator="runge_kutta"):
        """
        Performs a single step of the plant.
        input:
            - tau:  type as self.plant expects an actuation
            - dt:   float, time step, unit: s
            - integrator: string, "euler" for euler integrator,
                                  "runge_kutta" for Runge-Kutta integrator
        """
        if integrator == "runge_kutta":
            self.x += dt * self.runge_integrator(self.t, self.x, dt, tau)
        elif integrator == "euler":
            self.x += dt * self.euler_integrator(self.t, self.x, dt, tau)
        else:
            raise NotImplementedError(
                   f'Sorry, the integrator {integrator} is not implemented.')
        self.t += dt
        self.record_data(self.t, self.x.copy(), tau)

    def simulate(self, t0, x0, tf, dt, controller=None,
                 integrator="runge_kutta"):
        """
        Simulates the plant over a period of time.
        input:
            - t0: float, start time, unit s
            - x0: type as self.plant expects a state,
                  start state
            - tf: float, final time, unit: s
            - controller: A controller object of the type of the
                          AbstractController in utilities.abstract_controller.py
                          If None, a free pendulum is simulated.
            - integrator: string, "euler" for euler integrator,
                                  "runge_kutta" for Runge-Kutta integrator
        returns:
            - a list of time values
            - a list of states
            - a list of torques
        """
        self.set_state(t0, x0, 0)
        self.reset_data_recorder()

        while (self.t <= tf):
            if controller is not None:
                _, _, tau = controller.get_control_output(
                                        meas_pos=self.x[:self.plant.dof],
                                        meas_vel=self.x[self.plant.dof:],
                                        meas_tau=np.zeros(self.plant.dof),
                                        meas_time=self.t)
            else:
                tau = np.zeros(self.plant.n_actuators)
            self.step(tau, dt, integrator=integrator)

        return self.t_values, self.x_values, self.tau_values

    def _animation_init(self):
        """
        init of the animation plot
        """
        self.animation_ax.set_xlim(self.plant.workspace_range[0][0],
                                   self.plant.workspace_range[0][1])
        self.animation_ax.set_ylim(self.plant.workspace_range[1][0],
                                   self.plant.workspace_range[1][1])
        self.animation_ax.set_xlabel("x position [m]")
        self.animation_ax.set_ylabel("y position [m]")
        for ap in self.animation_plots[:-1]:
            ap.set_data([], [])
        self.animation_plots[-1].set_text("t = 0.000")
        return self.animation_plots

    def _animation_step(self, par_dict):
        """
        simulation of a single step which also updates the animation plot
        """
        t0 = time.time()
        dt = par_dict["dt"]
        controller = par_dict["controller"]
        integrator = par_dict["integrator"]
        if controller is not None:
            _, _, tau = controller.get_control_output(
                                    meas_pos=self.x[:self.plant.dof],
                                    meas_vel=self.x[self.plant.dof:],
                                    meas_tau=np.zeros(self.plant.dof),
                                    meas_time=self.t)
        else:
            tau = np.zeros(self.plant.n_actuators)
        self.step(tau, dt, integrator=integrator)
        ee_pos = self.plant.forward_kinematics(self.x[:self.plant.dof])
        ee_pos.insert(0, self.plant.base)
        ani_plot_counter = 0
        for link in range(self.plant.n_links):
            self.animation_plots[ani_plot_counter].set_data(ee_pos[link+1][0],
                                                            ee_pos[link+1][1])
            ani_plot_counter += 1
            self.animation_plots[ani_plot_counter].set_data(
                            [ee_pos[link][0], ee_pos[link+1][0]],
                            [ee_pos[link][1], ee_pos[link+1][1]])
            ani_plot_counter += 1
        t = float(self.animation_plots[ani_plot_counter].get_text()[4:])
        t = round(t+dt, 3)
        self.animation_plots[ani_plot_counter].set_text(f"t = {t}")

        # if the animation runs slower than real time
        # the time display will be red
        if time.time() - t0 > dt:
            self.animation_plots[ani_plot_counter].set_color("red")
        else:
            self.animation_plots[ani_plot_counter].set_color("black")
        return self.animation_plots

    def _ps_init(self):
        """
        init of the phase space animation plot
        """
        self.ps_ax.set_xlim(-np.pi, np.pi)
        self.ps_ax.set_ylim(-10, 10)
        self.ps_ax.set_xlabel("degree [rad]")
        self.ps_ax.set_ylabel("velocity [rad/s]")
        for ap in self.ps_plots:
            ap.set_data([], [])
        return self.ps_plots

    def _ps_update(self, i):
        """
        update of the phase space animation plot
        """
        for d in range(self.plant.dof):
            self.ps_plots[d].set_data(
                            np.asarray(self.x_values).T[d],
                            np.asarray(self.x_values).T[self.plant.dof+d])
        return self.ps_plots

    def simulate_and_animate(self, t0, x0, tf, dt, controller=None,
                             integrator="runge_kutta", phase_plot=False,
                             save_video=False, video_name="video"):

        """
        Simulation and animation of the plant motion.
        The animation is only implemented for 2d serial chains.
        input:
        Simulates the plant over a period of time.
        input:
            - t0: float, start time, unit s
            - x0: type as self.plant expects a state,
                  start state
            - tf: float, final time, unit: s
            - controller: A controller object of the type of the
                          AbstractController in utilities.abstract_controller.py
                          If None, a free pendulum is simulated.
            - integrator: string, "euler" for euler integrator,
                                  "runge_kutta" for Runge-Kutta integrator
            - phase_plot: bool, whether to show a plot of the phase space
                          together with the animation
            - save_video: bool, whether to save the animation as mp4 video
            - video_name: string, if save_video, the name of the file
                          where the video will be stored
        returns:
            - a list of time values
            - a list of states
            - a list of torques
        """

        self.set_state(t0, x0)
        self.reset_data_recorder()

        fig = plt.figure(figsize=(20, 20))
        self.animation_ax = plt.axes()
        self.animation_plots = []

        for link in range(self.plant.n_links):
            ee_plot, = self.animation_ax.plot([], [], "ro",
                                              markersize=25.0, color="blue")
            self.animation_plots.append(ee_plot)
            bar_plot, = self.animation_ax.plot([], [], "-",
                                               lw=5, color="black")
            self.animation_plots.append(bar_plot)

        text_plot = self.animation_ax.text(0.15, 0.85, [],
                                           fontsize=40,
                                           transform=fig.transFigure)

        self.animation_plots.append(text_plot)

        num_steps = int(tf / dt)
        par_dict = {}
        par_dict["dt"] = dt
        par_dict["controller"] = controller
        par_dict["integrator"] = integrator
        frames = num_steps*[par_dict]

        animation = FuncAnimation(fig, self._animation_step, frames=frames,
                                  init_func=self._animation_init, blit=True,
                                  repeat=False, interval=dt*1000)

        if phase_plot:
            ps_fig = plt.figure(figsize=(10, 10))
            self.ps_ax = plt.axes()
            self.ps_plots = []
            for d in range(self.plant.dof):
                ps_plot, = self.ps_ax.plot([], [], "-", lw=1.0, color="blue")
                self.ps_plots.append(ps_plot)

            animation2 = FuncAnimation(ps_fig, self._ps_update,
                                       init_func=self._ps_init, blit=True,
                                       repeat=False, interval=dt*1000)

        if save_video:
            print(f"Saving video to {video_name}.mp4")
            Writer = mplanimation.writers['ffmpeg']
            writer = Writer(fps=60, bitrate=1800)
            animation.save(video_name+'.mp4', writer=writer)
            print("Saving video done.")
        self.set_state(t0, x0)
        self.reset_data_recorder()
        plt.show()

        return self.t_values, self.x_values, self.tau_values
