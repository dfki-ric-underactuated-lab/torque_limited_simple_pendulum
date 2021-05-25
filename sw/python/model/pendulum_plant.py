import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.animation as mplanimation


class PendulumPlant:
    def __init__(self, mass=1.0, length=0.5, damping=0.1, gravity=9.81, coulomb_fric=0.0, inertia=None, torque_limit=np.inf):
        self.m = mass
        self.l = length
        self.b = damping
        self.g = gravity
        self.coulomb_fric = coulomb_fric
        if inertia is None:
            self.I = mass*length*length
        else:
            self.I = inertia

        self.torque_limit = torque_limit

        self.dof = 1
        self.x = np.zeros(2*self.dof)  # position, velocity
        self.t = 0.0  # time

    def set_state(self, time, x):
        self.x = x
        self.t = time

    def reset_data_recorder(self):
        self.t_values = []
        self.x_values = []
        self.tau_values = []

    def get_state(self):
        return self.t, self.x

    def forward_kinematics(self, pos):
        """
        forward kinematics, origin at fixed point
        """
        ee_pos_x = self.l * np.sin(pos)
        ee_pos_y = -self.l*np.cos(pos)
        return [ee_pos_x, ee_pos_y]

    def inverse_kinematics(self, ee_pos):
        """
        inverse kinematics, end-effector coordinates with origin at fixed point
        """
        pos = np.arctan2(ee_pos[1], ee_pos[0])
        return pos

    def forward_dynamics(self, pos, vel, tau):
        if vel > 0.0:
            fric_torque = self.coulomb_fric
        elif vel < 0.0:
            fric_torque = -self.coulomb_fric
        else:
            fric_torque = 0.0
        accn  = (tau - self.m * self.g * self.l * np.sin(pos) - self.b * vel - fric_torque) / self.I
        return accn

    def rhs(self, t, state, tau):
        pos = state[0]     # first element encodes the generalized position state
        vel = state[1]    # second element encodes the generalized velocity state

        # Forward dynamics
        accn = self.forward_dynamics(pos, vel, tau)

        # Next state
        res = np.zeros(2*self.dof)
        res[0] = state[1]
        res[1] = accn
        return res

    def euler_integrator(self, t, y, dt, tau):
        return self.rhs(t, y, tau)

    def runge_integrator(self, t, y, dt, tau):
        k1 = self.rhs(t, y, tau)
        k2 = self.rhs(t + 0.5 * dt, y + 0.5 * dt * k1, tau)
        k3 = self.rhs(t + 0.5 * dt, y + 0.5 * dt * k2, tau)
        k4 = self.rhs(t + dt, y + dt * k3, tau)
        return (k1 + 2 * (k2 + k3) + k4) / 6.0

    def step(self, tau, dt, integrator="runge_kutta"):
        tau = np.clip(tau, -self.torque_limit, self.torque_limit)
        if integrator == "runge_kutta":
            self.x += dt * self.runge_integrator(self.t, self.x, dt, tau)
        elif integrator == "euler":
            self.x += dt * self.euler_integrator(self.t, self.x, dt, tau)
        self.t += dt
        #self.x[0] = (self.x[0] + np.pi) % (np.pi*2) - np.pi
        # Store the time series output
        self.t_values.append(self.t)
        self.x_values.append(self.x.copy())
        self.tau_values.append(tau)

    def simulate(self, t0, y0, tf, dt, controller=None, integrator="runge_kutta"):
        self.set_state(t0, y0)

        self.t_values = []
        self.x_values = []
        self.tau_values = []

        while (self.t <= tf):
            if controller is not None:
                tau = controller.get_control_output(self.x)
            else:
                tau = 0
            self.step(tau, dt, integrator=integrator)

        return self.t_values, self.x_values, self.tau_values

    def _animation_init(self):
        """
        init of the animation plot
        """
        self.animation_ax.set_xlim(-1.5*self.l, 1.5*self.l)
        self.animation_ax.set_ylim(-1.5*self.l, 1.5*self.l)
        self.animation_ax.set_xlabel("x position [m]")
        self.animation_ax.set_ylabel("y position [m]")
        for ap in self.animation_plots:
            ap.set_data([], [])
        return self.animation_plots

    def _animation_step(self, par_dict):
        """
        simulation of a single step which also updates the animation plot
        """
        dt = par_dict["dt"]
        controller = par_dict["controller"]
        integrator = par_dict["integrator"]
        if controller is not None:
            tau = controller.get_control_output(self.x)
        else:
            tau = 0
        self.step(tau, dt, integrator=integrator)
        ee_pos = self.forward_kinematics(self.x[0])
        self.animation_plots[0].set_data(ee_pos[0], ee_pos[1])
        self.animation_plots[1].set_data([0, ee_pos[0]], [0, ee_pos[1]])
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
        self.ps_plots[0].set_data(np.asarray(self.x_values).T[0], np.asarray(self.x_values).T[1])
        return self.ps_plots

    def simulate_and_animate(self, t0, y0, tf, dt, controller=None, integrator="runge_kutta", phase_plot=False, save_video=False):
        """
        simulate and animate the pendulum
        """
        self.set_state(t0, y0)

        self.t_values = []
        self.x_values = []
        self.tau_values = []

        fig = plt.figure(figsize=(20,20))
        self.animation_ax = plt.axes()
        self.animation_plots = []
        ee_plot, = self.animation_ax.plot([], [], "ro", markersize=25.0, color="blue")
        bar_plot, = self.animation_ax.plot([], [], "-", lw=5, color="black")
        #text_plot = self.animation_ax.text(0.1, 0.1, [], xycoords="figure fraction")
        self.animation_plots.append(ee_plot)
        self.animation_plots.append(bar_plot)

        num_steps = int(tf / dt)
        par_dict = {}
        par_dict["dt"] = dt
        par_dict["controller"] = controller
        par_dict["integrator"] = integrator
        frames = num_steps*[par_dict]

        if phase_plot:
            ps_fig = plt.figure(figsize=(10,10))
            self.ps_ax = plt.axes()
            self.ps_plots = []
            ps_plot, = self.ps_ax.plot([], [], "-", lw=1.0, color="blue")
            self.ps_plots.append(ps_plot)

        animation = FuncAnimation(fig, self._animation_step, frames=frames, init_func=self._animation_init, blit=True, repeat=False, interval=dt*1000)
        if phase_plot:
            animation2 = FuncAnimation(ps_fig, self._ps_update, init_func=self._ps_init, blit=True, repeat=False, interval=dt*1000)

        if save_video:
            Writer = mplanimation.writers['ffmpeg']
            writer = Writer(fps=60, bitrate=1800)
            animation.save('pendulum_swingup.mp4', writer=writer)
            if phase_plot:
                Writer2 = mplanimation.writers['ffmpeg']
                writer2 = Writer2(fps=60, bitrate=1800)
                animation2.save('pendulum_swingup_phase.mp4', writer=writer2)
        plt.show()

        return self.t_values, self.x_values, self.tau_values
