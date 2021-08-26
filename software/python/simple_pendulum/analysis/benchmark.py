import numpy as np
import time

from simple_pendulum.model.pendulum_plant import PendulumPlant
from simple_pendulum.simulation.simulation import Simulator


def modify_pendulum_parameter(par):
    mod_par = np.copy(par)
    mod_par += np.random.rand()*0.1
    mod_par *= np.random.rand()*1.0 + 0.5
    return mod_par


class benchmarker():
    def __init__(self,
                 dt=0.01,
                 max_time=10.0,
                 integrator="runge_kutta",
                 benchmark_iterations=10):

        self.dt = dt
        self.max_time = max_time
        self.max_steps = int(self.max_time / self.dt)
        self.integrator = "runge_kutta"
        self.iterations = benchmark_iterations

        self.x0 = np.array([0.0, 0.0])
        self.goal = np.array([np.pi, 0.0])
        self.epsilon = np.array([0.1, 0.1])

    def init_pendulum(self, mass=0.57288, length=0.5, inertia=None,
                      damping=0.15, coulomb_friction=0.0, gravity=9.81,
                      torque_limit=2.0):
        """
        Initialize the pendulum parameters.

        Parameters
        ----------
        mass : float, default=0.57288
            mass of the pendulum [kg]
        length : float, default=0.5
            length of the pendulum [m]
        inertia : float, default=None
            inertia of the pendulum [kg m^2]
            defaults to point mass inertia (mass*length^2)
        damping : float, default=0.15
            damping factor of the pendulum [kg m/s]
        coulomb_friction : float, default=0.0
            coulomb friciton of the pendulum [Nm]
        gravity : float, default=9.81
            gravity (positive direction points down) [m/s^2]
        torque_limit : float, default=2.0
            the torque_limit of the pendulum actuator
        """

        self.pen_mass = mass
        self.pen_length = length
        if inertia is None:
            inertia = mass*length**2
        self.pen_inertia = inertia
        self.pen_damping = damping
        self.pen_cfric = coulomb_friction
        self.pen_gravity = gravity
        self.pen_torque_limit = torque_limit

        self.pendulum = PendulumPlant(mass=self.pen_mass,
                                      length=self.pen_length,
                                      damping=self.pen_damping,
                                      gravity=self.pen_gravity,
                                      coulomb_fric=self.pen_cfric,
                                      inertia=self.pen_inertia,
                                      torque_limit=self.pen_torque_limit)

        self.ref_pendulum = PendulumPlant(mass=self.pen_mass,
                                          length=self.pen_length,
                                          damping=self.pen_damping,
                                          gravity=self.pen_gravity,
                                          coulomb_fric=self.pen_cfric,
                                          inertia=self.pen_inertia,
                                          torque_limit=self.pen_torque_limit)

        self.simulator = Simulator(plant=self.pendulum)
        self.ref_simulator = Simulator(plant=self.ref_pendulum)

    def set_controller(self, controller):
        self.controller = controller

    def check_regular_execution(self):

        t0 = 0.0
        self.controller.set_goal(x=self.goal)
        self.controller.init(x0=self.x0)

        x = np.copy(self.x0)
        t = np.copy(t0)
        self.simulator.reset_data_recorder()
        self.ref_simulator.reset_data_recorder()
        self.simulator.set_state(time=t, x=np.copy(x))

        energy_consumed = 0.0
        swingup_time = 0.0
        swingup_success = False

        for i in range(self.max_steps):
            _, _, tau = self.controller.get_control_output(meas_pos=x[0],
                                                           meas_vel=x[1],
                                                           meas_tau=0,
                                                           meas_time=t)

            self.ref_simulator.set_state(time=t, x=np.copy(x))

            self.simulator.step(tau, self.dt, integrator=self.integrator)
            self.ref_simulator.step(0.0, self.dt, integrator=self.integrator)

            t, x = self.simulator.get_state()
            t_ref, x_ref = self.ref_simulator.get_state()

            energy = np.abs(self.pendulum.total_energy(x) -
                            self.pendulum.total_energy(x_ref))
            energy_consumed += energy

            diff = x - self.goal
            diff[0] = (diff[0] + np.pi) % (2*np.pi) - np.pi
            if np.abs(diff[0]) < self.epsilon[0]:
                if np.abs(diff[1]) < self.epsilon[1]:
                    swingup_success = True
            if not swingup_success:
                swingup_time += self.dt

        if not swingup_success:
            swingup_time = np.inf

        return swingup_success, swingup_time, energy_consumed

    def check_consistency(self):
        # different starting positions
        t0 = 0.0
        x0 = np.zeros(2)
        x0[0] = np.random.rand()*2*np.pi - np.pi  # in range [-pi, pi]
        x0[1] = np.random.rand()*4*np.pi - 2*np.pi  # in range [-2pi, 2pi]
        self.controller.set_goal(x=self.goal)
        self.controller.init(x0=x0)

        x = np.copy(x0)
        t = np.copy(t0)
        self.simulator.reset_data_recorder()
        self.simulator.set_state(time=t, x=np.copy(x))

        swingup_success = False

        while not swingup_success:
            _, _, tau = self.controller.get_control_output(meas_pos=x[0],
                                                           meas_vel=x[1],
                                                           meas_tau=0,
                                                           meas_time=t)

            self.simulator.step(tau, self.dt, integrator=self.integrator)
            t, x = self.simulator.get_state()

            diff = x - self.goal
            diff[0] = (diff[0] + np.pi) % (2*np.pi) - np.pi
            if np.abs(diff[0]) < self.epsilon[0]:
                if np.abs(diff[1]) < self.epsilon[1]:
                    swingup_success = True
            if t >= self.max_time:
                break

        return swingup_success

    def check_stability(self):
        # random perturbations during executions
        t0 = 0.0
        self.controller.set_goal(x=self.goal)
        self.controller.init(x0=self.x0)

        x = np.copy(self.x0)
        t = np.copy(t0)
        self.simulator.reset_data_recorder()
        self.simulator.set_state(time=t, x=np.copy(x))

        perturbation_times = np.linspace(0, self.max_time, 11)
        p_counter = 0
        swingup_success = False

        while not swingup_success:
            _, _, tau = self.controller.get_control_output(meas_pos=x[0],
                                                           meas_vel=x[1],
                                                           meas_tau=0,
                                                           meas_time=t)

            if t > perturbation_times[0]:
                tau += np.random.rand()*2.0 - 1.0
                p_counter += 1
                if p_counter >= 5:
                    perturbation_times = np.delete(perturbation_times, 0)
                    p_counter = 0

            self.simulator.step(tau, self.dt, integrator=self.integrator)
            t, x = self.simulator.get_state()

            diff = x - self.goal
            diff[0] = (diff[0] + np.pi) % (2*np.pi) - np.pi
            if np.abs(diff[0]) < self.epsilon[0]:
                if np.abs(diff[1]) < self.epsilon[1]:
                    swingup_success = True
            if t >= self.max_time:
                break

        return swingup_success

    def check_sensitivity(self):
        # sensitivity to model parameters

        mass = modify_pendulum_parameter(self.pen_mass)
        length = modify_pendulum_parameter(self.pen_length)
        damping = modify_pendulum_parameter(self.pen_damping)
        cfric = modify_pendulum_parameter(self.pen_cfric)
        inertia = modify_pendulum_parameter(self.pen_inertia)

        modified_pendulum = PendulumPlant(mass=mass,
                                          length=length,
                                          damping=damping,
                                          gravity=self.pen_gravity,
                                          coulomb_fric=cfric,
                                          inertia=inertia,
                                          torque_limit=self.pen_torque_limit)

        modified_simulator = Simulator(plant=modified_pendulum)

        t0 = 0.0
        self.controller.set_goal(x=self.goal)
        self.controller.init(x0=self.x0)

        x = np.copy(self.x0)
        t = np.copy(t0)
        modified_simulator.set_state(time=t, x=np.copy(x))

        swingup_success = False

        while not swingup_success:
            _, _, tau = self.controller.get_control_output(meas_pos=x[0],
                                                           meas_vel=x[1],
                                                           meas_tau=0,
                                                           meas_time=t)

            modified_simulator.step(tau, self.dt, integrator=self.integrator)
            t, x = modified_simulator.get_state()

            diff = x - self.goal
            diff[0] = (diff[0] + np.pi) % (2*np.pi) - np.pi
            if np.abs(diff[0]) < self.epsilon[0]:
                if np.abs(diff[1]) < self.epsilon[1]:
                    swingup_success = True
            if t >= self.max_time:
                break

        return swingup_success

    def check_reduced_torque_limit(self, tl=np.inf):
        t0 = 0.0
        self.controller.set_goal(x=self.goal)
        self.controller.init(x0=self.x0)

        x = np.copy(self.x0)
        t = np.copy(t0)
        self.simulator.reset_data_recorder()
        self.simulator.set_state(time=t, x=np.copy(x))

        swingup_success = False

        while not swingup_success:
            _, _, tau = self.controller.get_control_output(meas_pos=x[0],
                                                           meas_vel=x[1],
                                                           meas_tau=0,
                                                           meas_time=t)

            tau = np.clip(tau, -tl, tl)

            self.simulator.step(tau, self.dt, integrator=self.integrator)
            t, x = self.simulator.get_state()

            diff = x - self.goal
            diff[0] = (diff[0] + np.pi) % (2*np.pi) - np.pi
            if np.abs(diff[0]) < self.epsilon[0]:
                if np.abs(diff[1]) < self.epsilon[1]:
                    swingup_success = True
            if t >= 10*self.max_time:
                break

        return swingup_success

    def check_speed(self, N=1000):

        self.controller.set_goal(x=self.goal)
        self.controller.init(x0=self.x0)
        pos = np.random.rand(N)*2*np.pi - np.pi  # in range [-pi, pi]
        vel = np.random.rand(N)*4*np.pi - 2*np.pi
        start = time.time()
        for i in range(N):
            _, _, tau = self.controller.get_control_output(meas_pos=pos[i],
                                                           meas_vel=vel[i],
                                                           meas_tau=0,
                                                           meas_time=0)
        elapsed_time = time.time() - start
        min_dt = elapsed_time / N
        return min_dt

    def benchmark(self,
                  check_speed=True,
                  check_energy=True,
                  check_time=True,
                  check_consistency=True,
                  check_stability=True,
                  check_sensitivity=True,
                  check_torque_limit=True):

        if check_speed:
            N = 1000
            min_dt = self.check_speed(N)
            print("\n*********************************")
            print("Controller Speed")
            print("Minimal dt: ", min_dt, "s (", 1./min_dt, " Hz)")
            print("average over", N, " function calls")
            print("*********************************\n")

        if check_energy or check_time:
            successes = []
            times = []
            energies = []
            for i in range(self.iterations):
                s, t, e = self.check_regular_execution()
                successes.append(s)
                times.append(t)
                energies.append(e)

        if check_energy:
            ##########################
            # check energy consumption
            ##########################
            mean_energy = np.mean(energies)
            print("\n*********************************")
            print("Energy Consumption")
            print(mean_energy, "J")
            print("average over", self.iterations, "iterations")
            print("*********************************\n")

        if check_time:
            ##########################
            # check swingup time
            ##########################
            times = np.asarray(times)
            fails = np.count_nonzero(np.isinf(times))
            times = times[np.isfinite(times)]
            mean_swingup_time = np.mean(times)
            print("\n*********************************")
            print("Swingup time")
            print(mean_swingup_time, " s")
            print("average over", self.iterations - fails, "iterations")
            print(fails, " attempts failed")
            print("*********************************\n")

        if check_consistency:
            ##########################
            # check consistency
            ##########################
            c_success = 0
            for i in range(self.iterations):
                c_success += int(self.check_consistency())
            print("\n*********************************")
            print("Consistency")
            print("(Varying start state)")
            print(c_success, "/", self.iterations, " successful")
            print("*********************************\n")

        if check_stability:
            ##########################
            # check stability
            ##########################
            s_success = 0
            for i in range(self.iterations):
                s_success += int(self.check_stability())
            print("\n*********************************")
            print("Stability")
            print("(Random perturbations during execution)")
            print(s_success, "/", self.iterations, " successful")
            print("*********************************\n")

        if check_sensitivity:
            ##########################
            # check sensitivity
            ##########################
            sens_success = 0
            for i in range(self.iterations):
                sens_success += int(self.check_sensitivity())
            print("\n*********************************")
            print("Sensitivity")
            print("(Modified pendulum model parameters)")
            print(sens_success, "/", self.iterations, " successful")
            print("*********************************\n")

        if check_torque_limit:
            ##########################
            # check reduced torque limit
            ##########################
            rtl_success = 0
            tlimits = [10.0, 5.0, 2.0, 1.5, 1.0, 0.5, 0.25, 0.1]
            for i in range(len(tlimits)):
                s = self.check_reduced_torque_limit(tl=tlimits[i])
                rtl_success += int(s)
                if not s:
                    break
            print("\n*********************************")
            print("Reduced Torque limit")
            print("(Reduced pendulum torque limit)")
            print(rtl_success, "/", len(tlimits), " successful")
            if rtl_success > 0:
                print("Swingup successful with torque limit",
                      tlimits[rtl_success-1], " Nm")
            if rtl_success < len(tlimits):
                print("Swingup failed with torque limit",
                      tlimits[rtl_success], " Nm")
            print("*********************************\n")
