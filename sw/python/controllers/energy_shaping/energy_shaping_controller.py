# Other imports
import numpy as np

# Local imports
from controllers.abstract_controller import AbstractController
from controllers.lqr.lqr_controller import LQRController


def pendulum_calc_kinetic_energy(theta_dot, mass, length):
    return 0.5*mass*(length*theta_dot)**2.0


def pendulum_calc_potential_energy(theta, mass, length, gravity):
    return mass*gravity*length*(1-np.cos(theta))


def pendulum_calc_total_energy(theta, theta_dot, mass, length, gravity):
    kin = pendulum_calc_kinetic_energy(theta_dot, mass, length)
    pot = pendulum_calc_potential_energy(theta, mass, length, gravity)
    return kin + pot


class EnergyShapingController(AbstractController):
    def __init__(self,
                 mass=1.0,
                 length=0.5,
                 damping=0.1,
                 gravity=9.81,
                 k=1.0):

        self.m = mass
        self.l = length
        self.b = damping
        self.g = gravity
        self.k = k

    def set_goal(self, x):
        self.goal = [x[0], x[1]]
        self.desired_energy = pendulum_calc_total_energy(theta=x[0],
                                                         theta_dot=x[1],
                                                         mass=self.m,
                                                         length=self.l,
                                                         gravity=self.g)

    def get_control_output(self, meas_pos, meas_vel,
                           meas_tau=0, meas_time=0):

        if isinstance(meas_pos, (list, tuple, np.ndarray)):
            pos = meas_pos[0]
        else:
            pos = meas_pos

        if isinstance(meas_vel, (list, tuple, np.ndarray)):
            vel = meas_vel[0]
        else:
            vel = meas_vel
        total_energy = pendulum_calc_total_energy(theta=pos,
                                                  theta_dot=vel,
                                                  mass=self.m,
                                                  length=self.l,
                                                  gravity=self.g)
        des_tau = -self.k*vel*(total_energy - self.desired_energy) + \
                   self.k*self.b*vel

        # since this is a pure torque controller,
        # set des_pos and des_vel to None
        des_pos = None
        des_vel = None

        return des_pos, des_vel, des_tau


class EnergyShapingAndLQRController(AbstractController):
    def __init__(self, mass=1.0, length=0.5, damping=0.1,
                 gravity=9.81, torque_limit=np.inf, k=1.0):

        self.m = mass
        self.l = length
        self.b = damping
        self.g = gravity

        self.energy_shaping_controller = EnergyShapingController(mass,
                                                                 length,
                                                                 damping,
                                                                 gravity,
                                                                 k=k)
        self.lqr_controller = LQRController(mass,
                                            length,
                                            damping,
                                            gravity,
                                            torque_limit)

        self.active_controller = "none"

    def set_goal(self, x):
        self.energy_shaping_controller.set_goal(x)

    def get_control_output(self, meas_pos, meas_vel,
                           meas_tau=0, meas_time=0, verbose=False):
        des_pos, des_vel, u = self.lqr_controller.get_control_output(meas_pos,
                                                                     meas_vel)
        if u is not None:
            if self.active_controller != "lqr":
                self.active_controller = "lqr"
                if verbose:
                    print("Switching to lqr control")
        else:
            if self.active_controller != "EnergyShaping":
                self.active_controller = "EnergyShaping"
                if verbose:
                    print("Switching to energy shaping control")
            des_pos, des_vel, u = (self.energy_shaping_controller.
                                   get_control_output(meas_pos, meas_vel))
        return des_pos, des_vel, u
