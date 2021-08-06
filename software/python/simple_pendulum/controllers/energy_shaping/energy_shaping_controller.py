# Other imports
import numpy as np

# Local imports
from simple_pendulum.controllers.abstract_controller import AbstractController
from simple_pendulum.controllers.lqr.lqr_controller import LQRController


def pendulum_calc_kinetic_energy(theta_dot, mass, length):
    return 0.5*mass*(length*theta_dot)**2.0


def pendulum_calc_potential_energy(theta, mass, length, gravity):
    return mass*gravity*length*(1-np.cos(theta))


def pendulum_calc_total_energy(theta, theta_dot, mass, length, gravity):
    kin = pendulum_calc_kinetic_energy(theta_dot, mass, length)
    pot = pendulum_calc_potential_energy(theta, mass, length, gravity)
    return kin + pot


class EnergyShapingController(AbstractController):
    """
    Controller which swings up the pendulum by regulating its energy.
    """
    def __init__(self,
                 mass=1.0,
                 length=0.5,
                 damping=0.1,
                 gravity=9.81,
                 k=1.0):
        """
        Controller which swings up the pendulum by regulating its energy.

        Parameters
        ----------
        mass : float, default=1.0
            mass of the pendulum [kg]
        length : float, default=0.5
            length of the pendulum [m]
        damping : float, default=0.1
            damping factor of the pendulum [kg m/s]
        gravity : float, default=9.81
            gravity (positive direction points down) [m/s^2]
        k : float, default=1.0
            the weight determining the output torque with respect to the
            current energy level.
        """
        self.m = mass
        self.l = length
        self.b = damping
        self.g = gravity
        self.k = k

    def set_goal(self, x):
        """
        Set the goal for the controller.
        This function calculates the energy of the goal state.

        Parameters
        ----------
        x : array-like
            the goal state for the pendulum
        """
        self.goal = [x[0], x[1]]
        self.desired_energy = pendulum_calc_total_energy(theta=x[0],
                                                         theta_dot=x[1],
                                                         mass=self.m,
                                                         length=self.l,
                                                         gravity=self.g)

    def get_control_output(self, meas_pos, meas_vel,
                           meas_tau=0, meas_time=0):
        """
        The function to compute the control input for the pendulum actuator

        Parameters
        ----------
        meas_pos : float
            the position of the pendulum [rad]
        meas_vel : float
            the velocity of the pendulum [rad/s]
        meas_tau : float
            the meastured torque of the pendulum [Nm]
            (not used)
        meas_time : float
            the collapsed time [s]
            (not used)

        Returns
        -------
        des_pos : float
            the desired position of the pendulum [rad]
            (not used, returns None)
        des_vel : float
            the desired velocity of the pendulum [rad/s]
            (not used, returns None)
        des_tau : float
            the torque supposed to be applied by the actuator [Nm]
        """
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
    """
    Controller which swings up the pendulum with the energy shaping
    controller and stabilizes the pendulum with the lqr controller.
    """
    def __init__(self, mass=1.0, length=0.5, damping=0.1,
                 gravity=9.81, torque_limit=np.inf, k=1.0):
        """
        Controller which swings up the pendulum with the energy shaping
        controller and stabilizes the pendulum with the lqr controller.

        Parameters
        ----------
        mass : float, default=1.0
            mass of the pendulum [kg]
        length : float, default=0.5
            length of the pendulum [m]
        damping : float, default=0.1
            damping factor of the pendulum [kg m/s]
        gravity : float, default=9.81
            gravity (positive direction points down) [m/s^2]
        torque_limit : float, default=np.inf
            the torque_limit of the pendulum actuator
        k : float, default=1.0
            the weight determining the output torque with respect to the
            current energy level.
        """

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
        """
        Set the goal for the controller.

        Parameters
        ----------
        x : array-like
            the goal state for the pendulum
        """
        self.energy_shaping_controller.set_goal(x)

    def get_control_output(self, meas_pos, meas_vel,
                           meas_tau=0, meas_time=0, verbose=False):
        """
        The function to compute the control input for the pendulum actuator

        Parameters
        ----------
        meas_pos : float
            the position of the pendulum [rad]
        meas_vel : float
            the velocity of the pendulum [rad/s]
        meas_tau : float
            the meastured torque of the pendulum [Nm]
            (not used)
        meas_time : float
            the collapsed time [s]
            (not used)
        verbose : bool, default=False
            whether to print when the controller switches between
            energy shaping and lqr

        Returns
        -------
        des_pos : float
            the desired position of the pendulum [rad]
            (not used, returns None)
        des_vel : float
            the desired velocity of the pendulum [rad/s]
            (not used, returns None)
        des_tau : float
            the torque supposed to be applied by the actuator [Nm]
        """
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
