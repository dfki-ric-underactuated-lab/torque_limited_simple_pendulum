# Other imports
import sys
import numpy as np

# Local imports
from simple_pendulum.controllers.abstract_controller import AbstractController

import cpppendulumilqr


class iLQRMPCController(AbstractController):
    """
    Controller which computes an ilqr solution at every timestep and uses
    the first control output.
    """

    def __init__(
        self,
        mass=0.5,
        length=0.5,
        damping=0.15,
        coulomb_friction=0.0,
        gravity=9.81,
        inertia=0.125,
        torque_limit=10.0,
        dt=0.01,
        n=50,
        max_iter=1,
        break_cost_redu=1e-6,
        sCu=10.0,
        sCp=0.001,
        sCv=0.001,
        sCen=0.0,
        fCp=1000.0,
        fCv=10.0,
        fCen=300.0,
        dynamics="runge_kutta",
    ):
        """
        Controller which computes an ilqr solution at every timestep and uses
        the first control output.

        Parameters
        ----------
        mass : float, default=1.0
            mass of the pendulum [kg]
        length : float, default=0.5
            length of the pendulum [m]
        damping : float, default=0.1
            damping factor of the pendulum [kg m/s]
        coulomb_friction : float, default=0.0
            coulomb_friciton term of the pendulum
        gravity : float, default=9.81
            gravity (positive direction points down) [m/s^2]
        inertia : float, default=0.125
            inertia of the pendulum
        dt : float, default=0.01
            timestep of the simulation
        n : int, default=50
            number of timnesteps the controller optimizes ahead
        max_iter : int, default=1
            optimization iterations the alogrithm makes at every timestep
        break_cost_redu : float, default=1e-6
            cost at which the optimization breaks off early
        sCu : float, default=10.0
            running cost weight for the control input u
        sCp : float, default=0.001
            running cost weight for the position error
        sCv : float, default=0.001
            running cost weight for the velocity error
        sCen : float, default=0.0
            running cost weight for the energy error
        fCp : float, default=1000.0
            final cost weight for the position error
        fCv : float, default=10.0
            final cost weight for the velocity error
        fCen : float, default=300.0
            final cost weight for the energy error
        dynamics : string, default="runge_kutta"
            string that selects the integrator to be used for the simulation
            options are: "euler", "runge_kutta"
        """

        self.mass = mass
        self.length = length
        self.inertia = inertia
        self.damping = damping
        self.coulomb_friction = coulomb_friction
        self.gravity = gravity
        self.torque_limit = torque_limit

        self.N = n
        self.dt = dt

        self.sCu = sCu
        self.sCp = sCp
        self.sCv = sCv
        self.sCen = sCen
        self.fCp = fCp
        self.fCv = fCv
        self.fCen = fCen

        self.break_cost_redu = break_cost_redu
        self.max_iter = max_iter
        self.regu_init = 100.0

        if dynamics == "euler":
            self.integrator_int = 1
        else:
            self.integrator_int = 2
        # self.iLQR = cppilqr.cppilqr(self.N)
        # self.iLQR.set_parameters(2, 1, self.integrator_int, self.dt)
        # self.iLQR.set_pendulum_parameters(self.mass,
        #                                   self.length,
        #                                   self.damping,
        #                                   self.coulomb_friction,
        #                                   self.gravity,
        #                                   self.inertia,
        #                                   self.torque_limit)
        # self.iLQR.set_cost_parameters(self.sCu,
        #                               self.sCp,
        #                               self.sCv,
        #                               self.sCen,
        #                               self.fCp,
        #                               self.fCv,
        #                               self.fCen)

        # # set default start position
        # self.x0 = np.array([0.0, 0.0])
        # self.iLQR.set_start(self.x0[0], self.x0[1])

    def init(self, x0, verbose=False):
        self.x0 = x0
        # self.iLQR.set_start(self.x0[0], self.x0[1])
        self.compute_initial_guess(verbose=verbose)

    def load_initial_guess(self, filepath="Pendulum_data/trajectory.csv", verbose=True):
        """
        load initial guess trajectory from file

        Parameters
        ----------
        filepath : string, default="Pendulum_data/trajectory.csv"
            path to the csv file containing the initial guess for
            the trajectory
        verbose : bool, default=True
            whether to print from where the initial guess is loaded
        """
        if verbose:
            print("Loading initial guess from ", filepath)
        trajectory = np.loadtxt(filepath, skiprows=1, delimiter=",")
        self.u_traj = trajectory.T[3].T[: self.N]
        self.u_traj = np.expand_dims(self.u_traj, axis=1)
        self.x_traj = trajectory.T[1:3].T[: self.N]
        self.x_traj = np.insert(self.x_traj, 0, self.x_traj[0], axis=0)

    def set_initial_guess(self, u_trj=None, x_trj=None):
        """
        set initial guess from array like object

        Parameters
        ----------
        u_trj : array-like, default=None
            initial guess for control inputs u
            ignored if u_trj==None
        x_trj : array-like, default=None
            initial guess for state space trajectory
            ignored if x_trj==None
        """
        if u_trj is not None:
            self.u_traj = u_trj[: self.N]
        if x_trj is not None:
            self.x_traj = x_trj[: self.N]

    def compute_initial_guess(self, N=None, verbose=True, max_iter=100):
        """
        compute initial guess

        Parameters
        ----------
        N : int, default=None
            number of timesteps to plan ahead
            if N==None, N defaults to the number of timesteps that is also
            used during the online optimization (n in the class __init__)
        verbose : bool, default=True
            whether to print when the initial guess calculation is finished
        """
        if verbose:
            print("Computing initial guess")
        if N is None:
            N = self.N

        iLQR = cpppendulumilqr.cppilqr(N)
        iLQR.set_parameters(2, 1, self.integrator_int, self.dt)
        iLQR.set_start(self.x0[0], self.x0[1])
        iLQR.set_goal(self.goal[0], self.goal[1])
        iLQR.set_pendulum_parameters(
            self.mass,
            self.length,
            self.damping,
            self.coulomb_friction,
            self.gravity,
            self.inertia,
            self.torque_limit,
        )

        iLQR.set_cost_parameters(
            self.sCu, self.sCp, self.sCv, self.sCen, self.fCp, self.fCv, self.fCen
        )

        iLQR.run_ilqr(max_iter, self.break_cost_redu, self.regu_init)

        u_traj = iLQR.get_u_traj()
        p_traj = iLQR.get_p_traj()
        v_traj = iLQR.get_v_traj()

        self.u_traj = u_traj[: self.N - 1]
        self.p_traj = p_traj[: self.N]
        self.v_traj = v_traj[: self.N]

        if verbose:
            print("Computing initial guess done")
            print("u_traj", self.u_traj)

    def set_goal(self, x):
        """
        Set a goal for the controller. Initializes the cost functions.

        Parameters
        ----------
        x : array-like
            goal state for the pendulum
        """
        # self.iLQR.set_goal(x[0], x[1])
        self.goal = x

    def shift_trajectory(self):
        self.u_traj = np.append(self.u_traj[1 : self.N], [0.0], axis=0)
        # print("u shift", self.u_traj)

    def get_control_output(self, meas_pos, meas_vel, meas_tau=0, meas_time=0):
        """
        The function to compute the control input for the pendulum actuator

        Parameters
        ----------
        meas_pos : float
            the position of the pendulum [rad]
        meas_vel : float
            the velocity of the pendulum [rad/s]
        meas_tau : float, default=0
            the meastured torque of the pendulum [Nm]
            (not used)
        meas_time : float, default=0
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
        # print("Gettting control output ...")
        pos = float(np.squeeze(meas_pos))
        vel = float(np.squeeze(meas_vel))
        pos = pos % (2 * np.pi)

        iLQR = cpppendulumilqr.cppilqr(self.N)
        iLQR.set_parameters(2, 1, self.integrator_int, self.dt)
        iLQR.set_pendulum_parameters(
            self.mass,
            self.length,
            self.damping,
            self.coulomb_friction,
            self.gravity,
            self.inertia,
            self.torque_limit,
        )
        iLQR.set_cost_parameters(
            self.sCu, self.sCp, self.sCv, self.sCen, self.fCp, self.fCv, self.fCen
        )
        iLQR.set_start(pos, vel)
        iLQR.set_goal(self.goal[0], self.goal[1])

        self.shift_trajectory()
        iLQR.set_u_init_traj(self.u_traj.copy())

        iLQR.run_ilqr(self.max_iter, self.break_cost_redu, self.regu_init)

        self.u_traj = iLQR.get_u_traj()
        # self.p_traj = iLQR.get_p_traj()
        # elf.v_traj = iLQR.get_v_traj()

        # since this is a pure torque controller,
        # set pos_des and vel_des to None
        des_pos = None
        des_vel = None

        return des_pos, des_vel, self.u_traj[0]
