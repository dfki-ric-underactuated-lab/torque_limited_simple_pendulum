# Other imports
import numpy as np
from functools import partial

# Set path for local imports
import site
site.addsitedir('../..')

# Local imports
from controllers.abstract_controller import AbstractClosedLoopController
from trajectory_optimization.iLQR.iLQR import iLQR_Calculator
from trajectory_optimization.iLQR.pendulum import pendulum_discrete_dynamics_euler, \
                                                  pendulum_discrete_dynamics_rungekutta, \
                                                  pendulum_swingup_stage_cost, \
                                                  pendulum_swingup_final_cost, \
                                                  pendulum3_discrete_dynamics_euler, \
                                                  pendulum3_discrete_dynamics_rungekutta, \
                                                  pendulum3_swingup_stage_cost, \
                                                  pendulum3_swingup_final_cost


class iLQRMPCController(AbstractClosedLoopController):
    def __init__(self,
                 # parameter,
                 mass=0.5,
                 length=0.5,
                 damping=0.15,
                 coulomb_friction=0.0,
                 gravity=9.81,
                 inertia=0.125,
                 x0=np.array([0.0, 0.0]),
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
                 n_x=3):

        # self.gravity = parameter[0]
        # self.length = parameter[4]
        # self.mass = parameter[10]
        # self.damping = parameter[12]
        # self.coulomb_friction = parameter[13]
        self.mass = mass
        self.length = length
        self.damping = damping
        self.coulomb_friction = coulomb_friction
        self.gravity = gravity

        self.N = n
        self.n_x = n_x

        self.sCu = sCu
        self.sCp = sCp
        self.sCv = sCv
        self.sCen = sCen
        self.fCp = fCp
        self.fCv = fCv
        self.fCen = fCen

        self.break_cost_redu = break_cost_redu
        self.max_iter = max_iter

        # Setup dynamics function in ilqr calculator
        self.iLQR = iLQR_Calculator(n_x=n_x, n_u=1)
        if n_x == 2:
            if dynamics == "euler":
                dyn_func = pendulum_discrete_dynamics_euler
            else:
                dyn_func = pendulum_discrete_dynamics_rungekutta
        elif n_x == 3:
            if dynamics == "euler":
                dyn_func = pendulum3_discrete_dynamics_euler
            else:
                dyn_func = pendulum3_discrete_dynamics_rungekutta
        dyn = partial(dyn_func,
                      dt=dt,
                      m=mass,
                      l=length,
                      b=damping,
                      cf=coulomb_friction,
                      g=gravity,
                      inertia=inertia)
        self.iLQR.set_discrete_dynamics(dyn)

        self.iLQR.set_start(x0)

    def load_initial_guess(self, filepath="Pendulum_data/trajectory.csv",
                           verbose=True):
        '''
        load initial guess trajectory from file
        '''
        if verbose:
            print("Loading initial guess from ", filepath)
        if self.n_x == 2:
            trajectory = np.loadtxt(filepath, skiprows=1, delimiter=",")
            self.u_trj = trajectory.T[3].T[:self.N]
            self.u_trj = np.expand_dims(self.u_trj, axis=1)
            self.x_trj = trajectory.T[1:3].T[:self.N]
            self.x_trj = np.insert(self.x_trj, 0, self.x_trj[0], axis=0)
        if self.n_x == 3:
            trajectory = np.loadtxt(filepath, skiprows=1, delimiter=",")
            self.u_trj = trajectory.T[3].T[:self.N]
            self.u_trj = np.expand_dims(self.u_trj, axis=1)
            self.x_trj = trajectory.T[1:3].T[:self.N]
            self.x_trj = np.insert(self.x_trj, 0, self.x_trj[0], axis=0)
            sin = np.sin(self.x_trj.T[0])
            cos = np.cos(self.x_trj.T[0])
            v = self.x_trj.T[1]
            self.x_trj = np.stack((cos, sin, v), axis=1)

    def set_initial_guess(self, u_trj=None, x_trj=None):
        '''
        set initial guess from array like object
        '''
        if u_trj is not None:
            self.u_trj = u_trj[:self.N]
        if x_trj is not None:
            self.x_trj = x_trj[:self.N]

    def compute_initial_guess(self, N=None, verbose=True):
        '''
        compute initial guess
        '''
        if verbose:
            print("Computing initial guess")
        if N is None:
            N = self.N

        (self.x_trj, self.u_trj,
         cost_trace, regu_trace,
         redu_ratio_trace,
         redu_trace) = self.iLQR.run_ilqr(N=N,
                                          init_u_trj=None,
                                          init_x_trj=None,
                                          max_iter=500,
                                          regu_init=100,
                                          break_cost_redu=1e-6)
        self.x_traj = self.x_trj[:self.N]
        self.u_traj = self.u_trj[:self.N]
        if verbose:
            print("Computing initial guess done")

    def set_goal(self, x):
        if self.n_x == 2:
            s_cost_func = pendulum_swingup_stage_cost
            f_cost_func = pendulum_swingup_final_cost
        elif self.n_x == 3:
            s_cost_func = pendulum3_swingup_stage_cost
            f_cost_func = pendulum3_swingup_final_cost

        s_cost = partial(s_cost_func,
                         goal=x,
                         Cu=self.sCu,
                         Cp=self.sCp,
                         Cv=self.sCv,
                         Cen=self.sCen,
                         m=self.mass,
                         l=self.length,
                         b=self.damping,
                         cf=self.coulomb_friction,
                         g=self.gravity)
        f_cost = partial(f_cost_func,
                         goal=x,
                         Cp=self.fCp,
                         Cv=self.fCv,
                         Cen=self.fCen,
                         m=self.mass,
                         l=self.length,
                         b=self.damping,
                         cf=self.coulomb_friction,
                         g=self.gravity)

        self.iLQR.set_stage_cost(s_cost)
        self.iLQR.set_final_cost(f_cost)
        self.iLQR.init_derivatives()

    def get_control_output(self, meas_pos, meas_vel,
                           meas_tau=0, meas_time=0):

        if isinstance(meas_pos, (list, tuple, np.ndarray)):
            pos = meas_pos[0]
        else:
            pos = meas_pos

        pos = pos % (2*np.pi)

        if isinstance(meas_vel, (list, tuple, np.ndarray)):
            vel = meas_vel[0]
        else:
            vel = meas_vel

        if self.n_x == 2:
            state = np.array([pos, vel])
        if self.n_x == 3:
            state = np.asarray([np.cos(pos),
                                np.sin(pos),
                                vel])
        self.iLQR.set_start(state)
        (self.x_trj, self.u_trj,
         cost_trace, regu_trace,
         redu_ratio_trace,
         redu_trace) = self.iLQR.run_ilqr(init_u_trj=self.u_trj,
                                          init_x_trj=None,
                                          shift=True,
                                          max_iter=self.max_iter,
                                          regu_init=100,
                                          break_cost_redu=self.break_cost_redu)

        # since this is a pure torque controller,
        # set pos_des and vel_des to None
        des_pos = None
        des_vel = None

        return des_pos, des_vel, self.u_trj[0]
