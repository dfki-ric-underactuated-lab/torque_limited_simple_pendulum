#  iLQR Model Predictive Control #

Type: Model Predictive Control

State/action space contraints: No

Optimal: Yes

Versatility: Swingup and stabilization

## Theory #

This controller uses the trajectory optimization from the iLQR algorithm (see [iLQR](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/software/python/simple_pendulum/trajectory_optimization/ilqr/README.md)) in an MPC setting. This means that this controller recomputes an optimal trajectory (including the optimal sequence of control inputs) at every time step. The first control input of this solution is returned as control input for the current state. As the optimization happens every timestep the iLQR algorithm is only executed with one forward and one backward pass. As initial trajectory the solution of the previous timestep is parsed to the iLQR solver.

## Requirements #

pydrake (see [getting_started](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/docs/installation_guide.md))

## API

The controller can be initialized as:

    controller = iLQRMPCController(mass=0.5,
                                   length=0.5,
                                   damping=0.1,
                                   coulomb_friction=0.0,
                                   gravity=9.81,
                                   x0=[0.0,0.0],
                                   dt=0.02,
                                   N=50,
                                   max_iter=1,
                                   break_cost_redu=1e-1,
                                   sCu=30.0,
                                   sCp=0.001,
                                   sCv=0.001,
                                   sCen=0.0,
                                   fCp=100.0,
                                   fCv=1.0,
                                   fCen=100.0,
                                   dynamics="runge_kutta",
                                   n_x=n2)

where

- mass, length, damping, coulomb_friction, gravity are the pendulum parameters (see [PendulumPlant](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/software/python/simple_pendulum/model/README.md))
- x0:   array like, start state
- dt:   float, time step
- N:    int, time steps the controller plans ahead
- max_iter: int, number of optimization loops
- break_cost_redu: cost at which the optimization stops
- sCu: float, stage cost coefficient penalizing the control input every step
- sCp: float, stage cost coefficient penalizing the position error every step
- sCv: float, stage cost coefficient penalizing the velocity error every step
- sCen: float, stage cost coefficient penalizing the energy error every step
- fCp: float, final cost coefficient penalizing the position error at the final state
- fCv: float, final cost coefficient penalizing the velocity error at the final state
- fCen: float, final cost coefficient penalizing the energy error at the final state
- dynamics: string, "euler" for euler integrator, "runge_kutta" for Runge-Kutta integrator
- nx: int, nx=2, or n_x=3 for pendulum, n_x=2 uses <img src="https://render.githubusercontent.com/render/math?math=[\theta, \dot{\theta}]"> as pendulum state during the optimization, n_x=3 uses <img src="https://render.githubusercontent.com/render/math?math=[\cos(\theta), \sin(\theta), \dot{\theta}]"> as state

Before using the controller a goal has to be set via

    controller.set_goal(goal=[np.pi, 0])

which initializes the cost function derivatives inside the controller for the specified goal.

It is possible to either load an initial guess for the trajectory from a csv file by using

    controller.load_initial_guess(filepath="../../../../data/trajectories/iLQR/trajectory.csv"

for example a trajectory that has been found with the offline trajectory optimization [iLQR](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/software/python/simple_pendulum/trajectory_optimization/ilqr).
Alternatively, it is possible to compute a new initial guess with

    controller.compute_initial_guess(N=50)

With an initial guess set the control output can be obtained with the standard controller api

    controller.get_control_output(meas_pos, meas_vel,
                                  meas_tau=0, meas_time=0):

where only the measured position (meas_pos) and measured velocity (meas_vel) are used in the control loop. The function returns

- None, None, u

get_control_output returns None for the desired position and desired velocity (the iLQR controller is a pure torque controller). u is the first control input of the computed control sequence. as described in the Theory section.

## Usage #

The controller can be tested in simulation with

    python sim_ilqrMPC.py

## Comments

For coulomb_fricitons != 0 the optimization gets considerably slower.
