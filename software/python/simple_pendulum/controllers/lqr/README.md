#  LQR Control #

Type: Closed loop control

State/action space contraints: No

Optimal: Yes

Versatility: Stabilization only

## Theory #

A linear quadratic regulator (LQR) can be used to stabilize the pendulum at the unstable fixpoint. For a linear system of the form

<img src="https://render.githubusercontent.com/render/math?math=\dot{\mathbf{x}} =  \mathbf{A}\mathbf{x} %2B \mathbf{B}\mathbf{u}">

and a infinite horizon cost function in quadratic form:

<img src="https://render.githubusercontent.com/render/math?math=J = \int_0^{\infty} \left( \mathbf{x}^T \mathbf{Q}\mathbf{x} %2B \mathbf{u}^T \mathbf{R} \mathbf{u} \right)\text{d}t, \quad \mathbf{Q} = \mathbf{Q} \succeq 0, \, \mathbf{R} = \mathbf{R} \succeq 0">

the (provably) optimal controller is

<img src="https://render.githubusercontent.com/render/math?math=u(\mathbf{x}) = -\mathbf{R}^{-1}\mathbf{B}^{T}\mathbf{S} \mathbf{x} = -\mathbf{K} \mathbf{x}">

where <img src="https://render.githubusercontent.com/render/math?math=\mathbf{S}"> has to fulfill the algebraic Riccati equation

<img src="https://render.githubusercontent.com/render/math?math=\mathbf{SA} %2B \mathbf{A}^{T}\mathbf{S} - \mathbf{SBR}^{-1}\mathbf{B}\mathbf{S} %2B \mathbf{Q} = 0.">

There are many solvers for the algebraic Riccati equation. In this library the solver from the scipy package is used.

## API

The controller needs pendulum parameters as input during initialization:

    LQRController.__init__(self, mass=1.0, length=0.5, damping=0.1, gravity=9.81, torque_limit=np.inf)
        inputs:
            mass: float, default: 1.0
            length: float, default: 0.5
            damping: float, default: 0.1
            gravity: float, default: 9.81
            torque_limit: float, default: np.inf

The control output <img src="https://render.githubusercontent.com/render/math?math=\mathbf{u}(\mathbf{x})"> can be obtained with the API of the abstract controller class:

    LQRController.get_control_output(mean_pos, mean_vel, meas_tau, meas_time)
        inputs:
            meas_pos: float, position of the pendulum
            meas_vel: float, velocity of the pendulum
            meas_tau: not used
            meas_time: not used
        returns:
            None, None, u

get_control_output returns None for the desired position and desired velocity (the LQR controller is a pure torque controller). The returned torque u is the result of equation (3).
If the calculated torque is out of bounds of the pendulum's torque limits the controller will return u=None as torque.

## Usage #

A usage example can be found in the [examples folder](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/software/python/examples). The controller can be tested in simulation with

    python sim_lqr.py

## Comments

Without torque limits the LQR controller can drive the pendulum up from any position in a straight way. In practice this controller should only be used for stabilizing the pendulum at the unstable fixpoint. If the controller would require a torque larger than the pendulums torque limit, the controller returns None instead. This makes it possible to combine this controller with another controller and only use the LQR control if the output is not None. The region of attraction where this controller is able to stabilize the pendulum depends on the pendulum parameters and especially its torque limits.

