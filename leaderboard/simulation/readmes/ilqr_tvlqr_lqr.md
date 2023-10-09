# iterative LQR with TVLQR and LQR

The [iterative linear quadratic regularizer (iLQR)](https://ieeexplore.ieee.org/abstract/document/6907001) is an extension of the LQR controller. The LQR controller linearizes the dynamics at a given state and and assumes that these linear dynamics are valid at every other system state as well. In contrast to that, the iLQR optimization method has the ability to take the full system dynamics into account and plan ahead by optimizing over a sequence of control inputs.

The algorithm can be described as:

1. Set an initial state <img src="https://render.githubusercontent.com/render/math?math=x_0"> and an initial control sequence <img src="https://render.githubusercontent.com/render/math?math=\mathbf{U} = [u_0, u_1, ..., u_{N-1}]">, where <img src="https://render.githubusercontent.com/render/math?math=N"> is the number of steps that will optimized over (the time horizon).
2. Rollout the trajectory by applying the control sequence iteratively to the initial state.

The following steps are repeated until convergence:

3. Backward pass: Compute the derivatives of the cost function and the gains for the control sequence
4. Forward pass: Update the control sequence with the computed gains and rollout the new trajectory with the new control sequence. If the cost of the new trajectory is smaller than before, carry over the new control sequence and increase the gain factor. If not, keep the old trajectory and decrease the gain factor.

If the cost is below a specified threshold the algorithm stops.

## Time-varying Linear Quadratic Regulator (TVLQR)

This controller is designed to follow a precomputed trajectory
 from of a csv file to the simulator or the real pendulum. Particulary, the controller can process trajectories that have been found with help of the [trajectory optimization](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/software/python/simple_pendulum/trajectory_optimization) methods.

The Time-varying Linear Quadratic Regulator (TVLQR) is an extension to the regular [LQR controller](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/software/python/simple_pendulum/controllers/lqr). The LQR formalization is used for a time-varying linear dynamics function

<img src="https://render.githubusercontent.com/render/math?math=\dot{\mathbf{x}} =  \mathbf{A}(t)\mathbf{x} %2B \mathbf{B}(t)\mathbf{u}">

The TVLQR controller tries to stabilize the system along a nominal trajectory. For this, at every timestep the system dynamics are linearized around the state of the nominal trajectory <img src="https://render.githubusercontent.com/render/math?math=\mathbf{x}_0(t), \mathbf{u}_0(t)"> at the given timestep <img src="https://render.githubusercontent.com/render/math?math=t">. The LQR formalism then can be used to derive the optimal controller at timestep <img src="https://render.githubusercontent.com/render/math?math=t">:

<img src="https://render.githubusercontent.com/render/math?math=u(\mathbf{x}) = \mathbf{u}_0(t) - \mathbf{K}(t) \left( \mathbf{x} - \mathbf{x}_0(t)\right)">

For further reading, we recommend chapter 8 of this [Underactuated Robotics](http://underactuated.mit.edu/) lecture.


## LQR Controller

A linear quadratic regulator (LQR) can be used to stabilize the pendulum at the unstable fixpoint. For a linear system of the form

<img src="https://render.githubusercontent.com/render/math?math=\dot{\mathbf{x}} =  \mathbf{A}\mathbf{x} %2B \mathbf{B}\mathbf{u}">

and a infinite horizon cost function in quadratic form:

<img src="https://render.githubusercontent.com/render/math?math=J = \int_0^{\infty} \left( \mathbf{x}^T \mathbf{Q}\mathbf{x} %2B \mathbf{u}^T \mathbf{R} \mathbf{u} \right)\text{d}t, \quad \mathbf{Q} = \mathbf{Q} \succeq 0, \, \mathbf{R} = \mathbf{R} \succeq 0">

the (provably) optimal controller is

<img src="https://render.githubusercontent.com/render/math?math=u(\mathbf{x}) = -\mathbf{R}^{-1}\mathbf{B}^{T}\mathbf{S} \mathbf{x} = -\mathbf{K} \mathbf{x}">

where <img src="https://render.githubusercontent.com/render/math?math=\mathbf{S}"> has to fulfill the algebraic Riccati equation

<img src="https://render.githubusercontent.com/render/math?math=\mathbf{SA} %2B \mathbf{A}^{T}\mathbf{S} - \mathbf{SBR}^{-1}\mathbf{B}\mathbf{S} %2B \mathbf{Q} = 0.">

There are many solvers for the algebraic Riccati equation. In this library the solver from the scipy package is used.
