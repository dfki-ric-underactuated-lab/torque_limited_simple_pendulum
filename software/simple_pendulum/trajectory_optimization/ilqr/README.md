#  Iterative Linear Quadratic Requlator (iLQR) #

Type: Trajectory Optimization

State/action space contraints: No

Optimal: Yes

Versatility: Swingup and stabilization

## Theory #

The [iterative linear quadratic regularizer (iLQR) [1]](https://ieeexplore.ieee.org/abstract/document/6907001) is an extension of the LQR controller. The LQR controller linearizes the dynamics at a given state and and assumes that these linear dynamics are valid at every other system state as well. In contrast to that, the iLQR optimization method has the ability to take the full system dynamics into account and plan ahead by optimizing over a sequence of control inputs.

The algorithm can be described as:

1. Set an initial state <img src="https://render.githubusercontent.com/render/math?math=x_0"> and an initial control sequence <img src="https://render.githubusercontent.com/render/math?math=\mathbf{U} = [u_0, u_1, ..., u_{N-1}]">, where <img src="https://render.githubusercontent.com/render/math?math=N"> is the number of steps that will optimized over (the time horizon).
2. Rollout the trajectory by applying the control sequence iteratively to the initial state.

The following steps are repeated until convergence:

3. Backward pass: Compute the derivatives of the cost function and the gains for the control sequence
4. Forward pass: Update the control sequence with the computed gains and rollout the new trajectory with the new control sequence. If the cost of the new trajectory is smaller than before, carry over the new control sequence and increase the gain factor. If not, keep the old trajectory and decrease the gain factor.

If the cost is below a specified threshold the algorithm stops.

## API #

The iLQR algorithm is computed in the iLQR_Calculator class. The class can be used as follows:

The iLQR calculator has to be initialized with the dimension of the state space (n_x) and the dimension of the actuation space (n_u) of the system:

    iLQR = iLQR_Calculator(n_x=2, n_u=1)

For the pendulum n_x=2 (positions and velocity) and n_u=1. Next the dynamics and cost function have to be set in the calculator by using:

    iLQR.set_discrete_dynamics(dynamics)
    iLQR.set_stage_cost(stage_cost)
    iLQR.set_final_cost(final_cost)

where dynamics is a function of the form

    dynamics(x, u):
        ...
        return xd

i.e. takes the current state and control input as inputs and returns the integrated dynamics. Note that the time step dt is set by the definition of this function.

Similarily, state_cost and final_cost are functions of the form:

    stage_cost(x,u):
        ...
        return cost

    final_cost(x):
        ...
        return cost

Important: These functions have to be differentiable either with the pydrake symbolic library or with sympy! Examples for these functions for the pendulum are implemented in [pendulum.py](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/software/python/simple_pendulum/trajectory_optimization/ilqr/pendulum.py). With the 'partial' function from the 'functools' package additional input parameters of these functons can be set before passing the function with the correct input parameters to the iLQR solver. For an example usage of the partial function for this context see [compute_pendulum_iLQR.py](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/software/python/examples/compute_iLQR_swingup.py) in l.80 - l.87 for the dynamics and l.93 - l.113 for the cost functions.

Next: initialize the derivatives and the start state in the iLQR solver:

    iLQR.init_derivatives()
    iLQR.set_start(x0)

Finally, a trajectory can now be calculated with

    (x_trj, u_trj, cost_trace,
    regu_trace, redu_ratio_trace, redu_trace) = iLQR.run_ilqr(N=1000,
                                                              init_u_trj=None,
                                                              init_x_trj=None,
                                                              max_iter=100,
                                                              regu_init=100,
                                                              break_cost_redu=1e-6)

The run_ilqr function has the inputs

    - N: The number of timesteps to plan into the future
    - init_u_trj: Initial guess for the control sequence (optional)
    - init_u_trj: Initial guess for the state space trajectory (optional)
    - max_iter: Maximum number of iterations of forward and backward passes to compute
    - break_cost_redu: Break cost at which the computation stops early

Besides the state space trajectory x_trj and the control trajectory u_trj the calculation also returns the traces of the cost, regularization factor, the ratio of the cost reduction and the expected cost reduction and the cost reduction.

## Usage #

An example script for the pendulum can be found in the [examples directory](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/software/python/examples). It can be started with

    python compute_iLQR_swingup.py

## Comments #

The iLQR algorithm in this form cannot respect joint and torque limits. Instead, those  have to be enforced by penalizing unwanted values in the cost function.

## Requirements #

Optional: [pydrake [2]]((https://drake.mit.edu/)) (see [getting_started](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/docs/installation_guide.md))

## Notes #

The calculations with the pydrake symbolic library are about 30% faster than the calculations based on the sympy library in these implementations.

## References

[1] Y. Tassa, N. Mansard and E. Todorov, "Control-limited differential dynamic programming," 2014 IEEE International Conference on Robotics and Automation (ICRA), 2014, pp. 1168-1175, doi: [10.1109/ICRA.2014.6907001](https://ieeexplore.ieee.org/abstract/document/6907001).

[2] [Model-Based Design and Verification for Robotics](https://drake.mit.edu/).
