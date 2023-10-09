# Direct Collocation with TVLQR and LQR

## Direct Collocation

Direct collocation is an approach from ***collocation methods*** , which transforms the optimal control problem into a mathematical programming problem. The numerical solution can be achieved directly by solving the new problem using sequential quadratic programming [[1]](https://arc.aiaa.org/doi/pdf/10.2514/3.20223)[[2]](http://underactuated.mit.edu/trajopt.html).
The formulation of the optimization problem at the collocation points is as follows:

<div align="center">
<img src="https://render.githubusercontent.com/render/math?math=%5Cbegin%7Balign*%7D%0A%20%20%20%20%20%20%20%20%5Cmin_%7B%7B%5Cbf%7Bx%7D%7D%5B.%5D%2C%7B%7Bu%7D%7D%5B.%5D%7D%20%5Csum_%7Bn_0%7D%5E%7BN-1%7D%26%0A%20%20%20%20%20%20%20%20%20%20h_%7Bn%7Dl(%7B%7Bu%7D%7D%5Bn%5D)%5C%5C%0A%20%20%20%20%20%20%20%20%5Ctextrm%7Bs.t.%7D%20%5Cquad%20%26%20%5Cdot%7B%7B%5Cbf%7Bx%7D%7D%7D(t_%7Bc%2Cn%7D)%3Df(%7B%5Cbf%7Bx%7D%7D(t_%7Bc%2Cn%7D)%2Cu(t_%7Bc%2Cn%7D))%2C%20%5Chspace%7B0.2cm%7D%20%5Cforall%20n%20%5Cin%20%5B0%2CN-1%5D%20%5C%5C%0A%20%20%20%20%20%20%20%20%20%20%26%20%7Cu%7C%20%5Cleq%20u_%7Bmax%7D%5C%5C%0A%20%20%20%20%20%20%20%20%20%20%26%20%7B%5Cbf%7Bx%7D%7D%5B0%5D%20%3D%20%7B%5Cbf%7Bx%7D%7D_0%5C%5C%0A%20%20%20%20%20%20%20%20%20%20%26%20%7B%5Cbf%7Bx%7D%7D%5BN%5D%20%3D%20%7B%5Cbf%7Bx%7D%7D_F%0A%5Cend%7Balign*%7D%0A">
</div>

<!--
```math
\begin{align*}
        \min_{{\bf{x}}[.],{{u}}[.]} \sum_{n_0}^{N-1}&
          h_{n}l({{u}}[n])\\
        \textrm{s.t.} \quad & \dot{{\bf{x}}}(t_{c,n})=f({\bf{x}}(t_{c,n}),u(t_{c,n})), \hspace{0.2cm} \forall n \in [0,N-1] \\
          & |u| \leq u_{max}\\
          & {\bf{x}}[0] = {\bf{x}}_0\\
          & {\bf{x}}[N] = {\bf{x}}_F
\end{align*}
-->

- <img src="https://render.githubusercontent.com/render/math?math={\bf{x}} = {{[\theta(.),\dot{\theta}(.)]}}^T">: Angular position and velocity are the states of the system

- <img src="https://render.githubusercontent.com/render/math?math=u">: Input torque of the system applied by motor

- <img src="https://render.githubusercontent.com/render/math?math=N = 21">: Number of break points in the trajectory

- <img src="https://render.githubusercontent.com/render/math?math=h_k = t_{k%2B1} - t_k">: Time interval between two breaking points

- <img src="https://render.githubusercontent.com/render/math?math=l(u) = u^TR u">: Running cost

- <img src="https://render.githubusercontent.com/render/math?math=R = 10">: Input weight

- <img src="https://render.githubusercontent.com/render/math?math=\dot{{\bf{x}}}(t_{c,n})">: [Nonlinear dynamics of the pendulum](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/software/python/simple_pendulum/model) considered as equality constraint at collocation point

- <img src="https://render.githubusercontent.com/render/math?math=t_{c,k} = \frac{1}{2}\left(t_k %2B t_{k%2B1}\right)">: A collocation point at time instant <img src="https://render.githubusercontent.com/render/math?math=k,(i.e.,{\bf{x}}[k] = {\bf{x}}(t_k))">,in which the collocation constraints depends on the decision variables  <img src="https://render.githubusercontent.com/render/math?math={\bf{x}}[k], {\bf{x}}[k%2B1], u[k], u[k%2B1]">

- <img src="https://render.githubusercontent.com/render/math?math=u_{max} = 10">: Maximum torque limit

- <img src="https://render.githubusercontent.com/render/math?math={\bf{x}}_0 = [\theta = 0,\dot{\theta} = 0]:"> Initial state constraint

- <img src="https://render.githubusercontent.com/render/math?math={\bf{x}}_F = [\theta = \pi,\dot{\theta} = 0]:"> Terminal state constraint


Minimum and a maximum spacing between sample times set to <img src="https://render.githubusercontent.com/render/math?math=0.05, 0.5">. It assumes a **first-order hold** on the input trajectory, in which the signal is reconstructed as a piecewise linear approximation to the original sampled signal.

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
