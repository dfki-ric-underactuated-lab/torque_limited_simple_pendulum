# Trajectory optimization using direct collocation

Type: Trajectory Optimization

State/action space constraints: Yes

Optimal: Yes

Versatility: Swingup and stabilization

## Theory

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

## API

The direct collocation algorithm explained above can be executed by using the DirectCollocationCalculator class

    dircal = DirectCollocationCalculator()

To parse the pendulum parameters to the calculator, do:

      dircal.init_pendulum(mass=0.5,
                           length=0.5,
                           damping=0.1,
                           gravity=9.81,
                           torque_limit=1.5)

The optimal trajectory can be computed with:

  x_trajectory, dircol, result = dircal.compute_trajectory(N=21,
                                                           max_dt=0.5,
                                                           start_state=[0.0, 0.0],
                                                           goal_state=[3.14, 0.0])

The method returns three pydrake objects containing the optimization results. The trajectory as numpy arrays can be extracted from these objects with

    T, X, XD, U = dircal.extract_trajectory(x_trajectory, dircol, result, N=1000)

where T is the time, X the position, XD the velocity and U the control trajectory. The parameter N determines here how with how many steps the trajectories are sampled.

The phase space of the trajectory can be plotted with

    dircal.plot_phase_space_trajectory(x_trajectory, save_to="None")

If a string with a path to a file location is parsed with 'save_to' the plot is saved there.

## Dependencies

The trajectory optimization using direct collocation for the pendulum swing-up is accomplished by taking advantage of [Drake toolbox [3]](https://drake.mit.edu/).

## References
[[1] Hargraves, Charles R., and Stephen W. Paris. "Direct trajectory optimization using nonlinear programming and collocation." Journal of guidance, control, and dynamics 10.4 (1987): 338-342](https://arc.aiaa.org/doi/pdf/10.2514/3.20223)

[[2] Russ Tedrake. Underactuated Robotics: Algorithms for Walking, Running, Swimming, Flying, and Manipulation (Course Notes for MIT 6.832).](http://underactuated.mit.edu/)

[[3] Model-Based Design and Verification for Robotics](https://drake.mit.edu/).
