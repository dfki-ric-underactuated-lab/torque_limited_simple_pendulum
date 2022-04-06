# Region of Attraction (RoA) estimation 

The RoA estimation is a process that can be applied to the closed-loop dynamics of a system, i.e. Plant + Controller, in order to analize its state-space behaviour. Two different methods have been implemented here for studying the dynamics of the underactuated pendulum coupled with an LQR controller. 

## SOS method
 The implementation here is based on
> Tedrake, Russ, Ian R. Manchester, Mark Tobenkin, and John W. Roberts. “LQR-Trees: Feedback Motion Planning via Sums-of-Squares Verification.” The International Journal of Robotics Research 29, no. 8 (July 2010): 1038–52. https://doi.org/10.1177/0278364910369189.

Sum of squares optimization provides a natural generalization of SDP to optimizing over positive polynomials. Hence this method can be exploited to formulate and solve problems from Lyapunov analysis, at least fot the polynomial systems.  
Furthermore, any sublevel set of a Lyapunov function is also an invariant set. This permitts to use sublevel sets of a Lyapunov function as approximation of the region of attraction for nonlinear systems.

First, the simple pendulum plant has been put in polynomial form via Taylor approximation around the up-right position, which will be the goal of the LQR controller. From different tests it seems that at least the third order approximation is necessary to obtain a satisfying result.

The LQR controller has been initialized to stabilize our closed-loop system around the up-right position. From that it has been obtained a state feedback matrix K and a matrix S, which is the solution of the matrix Lyapunov equation. The last one is very usefull because it can be used to obtain the Lyapunov function as <img src="https://render.githubusercontent.com/render/math?math=V(x) = x^TSx">  
After obtaining the Lyapunov function the feasibility problem can be formulated as   
<img src="https://render.githubusercontent.com/render/math?math=-\dot{V}(x) + \lambda(x)(V(x)-\rho)\ is\ SOS\quad and \quad \lambda(x)\ is\ SOS ">  
This is coming from the so called "S-procedure" which makes use of the concept of Lagrangian multiplier to fomulate the Lyapunov conditions.  
It is important to highlight that other contraints are needed to include the torque inputs limits in the optimization problem.

Eventually, the above problem only verify one-sublevel set of the Lyapunov function. In order to obtain the best estimation, searching for the largest rho  that can satisfy these conditions is necessary. The two different methods that have been implemented are described below.

###  Maximization of <img src="https://render.githubusercontent.com/render/math?math=\rho"> : Simple line search 
Since the problem is convex with rho fixed, and rho is just a scalar, a simple line search on <img src="https://render.githubusercontent.com/render/math?math=\rho"> can be performed to find the maximum for which the convex optimization returns a feasible solution. In particular, a bisection-like algorithm has been implemented here for such a purpose. However, since this formulation can be computationally heavy then other formulations might be considered.

### Maximization of <img src="https://render.githubusercontent.com/render/math?math=\rho"> : Equality-constrained formulation 
This is an important variation since it makes use of the "S-procedure" to make the problem be jointly convex in <img src="https://render.githubusercontent.com/render/math?math=\lambda(x)"> and <img src="https://render.githubusercontent.com/render/math?math=\rho">, so a single convex optimization is needed.  
Under the assumption that <img src="https://render.githubusercontent.com/render/math?math=\dot{V}(x)"> is negative-definite at the fixed point, the problem can be written as

<img src="https://render.githubusercontent.com/render/math?math=\max_{\rho,\lambda} \quad \rho">   

<img src="https://render.githubusercontent.com/render/math?math=\textrm{s.t.} \quad (x^Tx)^d(V(x) - \rho) + \lambda(x)\dot{V}(x)\ \ \ is \ \ \ SOS\\">

Also in this case, input limits have then to be included to obtain the desired result.

## Probabilistic Method
For simple systems, such as the torque limited simple pendulum under LQR control, the RoA can be estimated by just evaluating the Lyapunov conditions for initial states from a continuously shrinking estimate of the ROA. This approach was introduced in:
> E. Najafi, R. Babuška, and G. A. D. Lopes, “A fast sampling method for estimating the domain of attraction,” Nonlinear Dyn, vol. 86, no. 2, pp. 823–834, Oct. 2016, doi: 10.1007/s11071-016-2926-7.

The RoA is estimated by a sublevel set of a quadratic Lyapunov function:

<img src="https://render.githubusercontent.com/render/math?math=\begin{align*}\mathcal{B}%20=%20\left\{%20\mathbf{x}%20\vert%20V(\mathbf{x})%20%3C%20\rho%20\right\} = \left\{%20\mathbf{x}%20\vert%20V \bar{\mathbf{x}}^{\mathrm{T}} \mathbf{S} \bar{\mathbf{x}} <\rho \right\}.\end{align*}">

Where  <img src="https://render.githubusercontent.com/render/math?math=\bar{\mathbf{x}} =\mathbf{x} - \mathbf{x}^\star"> denotes the deviation from the fixed point <img src="https://render.githubusercontent.com/render/math?math=\mathbf{x}^\star">

A random initial condition <img src="https://render.githubusercontent.com/render/math?math=V(\hat{\mathbf{x}})"> is sampled from <img src="https://render.githubusercontent.com/render/math?math=\mathcal{B}"> and the Lyapunov conditions (<img src="https://render.githubusercontent.com/render/math?math=V(\hat{\mathbf{x}}) > 0$"> and <img src="https://render.githubusercontent.com/render/math?math=\dot{V}(\hat{\mathbf{x}}) = \nabla V \mathbf{f}(\mathbf{\hat{x}}) < 0">) are evaluated. If these conditions are satisfied, the next initial state is sampled. However, if these conditions are not met, the estimate is shrunk, such that <img src="https://render.githubusercontent.com/render/math?math=\rho = V(\hat{\mathbf{x}})">

## Comments 
Further possible improvements:

- Verifying the dynamics in implicit form will be necessary for more complex systems where the mass is not a scalar value. The SOS framework premitts it and would be usefull to try.
- Implementing more efficient ways to come up with a polynomial representation of the closed-loop dynamics. Approxiations usually generate differences between the real behaviour and the simulated one.
- Quotient ring of algebraic varieties in the "Equality-constrained formulation".
