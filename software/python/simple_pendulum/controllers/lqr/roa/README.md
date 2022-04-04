# Region of Attraction(RoA) estimation #

The RoA estimation is a process that can be applied to the closed-loop dynamics of a system, i.e. Plant + Controller, in order to analize its state-space behaviour. Two different methods have been implemented here for studying the dynamics of the underactuated pendulum coupled with an LQR controller. 

## SOS method
 The implementation here is based on
> Tedrake, Russ, Ian R. Manchester, Mark Tobenkin, and John W. Roberts. “LQR-Trees: Feedback Motion Planning via Sums-of-Squares Verification.” The International Journal of Robotics Research 29, no. 8 (July 2010): 1038–52. https://doi.org/10.1177/0278364910369189.

Sum of squares optimization provides a natural generalization of SDP to optimizing over positive polynomials. Hence this method can be exploited to formulate and solve problems from Lyapunov analysis, at least fot the polynomial systems.  
Furthermore, any sublevel set of a Lyapunov function is also an invariant set. This permitts to use sublevel sets of a Lyapunov function as approximation of the region of attraction for nonlinear systems.

First, the simple pendulum plant has been put in polynomial form via Taylor approximation around the up-right position, which will be the goal of the LQR controller. From different tests it seems that at least the third order approximation is necessary to obtain a satisfying result.

The LQR controller has been initialized to stabilize our closed-loop system around the up-right position. From that it has been obtained a state feedback matrix K and a matrix S, which is the solution of the matrix Lyapunov equation. The last one is very usefull because it can be used to ontain the Lyapunov function as $$V(x) = x^TSx$$ 
After obtaining the Lyapunov function the feasibility problem can be formulated as 
$$ -\dot{V}(x) + \lambda(x)(V(x)-\rho)\ is\ SOS\quad and \quad \lambda(x)\ is\ SOS  $$
This is coming from the so called "S-procedure" which makes use of the concept of Lagrangia multiplier to fomulate the Lyapunov conditions.  
It is important to highlight that other contraints are needed to include the torque inputs limits in the optimization problem.

Eventually, the above problem only verify one-sublevel set of the Lyapunov function. In order to obtain the best estimation, searching for the largest rho  that can satisfy these conditions is necessary. The two different methods that have been implemented are described below.

###  $\rho$  maximization: Simple line search #
Since the problem is convex with rho fixed, and rho is just a scalar, a simple line search on rho can be performed to find the maximum for which the convex optimization returns a feasible solution. In particular, a bisection-like algorithm has been implemented here for such a purpose. However, since this formulation can be computationally heavy then other formulations might be considered.

### $\rho$ maximization: Equality-constrained formulation #
This is an important variation since it makes use of the "S-procedure" to make the problem be jointly convex in $\lambda$(x) and $\rho$, so a single convex optimization is needed.  
Under the assumption that $\dot{V}(x)$ is negative-definite at the fixed point, the problem can be written as

$$ \begin{equation} \begin{aligned} \max_{\rho,\lambda} \quad & \rho\\
\textrm{s.t.} \quad & (x^Tx)^d(V(x) - \rho) + \lambda(x)\dot{V}(x)\ is \ SOS\\ \end{aligned} \end{equation} $$

Also in this case, input limits have then to be included to obtain the desired result.


## Usage #
Some usage example can be found in the [examples folder](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/software/python/examples). The different methods can be compared in simulation with

    python plot_roa_estimations.py

The RoA certification reliability can be verified with

    python verify_roa_estimation.py

Furthermore, the effects of the Taylor approximation on the closed-loop dynamics can be seen with

    python taylorApprox_roa_sos.py

## Comments #
Further possible improvements:

- Verifying the dynamics in implicit form will be necessary for more complex systems where the mass is not a scalar value. The SOS framework premitts it and would be usefull to try.
- Implementing more efficient ways to come up with a polynomial representation of the closed-loop dynamics. Approxiations usually generate differences between the real behaviour and the simulated one.
- Quotient ring of algebraic varieties in the "Equality-constrained formulation".