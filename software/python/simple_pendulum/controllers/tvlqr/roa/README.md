# Region of Attraction (RoA) estimation 

The region of attraction estimation is an offline process that can be applied to the closed-loop dynamics of a system (Plant and Controller) in order to analyze the control behaviour. The time-varying version of this analysis offers a very interesting point of view to understand how much the state can vary with respect to a nominal trajectory that brings a fixed initial state to a desired state. Hence, this process results in a very powerful tool to verify the robustness of a time-varying controller.

## Probabilistic Method

For simple systems, such as the torque limited simple pendulum under TVLQR control, the RoA can be estimated by a very intuitive simulation-based method.
The implemented solution takes inspiration from some related works that has been studied and extended.
E. Najafi proposed a computationally effective sampling approach to estimate the DoAs of nonlinear systems in real time:
> E. Najafi, R. Babuška, and G. A. D. Lopes, “A fast sampling method for estimating the domain of attraction,” Nonlinear Dyn, vol. 86, no. 2, pp. 823–834, Oct. 2016, doi: 10.1007/11071-016-2926-7.

He used this method for estimating a time-invariant region of attraction in two different ways: "memoryless sampling" and "sampling with memory". We have exploited his results from the memoryless version by extending the study to the time-varying case.
This extension has been already somehow addressed by P. Reist which included it in a simulation-based variant of the LQR-Tree feedback-motion-planning approach:
> P. Reist, P. V. Preiswerk, and R. Tedrake, “Feedback-motion-planning with simulation-based
lqr-trees”, SAGE, 2016, https://groups.csail.mit.edu/robotics-center/public_papers/Reist15.pdf. 

However, his implementation is strictly related to the LQR-Tree algorithm while we will focus on the region of attraction estimation.

First we have to fix the number of knot points N.
Except for the given first and last values, rho is initially fixed to an N-dimensional array of infinity. The last value of rho comes from the  [time-invariant region of attraction estimation](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/software/python/simple_pendulum/controllers/lqr/roa), while the first one has to be fixed big enough to overcome its "real" value.
Now, after computing the nominal trajectory and the related TVLQR controller that bring to the goal we can start the estimation process.
Iterating backward in the knot points, we can exploit the knowledge of the final rho from the time-invariant case. The "previous" and the "next" ellipse have been considered. We sample random initial states from the "previous" ellipse and we simulate them until the "next" one. In doing so, we can check if the simulated trajectory exits from the "next" ellipse. The condition is the following one:

<img src="https://render.githubusercontent.com/render/math?math=\bar{\mathbf{x}}^{\mathrm{T}} \mathbf{S} \bar{\mathbf{x}} <\rho">

If a simulated trajectory triggers this condition, the algorithm shrinks the "previous" ellipse using the value of the computed optimal cost to go:

<img src="https://render.githubusercontent.com/render/math?math=\rho_{new} = \min (\rho_{old}, ctg)">   

An implementation detail gives the possibility to reduce the time consumption. The estimation of each ellipse has been considered done after a maximum number of successful simulations.