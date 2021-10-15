#  Gravity Compensation Control #

Type: Closed loop control

State/action space constraints: -

Optimal: -

Versatility: only compensates for gravitational force acting on the pendulum, no swing-up or stabilization at the upright position

## Theory #

A controller compensating the gravitational force acting on the pendulum. The control function is given by:

```math
\begin{equation}
u(\theta) = mgl \sin(\theta)
\end{equation}
```
where $`u`$ is commanded torque, $`m`$ is the 0,5kg mass attached to the rod together with the mass of the rod and the mounitng parts, $`l`$ is the length of the rod and $`theta`$ is the current position of the pendulum. 


While the controller is running it actively compensates for the gravitational force acting on the pendulum, therfore the pendulum can be moved as if it was in zero-g.
