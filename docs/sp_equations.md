<div align="center">

#  Simple Pendulum
</div>


### Equation of Motion


```math
\begin{equation}
I\ddot{\theta} + b\dot{\theta} + c_f \text{sign}(\dot{\theta}) + mgl \sin(\theta) = \tau
\end{equation}
```

<img align="right" img height="200" src="../docs/pendulum.png" />

where

- $`\theta`$, $`\dot{\theta}`$, $`\ddot{\theta}`$ are the angular displacement, angular velocity and angular acceleration of the pendulum. $`\theta=0`$ means the pendulum is at its stable fixpoint (i.e. hanging down).
- $`I`$ is the inertia of the pendulum. For a point mass: $`I=ml^2`$
- $`m`$ mass of the pendulum
- $`l`$ length of the pendulum
- $`b`$ damping friction coefficient
- $`c_f`$ coulomb friction coefficient
- $`g`$ gravity (positive direction points down)
- $`\tau`$ torque applied by the motor

The pendulum has two fixpoints, one of them being stable (the pendulum hanging down) and the other being unstable (the pendulum pointing upwards). A challenge from the control point of view is to swing the pendulum up to the unstable fixpoint and stabilize the pendulum in that state.

### Energy of the Pendulum
--------------------------------------------------------------------

* Kinetic Energy (K) = 
```math 
K = \frac{1}{2}ml^2\dot{\theta}^2 
```
* Potential Energy (U) 
```math 
U = - mgl\cos(\theta)
```
* Total Energy (E) 
```math 
E = K + U
```
<br/>
