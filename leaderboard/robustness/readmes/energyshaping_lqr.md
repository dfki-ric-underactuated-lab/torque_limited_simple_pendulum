# Energy Shaping and LQR Controller

## Energy Shaping

The energy of a simple pendulum in state <img src="https://render.githubusercontent.com/render/math?math=x = [\theta, \dot{\theta}]"> is given by:

<img src="https://render.githubusercontent.com/render/math?math=E(\theta, \dot{\theta}) = \frac{1}{2}ml^2\dot{\theta}^2 - mgl\cos(\theta)">

where the first term is the kinetic energy of the system and the second term is the potential energy.
In the standing upright position ( <img src="https://render.githubusercontent.com/render/math?math=x = [\pi, 0]"> ) the whole energy of the pendulum is potential energy and the kinetic energy is zero. As that is the goal state of a swingup motion, the desired energy can be defined as the energy of that state:

<img src="https://render.githubusercontent.com/render/math?math=E_{des} = mgl">

The idea behind energy-shaping control is simple:

- If <img src="https://render.githubusercontent.com/render/math?math=E &lt; E_{des}">, the controller adds energy to the system by outputting torque in the direction of motion.
- If <img src="https://render.githubusercontent.com/render/math?math=E &gt; E_{des}">, the controller subtracts energy from the system by outputting torque in the opposite direction of the direction of motion.

Consequently, the control function reads:

<img src="https://render.githubusercontent.com/render/math?math=u(\theta, \dot{\theta}) = -k \dot{\theta} \left( E(\theta, \dot{\theta}) - E_{des} \right), \quad k &gt; 0">

This controller is applicable in the whole state space of the pendulum, i.e. it will always push the system towards the upright position. Note however, that the controller does not stabilize the upright position! If the pendulum overshoots the unstable fixpoint, the controller will make the pendulum turn another round.

In order to stabilize the pendulum at the unstable fixpoint, energy-shaping control can be combined with a stabilizing controller such as the [LQR controller](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/software/python/simple_pendulum/controllers/lqr).

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

