#  Energy Shaping Control #

Type: Closed loop control

State/action space contraints: No

Optimal: No

Versatility: Swingup only

## Theory #

The energy of a simple pendulum in state \(x = [\theta, \dot{\theta}]\) is given by:

$$
\begin{equation}
E(\theta, \dot{\theta}) = \frac{1}{2}ml^2\dot{\theta}^2 - mgl\cos(\theta)
\end{equation}
$$

where the first term is the kinetic energy of the system and the second term is the potential energy.
In the standing upright position ( \(x = [\pi, 0]\) ) the whole energy of the pendulum is potential energy and the kinetic energy is zero. As that is the goal state of a swingup motion, the desired energy can be defined as the energy of that state:

$$
\begin{equation}
E_{des} = mgl
\end{equation}
$$

The idea behind energy-shaping control is simple:

- If \( E < E_{des} \), the controller adds energy to the system by outputting torque in the direction of motion.
- If \( E > E_{des} \), the controller subtracts energy from the system by outputting torque in the opposite direction of the direction of motion.

Consequently, the control function reads:
$$
\begin{equation}
u(\theta, \dot{\theta}) = -k \dot{\theta} \left( E(\theta, \dot{\theta}) - E_{des} \right), \quad k > 0
\end{equation}
$$

This controller is applicable in the whole state space of the pendulum, i.e. it will always push the system towards the upright position. Note however, that the controller does not stabilize the upright position! If the pendulum overshoots the unstable fixpoint, the controller will make the pendulum turn another round.

In order to stabilize the pendulum at the unstable fixpoint, energy-shaping control can be combined with a stabilizing controller such as the LQR controller at controllers.LQR.

## Requirements #

-

## API

The controller needs pendulum parameters as well as the control term k (see equaiton (3)) as input during initialization:

    EnergyShapingController.__init__(self, mass=1.0, length=0.5, damping=0.1, gravity=9.81, k=1.0)
        inputs:
            mass: float, default: 1.0
            length: float, default: 0.5
            damping: float, default: 0.1
            gravity: float, default: 9.81
            k: float, default: 1.0

Before using the controller, the function EnergyShapingController.set_goal must be called with input

    EnergyShapingController.set_goal(x)
        inputs:
            x: list of length 2

where x is the desired goal state. This function sets the desired energy for the output calculation.

The control output \(\mathbf{u}(\mathbf{x})\) can be obtained with the API of the abstract controller class: 

    EnergyShapingController.get_control_output(mean_pos, mean_vel, meas_tau, meas_time)
        inputs:
            meas_pos: float, position of the pendulum
            meas_vel: float, velocity of the pendulum
            meas_tau: not used
            meas_time: not used
        returns:
            None, None, u

get_control_output returns None for the desired position and desired velocity (the energy shaping controller is a pure torque controller). The returned torque u is the result of equation (3).

## Usage #

Start a simulation with energy-shaping control for pendulum swingup and lqr control stabilization at the unstable fixpoint:

    python sim_energyshaping.py


## Comments

