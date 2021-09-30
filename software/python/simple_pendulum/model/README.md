#  Pendulum Dynamics #

The PendulumPlant class contains the kinematics and dynamics functions of the simple, torque limited pendulum.

## Theory #

<div align="center">
<img width="100" src="../../../../docs/pendulum.png">
</div>

The equations of motion of a pendulum are

```math
\begin{equation}
I\ddot{\theta} + b\dot{\theta} + c_f \text{sign}(\dot{\theta}) + mgl \sin(\theta) = \tau
\end{equation}
```

where

- $`\theta`$, $`\dot{\theta}`$, $`\ddot{\theta}`$ are the angular displacement, angular velocity and angular acceleration of the pendulum. $`\theta=0`$ means the pendulum is at its stable fixpoint (i.e. hanging down).
- $`I`$ is the inertia of the pendulum. For a point mass: $`I=ml^2`$
- $`m`$ mass of the pendulum
- $`l`$ length of the pendulum
- $`b`$ damping friction coefficient
- $`c_f`$ coulomb friction coefficient
- $`g`$ gravity (positive direction points down)
- $`\tau`$ torque applied by the motor


## API #

The pendulum plant can be initialized as follows

    pendulum = PendulumPlant(mass=1.0,
                             length=0.5,
                             damping=0.1,
                             gravity=9.81,
                             coulomb_fric=0.02,
                             inertia=None,
                             torque_limit=2.0)

where the input parameters correspond to the parameters in the equaiton of motion (1). The input inertia=None is the default and the intertia is set to the inertia of a point mass at the end of the pendulum stick ($`I=ml^2`$). Additionally, a torque_limit can be passed to the class. Torques greater than the torque_limit or smaller than -torque_limit will be cut off.

The plant can now be used to calculate the forward kinematics with

    [[x,y]] = pendulum.forward_kinematics(pos)

where pos is the angle ($`\theta`$) of interest. This function returns the (x,y) coordinates of the tip of the pendulum inside a list. The return is a list of all link coordinates of the system (as the pendulum has only one, this returns [[x,y]]).

Similarily, inverse kinematics can be computed with

    pos = pendulum.inverse_kinematics(ee_pos)

where ee_pos is a list of the end_effector coordinates [x,y]. pendulum.inverse_kinematics returns the angle of the system as a float.

Forward dynamics can be calculated with

    accn = pendulum.forward_dynamics(state, tau)

where state is the state of the pendulum $`[\theta, \dot{theta=]`$ and tau the motor torque as a float. The function returns the angular acceleration.

For inverse kinematics:

    tau = pendulum.inverse_kinematics(state, accn)

where again state is the state of the pendulum $`[\theta, \dot{\theta}]`$ and accn the acceleration. The function return the motor torque $`\tau`$ that would be neccessary to produce the desired acceleration at the specified state.

Finally, the function

    res = pendulum.rhs(t, state, tau)

returns the integrand of the equaitons of motion, i.e. the object that can be calculated with a time step to obtain the forward evolution of the system. The API of the function is written to match the API requested inside the simulator class.
t is the time which is not used in the pendulum dynamics (the dynamics do not change with time). state again is the pendulum state and tau the motor torque. res is a numpy array with shape np.shape(res)=(2,) and res = $`[\dot{\theta}, \ddot{\theta}]`$.


## Usage #

The class is inteded to be used inside the simulator class.


## Comments #

