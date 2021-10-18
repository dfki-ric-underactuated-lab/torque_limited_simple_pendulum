#  Time-varying Linear Quadrativ Regulator (TVLQR) #

Type: Closed loop control

State/action space contraints: -

Optimal: -

Versatility: -

## Theory #

This controller is designed to follow a precomputed trajectory
 from of a csv file to the simulator or the real pendulum. Particulary, the controller can process trajectories that have been found with help of the [trajectory optimization](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/software/python/simple_pendulum/trajectory_optimization) methods.

The Time-varying Linear Quadratic Regulator (TVLQR) is an extension to the regular [LQR controller](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/software/python/simple_pendulum/controllers/lqr). The LQR formalization is used for a time-varying linear dynamics function

<img src="https://render.githubusercontent.com/render/math?math=\dot{\mathbf{x}} =  \mathbf{A}(t)\mathbf{x} %2B \mathbf{B}(t)\mathbf{u}">

The TVLQR controller tries to stabilize the system along a nominal trajectory. For this, at every timestep the system dynamics are linearized around the state of the nominal trajectory <img src="https://render.githubusercontent.com/render/math?math=\mathbf{x}_0(t), \mathbf{u}_0(t)"> at the given timestep <img src="https://render.githubusercontent.com/render/math?math=t">. The LQR formalism then can be used to derive the optimal controller at timestep <img src="https://render.githubusercontent.com/render/math?math=t">:

<img src="https://render.githubusercontent.com/render/math?math=u(\mathbf{x}) = \mathbf{u}_0(t) - \mathbf{K}(t) \left( \mathbf{x} - \mathbf{x}_0(t)\right)">

For further reading, we recommend chapter 8 of this [Underactuated Robotics [1]](http://underactuated.mit.edu/) lecture.

## API

The controller needs pendulum parameters as input during initialization:

    TVLQRController.__init__(self, data_dict, mass, length, damping, gravity, torque_limit)
        inputs:
            data_dict: dictionary
                A dictionary containing a trajectory in the below specified format
            mass: float, default: 1.0
            length: float, default: 0.5
            damping: float, default: 0.1
            gravity: float, default: 9.81
            torque_limit: float, default: np.inf

The data_dict dictionary should have the entries:

            data_dict["des_time_list"] : desired timesteps
            data_dict["des_pos_list"] : desired positions
            data_dict["des_vel_list"] : desired velocities
            data_dict["des_tau_list"] : desired torques

The values are assumed to be in SI units, i.e. time in s, position in rad, velocity in rad/s, torque in Nm.

The control output <img src="https://render.githubusercontent.com/render/math?math=\mathbf{u}(\mathbf{x})"> can be obtained with the API of the abstract controller class:

    TVLQRController.get_control_output(mean_pos, mean_vel, meas_tau, meas_time)
        inputs:
            meas_pos: float, position of the pendulum
            meas_vel: float, velocity of the pendulum
            meas_tau: not used
            meas_time: not used
        returns:
            des_pos, des_vel, u

The function returns the desired position, desired velocity as specified in the csv file at the given index. The returned torque is processed by the TVLQR controller as described in the theory section.

The index counter is incremeted by 1 every time the get_control_output function is called.

## Usage #

Before using this controller, first a trajectory has to be defined/calculated
and stored to csv file or dictionary in a suitable format.

## Dependencies

The trajectory optimization using direct collocation for the pendulum swing-up is accomplished by taking advantage of [Drake toolbox [2]](https://drake.mit.edu/).

## Comments

The controller will not scale the trajectory according to the time values in the csv file. All it does is process the rows of the file one by one with each call of get_control_output.

## References

[[1] Russ Tedrake. Underactuated Robotics: Algorithms for Walking, Running, Swimming, Flying, and Manipulation (Course Notes for MIT 6.832).](http://underactuated.mit.edu/)

[[2] Model-Based Design and Verification for Robotics](https://drake.mit.edu/).

