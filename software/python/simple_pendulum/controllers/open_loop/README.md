#  Open Loop Control #

Type: Open loop control

State/action space contraints: -

Optimal: -

Versatility: -

## Theory #

This controller is designed to feed a precomputed trajectory
 in from of a csv file to the simulator or the real pendulum. Particulary, the controller can process trajectories that have been found with help of the [trajectory optimization](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/software/python/simple_pendulum/trajectory_optimization) methods.

## API

The controller needs pendulum parameters as input during initialization:

    OpenLoopController.__init__(self, data_dict)
        inputs:
            data_dict: dictionary
                A dictionary containing a trajectory in the below specified format

The data_dict dictionary should have the entries:

            data_dict["des_time_list"] : desired timesteps
            data_dict["des_pos_list"] : desired positions
            data_dict["des_vel_list"] : desired velocities
            data_dict["des_tau_list"] : desired torques

The values are assumed to be in SI units, i.e. time in s, position in rad, velocity in rad/s, torque in Nm.

The control output \(\mathbf{u}(\mathbf{x})\) can be obtained with the API of the abstract controller class:

    OpenLoopController.get_control_output(mean_pos, mean_vel, meas_tau, meas_time)
        inputs:
            meas_pos: float, position of the pendulum
            meas_vel: float, velocity of the pendulum
            meas_tau: not used
            meas_time: not used
        returns:
            des_pos, des_vel, u

The function returns the desired position, desired velocity and a torque as specified in the csv file at the given index. The index counter is incremeted by 1 every time the get_control_output function is called.

## Usage #

Before using this controller, first a trajectory has to be defined/calculated
and stored to csv file in a suitable format.

## Comments

The controller will not scale the trajectory according to the time values in the csv file. All it does is return the rows of the file one by one with each call of get_control_output.

