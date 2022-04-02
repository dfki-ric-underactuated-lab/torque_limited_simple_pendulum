#  Proportional–Integral–Derivative (PID) Control #

Type: Closed loop control

State/action space contraints: -

Optimal: -

Versatility: -

## Theory #

This controller is designed to follow a precomputed trajectory
 from of a csv file to the simulator or the real pendulum. Particulary, the controller can process trajectories that have been found with help of the [trajectory optimization](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/software/python/simple_pendulum/trajectory_optimization) methods.

The torque processed by the PID control terms via (with feed forward torque):

<img src="https://render.githubusercontent.com/render/math?math=u(t) = \tau %2B K_p e(t) %2B K_i \int_0^t e(t') \text{d}t' + K_d \frac{\text{d}e(t)}{\text{d}t}">

where <img src="https://render.githubusercontent.com/render/math?math=\tau"> is the torque from the csv file and <img src="https://render.githubusercontent.com/render/math?math=e(t)"> is the position error at timestep t.
Without feed forward torque, the torque from the precomputed trajectory file is omitted:

<img src="https://render.githubusercontent.com/render/math?math=u(t) = K_p e(t) %2B K_i \int_0^t e(t') \text{d}t' %2B K_d \frac{\text{d}e(t)}{\text{d}t}">

## API

The controller needs pendulum parameters as input during initialization:

    PIDController.__init__(self, data_dict, Kp, Ki, Kd, use_feed_forward=True)
        inputs:
            data_dict: dictionary
                A dictionary containing a trajectory in the below specified format
            Kp : float
                proportional term,
                gain proportial to the position error
            Ki : float
                integral term,
                gain proportional to the integral
                of the position error
            Kd : float
                derivative term,
                gain proportional to the derivative of the position error
            use_feed_forward : bool
                whether to use the torque that is provided in the csv file

The data_dict dictionary should have the entries:

            data_dict["des_time_list"] : desired timesteps
            data_dict["des_pos_list"] : desired positions
            data_dict["des_vel_list"] : desired velocities
            data_dict["des_tau_list"] : desired torques

The values are assumed to be in SI units, i.e. time in s, position in rad, velocity in rad/s, torque in Nm.

The control output <img src="https://render.githubusercontent.com/render/math?math=\mathbf{u}(\mathbf{x})"> can be obtained with the API of the abstract controller class:

    PIDController.get_control_output(mean_pos, mean_vel, meas_tau, meas_time)
        inputs:
            meas_pos: float, position of the pendulum
            meas_vel: float, velocity of the pendulum
            meas_tau: not used
            meas_time: not used
        returns:
            des_pos, des_vel, u

The function returns the desired position, desired velocity as specified in the csv file at the given index. The returned torque is processed by the PID controller as described in the theory section.

The index counter is incremeted by 1 every time the get_control_output function is called.

## Usage #

Before using this controller, first a trajectory has to be defined/calculated
and stored to csv file in a suitable format.

## Comments

The controller will not scale the trajectory according to the time values in the csv file. All it does is process the rows of the file one by one with each call of get_control_output.

