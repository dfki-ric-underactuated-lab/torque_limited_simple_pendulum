#  Proportional–Integral–Derivative (PID) Control #

Type: Closed loop control

State/action space contraints: -

Optimal: -

Versatility: -

## Theory #

This controller is designed to follow a precomputed trajectory
 from of a csv file to the simulator or the real pendulum. Particulary, the controller can process trajectories that have been found with help of the trajectory optimization methods in software/python/trajectory_optimization.

## Requirements #

-

## API

The controller needs pendulum parameters as input during initialization:

    PIDController.__init__(self, csv_path, Kp, Ki, Kd, use_feed_forward=True)
        inputs:
            csv_path: string
                path to a csv file containing a trajectory in the below specified format
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

The csv file should have 4 columns with values for [time, position, velocity, torque] respectively. The values shopuld be separated with a comma. Each row in the file is one timestep. The number of rows can vary.
The values are assumed to be in SI units, i.e. time in s, position in rad, velocity in rad/s, torque in Nm.
The first line in the csv file is reserved for comments and will be skipped during read out.

Example:

    # time, position, velocity, torque
    0.00, 0.00, 0.00, 0.10
    0.01, 0.01, 0.01, -0.20
    0.02, ....

The control output $`\mathbf{u}(\mathbf{x})`$ can be obtained with the API of the abstract controller class:

    PIDController.get_control_output(mean_pos, mean_vel, meas_tau, meas_time)
        inputs:
            meas_pos: float, position of the pendulum
            meas_vel: float, velocity of the pendulum
            meas_tau: not used
            meas_time: not used
        returns:
            des_pos, des_vel, u

The function returns the desired position, desired velocity as specified in the csv file at the given index. The returned torque processed by the PID control terms via (use_feed_forward=True):
```math
u(t) = \tau + K_p e(t) + K_i \int_0^t e(t') \text{d}t' + K_d \frac{\text{d}e(t)}{\text{d}t}
```
where $`\tau`$ is the torque from the csv file and $`e(t)`$ is the position error at timestep t.
If use_feed_forward is set to False, the torque from the csv file is omitted:
```math
u(t) = K_p e(t) + K_i \int_0^t e(t') \text{d}t' + K_d \frac{\text{d}e(t)}{\text{d}t}
```

The index counter is incremeted by 1 every time the get_control_output function is called.

## Usage #

Before using this controller, first a trajectory has to be defined/calculated
and stored to csv file in a suitable format.

## Comments

The controller will not scale the trajectory according to the time values in the csv file. All it does is process the rows of the file one by one with each call of get_control_output.

