# Global imports
import time
import numpy as np
# driver for t-motors AK80-6
from motor_driver.canmotorlib import CanMotorController

from simple_pendulum.utilities.process_data import prepare_empty_data_dict


def ak80_6(controller, kp=0., kd=0., torque_limit=1.0, dt=0.005, tf=10.,
           motor_id=0x01, motor_type='AK80_6_V2', can_port='can0'):
    """
    Motor Control Loop
    ==================

    The motor control loop only contains the minimum of code necessary to
    send commands to and receive measurement data from the motor control
    board in real time over CAN bus. It specifies the outgoing CAN port and
    the CAN ID of the motor on the CAN bus and transfers this information to
    the motor driver. It furthermore requires the following arguments:

    The return values start, end and meas_dt are required to monitor if
    desired and measured time steps match.

    Parameters
    ----------
    controller : controller object inheriting from the abstract controller class
        Calls the controller, which executes the respective
        control policy and returns the input torques
    kp : float, default=0.0
        Weight for position control
    kd : float, default=0.0
        Weight for velocity control
    torque_limit : float default=1.0
        torque limit for the motor [Nm]
    dt : float, default=0.005
        time step in control loop [s]
    tf : float, default=10.0
        length of the experiment [s]
    motor_id : int, default=1
        id of the used motor
    motor_type : string, default='AK80_6_V1p1'
        type of the motor
    can_port : string, default='can0'
        can port for motor communication

    Returns
    -------
    data_dict : dict
        dictionary containing the recorded data
    """

    # load dictionary entries
    n = int(tf / dt)

    data_dict = prepare_empty_data_dict(dt, tf)

    kp_in = kp
    kd_in = kd
    control_tau_max = torque_limit

    # limit torque input to max. actuator torque
    motor01_tau_max = 12
    if control_tau_max > motor01_tau_max:
        control_tau_max = motor01_tau_max

    # connect to motor controller board
    motor_01 = CanMotorController(can_port, motor_id, motor_type)
    motor_01.enable_motor()

    # initiate actuator from zero position
    meas_pos, meas_vel, meas_tau = motor_01.send_deg_command(0, 0, 0, 0, 0)
    print("After enabling motor, pos: ", meas_pos, ", vel: ", meas_vel,
          ", tau: ", meas_tau)
    while abs(meas_pos) > 1 or abs(meas_vel) > 1.1 or abs(meas_tau) > 0.1:
        motor_01.set_zero_position()
        meas_pos, meas_vel, meas_tau = motor_01.send_deg_command(0, 0, 0, 0, 0)
        print("Motor position after setting zero position: ", meas_pos,
              ", vel: ", meas_vel, ", tau: ", meas_tau)

    if input('Do you want to proceed for real time execution?(y) ') == 'y':

        # print()
        # print("Executing", name)
        # print()
        # #print("Control type = ", attribute)
        # print("Torque limit is set to: ", control_tau_max)
        # print("kp = ", kp)
        # print("kd = ", kd)
        # print("Desired control frequency = ", 1/dt, " Hz")
        # print("Parameters can be changed within the corresponding .yaml file "
        #       "under ~/data/parameters/.")
        # print()

        # defining runtime variables
        i = 0
        meas_dt = 0.0
        meas_time = 0.0
        vel_filtered = 0
        # start = time.time()

        try:
            while i < n:
                start_loop = time.time()
                meas_time += meas_dt

                des_pos, des_vel, des_tau = controller.get_control_output(
                    meas_pos, vel_filtered, meas_tau, meas_time)

                if des_pos is None:
                    des_pos = 0
                    kp_in = 0
                else:
                    kp_in = kp

                if des_vel is None:
                    des_vel = 0
                    kd_in = 0
                else:
                    kd_in = kd


                # clip max.torque for safety
                if des_tau > control_tau_max:
                    des_tau = control_tau_max
                if des_tau < -control_tau_max:
                    des_tau = -control_tau_max


                # send control input
                meas_pos, meas_vel, meas_tau = motor_01.send_rad_command(
                    des_pos, des_vel, kp_in, kd_in, des_tau)


                # filter noisy velocity measurements
                if i > 0:
                    vel_filtered = np.mean(data_dict["meas_vel"][max(0, i-10):i])
                else:
                    vel_filtered = 0
                # or use the time derivative of the position instead
                # vel_filtered = (meas_pos - meas_pos_prev) / dt
                # meas_pos_prev = meas_pos

                # record data
                data_dict["meas_pos"][i] = meas_pos
                data_dict["meas_vel"][i] = meas_vel
                data_dict["meas_tau"][i] = meas_tau
                data_dict["meas_time"][i] = meas_time
                data_dict["vel_filt"][i] = vel_filtered
                data_dict["des_pos"][i] = des_pos
                data_dict["des_vel"][i] = des_vel
                data_dict["des_tau"][i] = des_tau
                data_dict["des_time"][i] = dt * i

                i += 1
                exec_time = time.time() - start_loop
                if exec_time > dt:
                    print("Control loop is too slow!")
                    print("Control frequency:", 1/exec_time, "Hz")
                    print("Desired frequency:", 1/dt, "Hz")
                    print()
                while time.time() - start_loop < dt:
                    pass
                meas_dt = time.time() - start_loop

        except BaseException as e:
            print('*******Exception Block!********')
            print(e)

    try:
        print("Disabling Motors...")
        motor_01.send_rad_command(0, 0, 0, 0, 0)
        motor_01.disable_motor()
    except BaseException as e:
        print('Motors already disabled')

    return data_dict
