# Global imports
import time
import numpy as np
# package for t-motors AK80-6
from motor_driver.canmotorlib import CanMotorController


def ak80_6(control_method, name, attribute, params, data_dict):
    motor_id = 0x02
    can_port = 'can0'
    n = data_dict['n']

    if attribute is "open_loop":
        dt = data_dict['dt']
    if attribute is "closed_loop":
        dt = params['dt']
    des_time_list = data_dict["des_time_list"]
    des_pos_list = data_dict["des_pos_list"]
    des_vel_list = data_dict["des_vel_list"]
    des_tau_list = data_dict["des_tau_list"]

    # load parameters
    kp = params['kp']
    kd = params['kd']
    control_tau_max = params['torque_limit']

    # limit torque input to max. actuator torque
    motor01_tau_max = 12
    if control_tau_max > motor01_tau_max:
        control_tau_max = motor01_tau_max

    # connect to motor controller board
    motor_01 = CanMotorController(can_port, motor_id)
    motor_01.enable_motor()

    print()
    # initiate actuator from zero position
    meas_pos, meas_vel, meas_tau = motor_01.send_deg_command(0, 0, 0, 0, 0)
    print("After enabling motor, pos: ", meas_pos, ", vel: ", meas_vel,
          ", tau: ", meas_tau)
    while abs(meas_pos) > 1 or abs(meas_vel) > 1 or abs(meas_tau) > 0.1:
        motor_01.set_zero_position()
        meas_pos, meas_vel, meas_tau = motor_01.send_deg_command(0, 0, 0, 0, 0)
        print("Motor position after setting zero position: ", meas_pos,
              ", vel: ", meas_vel, ", tau: ", meas_tau)
    meas_time = 0.0
    vel_filtered = 0

    print()
    print("Executing", name)
    print()
    print("Control type = ", attribute)
    print("Torque limit is set to: ", control_tau_max)
    print("kp = ", kp)
    print("kd = ", kd)
    print("Desired control frequency = ", 1/dt, " Hz")
    print()

    # defining runtime variables
    i = 0
    meas_dt = 0.0
    start = time.time()

    while i < n:
        start_loop = time.time()
        meas_time += meas_dt

        if attribute is "open_loop":
            # get control input
            des_pos, des_vel, des_tau = control_method.get_control_output(
                des_time_list, des_pos_list, des_vel_list, des_tau_list)

            # clip max.torque for safety
            if des_tau > control_tau_max:
                des_tau = control_tau_max
            if des_tau < -control_tau_max:
                des_tau = -control_tau_max

            # send control input to the actuator
            meas_pos, meas_vel, meas_tau = motor_01.send_rad_command(
                des_pos, des_vel, kp, kd, des_tau)

            # record data
            data_dict["meas_pos_list"][i] = meas_pos
            data_dict["meas_vel_list"][i] = meas_vel
            data_dict["meas_tau_list"][i] = meas_tau
            data_dict["meas_time_list"][i] = meas_time
        if attribute is "closed_loop":
            # get control input
            des_pos, des_vel, des_tau = control_method.get_control_output(
                meas_pos, vel_filtered, meas_tau, meas_time)

            if des_pos is None:
                des_pos = 0
                kp = 0
            if des_vel is None:
                des_vel = 0
                kd = 0

            # clip max.torque for safety
            if des_tau > control_tau_max:
                des_tau = control_tau_max
            if des_tau < -control_tau_max:
                des_tau = -control_tau_max

            # send control input
            meas_pos, meas_vel, meas_tau = motor_01.send_rad_command(
                des_pos, des_vel, kp, kd, des_tau)

            # filter noise velocity measurements
            if i > 0:
                vel_filtered = np.mean(data_dict["meas_vel_list"][max(0,
                                                                      i-10):i])
            else:
                vel_filtered = 0
            # or instead use the time derivative of the position
            # vel_filtered = (meas_pos - meas_pos_prev) / dt
            # meas_pos_prev = meas_pos

            # record data
            data_dict["meas_pos_list"][i] = meas_pos
            data_dict["meas_vel_list"][i] = meas_vel
            data_dict["meas_tau_list"][i] = meas_tau
            data_dict["meas_time_list"][i] = meas_time
            data_dict["vel_filt_list"][i] = vel_filtered
            data_dict["des_pos_list"][i] = des_pos
            data_dict["des_vel_list"][i] = des_vel
            data_dict["des_tau_list"][i] = des_tau
            data_dict["des_time_list"][i] = dt * i
        #else:
        #   print("Missing control loop attribute.")

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
    end = time.time()

    print("Disabling Motors...")
    motor_01.send_rad_command(0, 0, 0, 0, 0)
    motor_01.disable_motor()

    return start, end, meas_dt, data_dict
