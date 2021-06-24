# Global imports
import time
import numpy as np
# package for t-motors AK80-6
from motor_driver.canmotorlib import CanMotorController


def ak80_6(control_method, name, attribute, params, prep):
    motor_id = 0x02
    can_port = 'can0'
    dt = prep.dt
    n = prep.n
    kp = params['kp']
    kd = params['kd']
    lim_tau = params['torque_limit']

    # connect to motor controller board
    motor_controller = CanMotorController(can_port, motor_id)
    motor_controller.enable_motor()

    # initiate actuator from zero position
    meas_pos, meas_vel, meas_tau = motor_controller.send_deg_command(0, 0, 0,
                                                                     0, 0)
    print()
    print("After enabling motor, pos: ", meas_pos, ", vel: ", meas_vel,
          ", tau: ", meas_tau)
    while abs(meas_pos) > 1 or abs(meas_vel) > 1 or abs(meas_tau) > 0.1:
        motor_controller.set_zero_position()
        meas_pos, meas_vel, meas_tau = motor_controller.send_deg_command(0, 0,
                                                                         0, 0,
                                                                         0)
        print("Motor position after setting zero position: ", meas_pos,
              ", vel: ", meas_vel, ", tau: ", meas_tau)
        print()
        print("Executing ", name)
        print()
        print("Desired Control Frequency = ", 1/dt, " Hz")
        print("Torque limit: ", lim_tau)
        print("kp = ", kp)
        print("kd = ", kd)

    # defining runtime variables
    i = 0
    meas_time = 0.0
    meas_dt = 0.0
    exec_time = 0.0
    start = time.time()

    while i < n:
        start_loop = time.time()
        meas_time += meas_dt

        if attribute is "open loop":
            # get control input
            des_pos, des_vel, des_tau = control_method.get_control_output()

            # clip max.torque for safety
            if des_tau > lim_tau:
                des_tau = lim_tau
            if des_tau < -lim_tau:
                des_tau = -lim_tau

            # send control input to the actuator
            meas_pos, meas_vel, meas_tau = motor_controller.send_rad_command(
                des_pos, des_vel, kp, kd, des_tau)

            # record data
            prep.meas_pos_list[i] = meas_pos
            prep.meas_vel_list[i] = meas_vel
            prep.meas_tau_list[i] = meas_tau
            prep.meas_time_list[i] = meas_time

        if attribute is "closed loop":
            # get control input
            des_pos, des_vel, des_tau = control_method.get_control_output(
                meas_pos, meas_vel, meas_tau, meas_time)

            # clip max.torque for safety
            if des_tau > lim_tau:
                des_tau = lim_tau
            if des_tau < -lim_tau:
                des_tau = -lim_tau

            # send control input
            meas_pos, meas_vel, meas_tau = motor_controller.send_rad_command(
                des_pos, des_vel, kp, kd, des_tau)

            # record data
            prep.meas_pos_list[i] = meas_pos
            prep.meas_vel_list[i] = meas_vel
            prep.meas_tau_list[i] = meas_tau
            prep.meas_time_list[i] = meas_time
            prep.des_pos_list[i] = des_pos
            prep.des_vel_list[i] = des_vel
            prep.des_tau_list[i] = des_tau
            prep.des_time_list[i] = dt * i

            # filter noise velocity measurements
            vel_filtered = np.mean(meas_vel[max(0, i-50):i])

        else:
            print("Missing control loop attribute.")

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

    # Disable the motor
    motor_controller.send_rad_command(0, 0, 0, 0, 0)
    motor_controller.disable_motor()

    return start, end, meas_dt, prep.des_pos_list, prep.des_vel_list, \
        prep.des_tau_list, prep.des_time_list, prep.meas_pos_list, \
        prep.meas_vel_list,  prep.meas_tau_list
