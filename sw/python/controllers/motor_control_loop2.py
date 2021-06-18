import sys
import time
import numpy as np
import asyncio

# mjbots moteus driver  
import moteus
# t-motors AK80-6 driver                                                                                        
from motor_driver.canmotorlib import CanMotorController                                                 
# import socket
# from bitstring import BitArray
                    
async def qdd100(CSV_FILE, n, dt, des_pos_out, des_vel_out, des_tau_in,
                 meas_pos, meas_vel, meas_tau, meas_time, gr, rad2outputrev):

    # define interface to "Controller" class in moteus.py
    c1 = moteus.Controller(id=1)

    # in case the controller had faulted previously,
    # the stop command clears all fault states
    await c1.set_stop()
    print("Motor enabled.")

    # defining running index variables
    i = 0
    t = 0.0
    meas_dt = 0.0
    exec_time = 0.0

    print("Executing trajectory:", CSV_FILE)
    start = time.time()

    while i < n:
        start_loop = time.time()
        t += meas_dt                    # add the real_dt to previous time step

        # read out the current data in each time step from the trajectory lists
        pos = des_pos_out[i]
        vel = des_vel_out[i]
        tau = des_tau_in[i]

        # position control
        state1 = await c1.set_position(position=pos, velocity=None, kp_scale=1,
                                       kd_scale=None, stop_position=None,
                                       feedforward_torque=None,
                                       maximum_torque=1.0,
                                       watchdog_timeout=None, query=True)

        # store measured sensor data of pos, vel and torque in each time step
        meas_pos[i] = state1.values[moteus.Register.POSITION]/rad2outputrev
        meas_vel[i] = state1.values[moteus.Register.VELOCITY]/rad2outputrev
        meas_tau[i] = state1.values[moteus.Register.TORQUE]*gr
        meas_time[i] = t
        i += 1

        exec_time = time.time() - start_loop
        if exec_time > dt:
            print("Control loop is too slow!")
            print("Control frequency:", 1/exec_time, "Hz")
            print("Desired frequency:", 1/dt, "Hz")
            print()

        while (time.time() - start_loop < dt):
            pass
        meas_dt = time.time() - start_loop
    end = time.time()

    # Disabling the motor
    await c1.set_position(position=None, velocity=None, kp_scale=0, kd_scale=0,
                          stop_position=None, feedforward_torque=0,
                          maximum_torque=0, watchdog_timeout=None, query=True)
    await c1.set_stop()
    print("Motor disabled.")

    return start, end, meas_dt, meas_pos, meas_vel, meas_tau, meas_time


def ak80_6(controller, kp, kd, n, dt):
    motor_id = 0x02
    can_port = 'can0'

    meas_pos = np.zeros(n)
    meas_vel = np.zeros(n)
    meas_tau = np.zeros(n)
    meas_time = np.zeros(n)

    des_pos_list = np.zeros(n)
    des_vel_list = np.zeros(n)
    des_tau_list = np.zeros(n)
    des_time_list = np.zeros(n)

    motor_controller = CanMotorController(can_port, motor_id)
    motor_controller.enable_motor()
    pos, vel, tau = motor_controller.send_deg_command(0, 0, 0, 0, 0)
    print()
    print("After enabling motor, pos: ", pos, ", vel: ", vel,
          ", tau: ", tau)

    # ensure that the actuator starts from the zero positon
    while abs(pos) > 1 or abs(vel) > 1 or abs(tau) > 0.1:
        motor_controller.set_zero_position()
        pos, vel, tau = motor_controller.send_deg_command(0, 0, 0, 0, 0)
        print("Motor position after setting zero position: ", pos,
              ", vel: ", vel, ", tau: ", tau)

    # defining running index variables
    i = 0
    t = 0.0
    vel_filter = vel
    vel_filter_list = []
    meas_dt = 0.0
    exec_time = 0.0

    print()
    print("Executing Energy Shaping.")
    start = time.time()

    while i < n:
        start_loop = time.time()
        t += meas_dt                # add the real_dt to the previous time step

        des_pos, des_vel, des_tau = controller.get_control_output(pos,
                                                                  vel,
                                                                  tau)

        if des_tau > 4.0:
             des_tau = 4.0
        if des_tau < -4.0:
             des_tau = -4.0

        #if des_pos is None:
        des_pos = 0
        #     kp = 0

        #if des_vel is None:
        des_vel = 0
        #     kd = 0

        # pd control
        pos, vel, tau = motor_controller.send_rad_command(des_pos, des_vel,
                                                          kp, kd, des_tau)
        # torque control
        # pos, vel, tau = motor_controller.send_rad_command(des_pos[i],
        # des_vel[i], kp, kd, des_tau[i])

    # store measured sensor data of pos, vel and torque in each time step
        meas_pos[i] = pos
        meas_vel[i] = vel
        meas_tau[i] = tau
        meas_time[i] = t

        des_pos_list[i] = des_pos
        des_vel_list[i] = des_vel
        des_tau_list[i] = des_tau
        des_time_list[i] = dt * i
        i += 1

        # low pass filter
        #vel_filter_list.append(vel)
        #if len(vel_filter_list) > 10:
        #    del vel_filter_list[0]

        vel_filter = np.mean(vel_filter_list)
        vel_filter = np.mean(meas_vel[max(0, i-50):i])

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

    return start, end, meas_dt, meas_pos, meas_vel, meas_tau, meas_time, \
           des_pos_list, des_vel_list, des_tau_list, des_time_list
