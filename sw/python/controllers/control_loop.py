import sys
import time
import asyncio

import socket
from bitstring import BitArray

import moteus                                       # mjbots moteus driver                                                  
from utilities import canmotorlib                   # t-motors AK80-6 driver                                                  
from utilities.canmotorlib import CanMotorController                
                    
async def qdd100(CSV_FILE, n, dt, des_pos_out, des_vel_out, des_tau_in, meas_pos, meas_vel, meas_tau, meas_time, gr, rad2outputrev):
    c1 = moteus.Controller(id=1)                                    # define the interface to the "Controller" class in moteus.py
    await c1.set_stop()                                             # in case the controller had faulted previously, the stop command clears all fault states
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
        t += meas_dt                                                    # add the real_dt to the previous time step

        pos = des_pos_out[i]                                            # read out the current data in each time step from the trajectory lists
        vel = des_vel_out[i]
        tau = des_tau_in[i]

        # position control
        state1 = await c1.set_position(position=pos, velocity=None, kp_scale=1, kd_scale=None, stop_position=None,
                                        feedforward_torque=None, maximum_torque=1.0, watchdog_timeout=None, query=True)

        # store the measured sensor data of position, velocity and torque in each time step
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
    await c1.set_position(position=None, velocity=None, kp_scale=0, kd_scale=0, stop_position=None,
                            feedforward_torque=0, maximum_torque=0, watchdog_timeout=None, query=True)
    await c1.set_stop()
    print("Motor disabled.")

    return start, end, meas_dt, meas_pos, meas_vel, meas_tau, meas_time

def ak80_6(CSV_FILE, data, can_port, motor_id, Kp, Kd, n, dt, des_pos, des_vel, des_tau, meas_pos, meas_vel, meas_tau, meas_time):
    # Motor ID
    motor_id = 0x01
    can_port = 'can0'
    motor_controller = CanMotorController(can_port, motor_id)
    motor_controller.enable_motor()
    pos, vel, effort = motor_controller.send_deg_command(0, 0, 0, 0, 0)
    print("After enabling motor, pos: ", pos, ", vel: ", vel, ", effort: ", effort)

    # ensure that the actuator starts from the zero positon
    while abs(pos) > 1 or abs(vel) > 1 or abs(effort) > 0.1:
        motor_controller.set_zero_position()
        pos, vel, effort = motor_controller.send_deg_command(0, 0, 0, 0, 0)
        print("Motor position after setting zero position: ", pos, ", vel: ", vel, ", effort: ", effort)

    # defining running index variables
    i = 0
    t = 0.0
    meas_dt = 0.0
    exec_time = 0.0

    print("Executing trajectory:", CSV_FILE)
    start = time.time()

    while i < n:
        start_loop = time.time()
        t += meas_dt                                                                             # add the real_dt to the previous time step

        # pd control
        pos, vel, tau = motor_controller.send_rad_command(des_pos[i], des_vel[i], Kp, Kd, 0)
        # torque control
        #pos, vel, tau = motor_controller.send_rad_command(des_pos[i], des_vel[i], kp, kd, des_tau[i]) 

    # store the measured sensor data of position, velocity and torque in each time step
        meas_pos[i] = pos
        meas_vel[i] = vel
        meas_tau[i] = tau
        meas_time[i] = t
        i += 1
        t = t + dt

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

    return start, end, meas_dt, meas_pos, meas_vel, meas_tau, meas_time