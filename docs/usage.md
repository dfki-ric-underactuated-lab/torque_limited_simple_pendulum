
<div align="center">

#  Simple Pendulum
</div>
<br/>

## Setting up the CAN interface

* Run this command and make sure that `can0` (or any other can interface depending on the system)shows up as an interface after connecting the USB cable to your laptop: `ip link show`

* Configure the `can0` interface to have a 1 Mbaud communication frequency: `sudo ip link set can0 type can bitrate 1000000`

* To bring up the `can0` interface, run: `sudo ip link set up can0`

  Note: Alternatively, one could run the shell script `setup_caninterface.sh` which will do the job for you. 

* To change motor parameters such as CAN ID or to calibrate the encoder, a serial connection is used. The serial terminal GUI used on linux for this purpose is `cutecom`
<br/>

## Usage
**Testing Communication:** To enable one motor at `0x01`, set zero position and disable the motor, run: `python3 can_motorlib_test.py can0`

**Use in Scripts:** Add the following import to your python script: `from canmotorlib import CanMotorController` after making sure this folder is available in the import path/PYTHONPATH.

Example Motor Initialization: `motor = CanMotorController(can_socket='can0', motor_id=0x01, socket_timeout=0.5)`

Available Functions:

- `enable_motor()`
- `disable_motor()`
- `set_zero_position()`
- `send_deg_command(position_in_degrees, velocity_in_degrees, Kp, Kd, tau_ff):`
- `send_rad_command(position_in_radians, velocity_in_radians, Kp, Kd, tau_ff):`

All functions return current position, velocity, torque in SI units except for `send_deg_command`.

**Performance Profiler:** Sends and received 1000 zero commands to measure the communication frequency with 1/2 motors. Be careful as the motor torque will be set to zero.
