import os
from datetime import datetime
from pathlib import Path

from simple_pendulum.utilities import plot, process_data
from simple_pendulum.utilities.performance_profiler import profiler
from simple_pendulum.controllers import motor_control_loop
from simple_pendulum.controllers.gamepad.gamepad_controller import GamepadController

# set motor parameters
motor_id = 0x09
can_port = 'can0'

# set your workspace
WORK_DIR = Path(Path(os.path.abspath(__file__)).parents[3])
TIMESTAMP = datetime.now().strftime("%Y%m%d-%I%M%S-%p")
folder_name = "gamepad"
output_folder = str(WORK_DIR) + f'/results/{TIMESTAMP}_' + folder_name

# controller parameters
dt = 0.005
t_final = 60.
torque_limit = 1.0

controller = GamepadController(torque_limit=torque_limit,
                               gamepad_name="Logitech Logitech RumblePad 2 USB")

# start control loop for ak80_6
start, end, meas_dt, data_dict = motor_control_loop.ak80_6(controller,
                                                           kp=0.,
                                                           kd=0.,
                                                           torque_limit=torque_limit,
                                                           dt=dt,
                                                           tf=t_final,
                                                           motor_id=motor_id,
                                                           motor_type='AK80_6_V1p1',
                                                           can_port=can_port)

## performance profiler
#profiler(data_dict, start, end, meas_dt)

# save measurements
process_data.save(output_folder, data_dict)

# plot data
plot.swingup(False, output_folder, data_dict)
