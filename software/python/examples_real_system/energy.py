import os
from datetime import datetime
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

from simple_pendulum.utilities import plot, process_data
from simple_pendulum.utilities.performance_profiler import profiler
from simple_pendulum.controllers import motor_control_loop
from simple_pendulum.controllers.energy_shaping.energy_shaping_controller import EnergyShapingAndLQRController


# set motor parameters
motor_id = 0x09
can_port = 'can0'

# pendulum parameters
mass = 0.57288 #0.6755
length = 0.4
damping = 0.15
gravity = 9.81
coulomb_fric = 0.19
inertia = mass*length*length

# set your workspace
WORK_DIR = Path(Path(os.path.abspath(__file__)).parents[3])
TIMESTAMP = datetime.now().strftime("%Y%m%d-%I%M%S-%p")
folder_name = "energy"
output_folder = str(WORK_DIR) + f'/results/{TIMESTAMP}_' + folder_name

# controller parameters
dt = 0.005
t_final = 10.
torque_limit = 0.5
k = 1.0

controller = EnergyShapingAndLQRController(
                                    mass=mass,
                                    length=length,
                                    damping=damping,
                                    coulomb_fric=coulomb_fric,
                                    gravity=gravity,
                                    torque_limit=torque_limit,
                                    k=k)
controller.set_goal([np.pi, 0])

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
#plot.swingup(False, output_folder, data_dict)
fig, ax = plt.subplots(3, 1, figsize=(18, 6), sharex="all")
ax[0].plot(data_dict["meas_time_list"], data_dict["meas_pos_list"], label="theta")
ax[0].set_ylabel("angle [rad]")
ax[0].legend(loc="best")
ax[1].plot(data_dict["meas_time_list"], data_dict["meas_vel_list"], label="theta dot")
ax[1].set_ylabel("angular velocity [rad/s]")
ax[1].legend(loc="best")
ax[2].plot(data_dict["meas_time_list"], data_dict["meas_tau_list"], label="u")
ax[2].set_xlabel("time [s]")
ax[2].set_ylabel("input torque [Nm]")
ax[2].legend(loc="best")
plt.show()
