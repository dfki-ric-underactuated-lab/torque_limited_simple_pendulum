import os
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt

from simple_pendulum.utilities import plot, process_data
from simple_pendulum.controllers import motor_control_loop
from simple_pendulum.controllers.ilqr.iLQR_MPC_controller_cpp import (
    iLQRMPCcppController,
)


# set motor parameters
motor_id = 0x01
can_port = "can0"

# pendulum parameters
mass = 0.57288  # 0.57288  # 0.6755
length = 0.4
damping = 0.1  # 15
gravity = 9.81
coulomb_fric = 0.05  # 0.05
inertia = mass * length * length
torque_limit = 1.5

# set your workspace
TIMESTAMP = datetime.now().strftime("%Y%m%d-%I%M%S-%p")
save_path = f"data/iLQR_MPC/{TIMESTAMP}"
save_path_csv = os.path.join(save_path, "experiment_trajectory.csv")

# controller parameters
dt = 0.01
t_final = 10.0
x0 = np.array([0.0, 0.0])
goal = np.array([np.pi, 0])

controller = iLQRMPCcppController(
    mass=mass,
    length=length,
    damping=damping,
    coulomb_friction=coulomb_fric,
    gravity=gravity,
    inertia=inertia,
    torque_limit=torque_limit,
    dt=dt,
    n=200,  # horizon size
    max_iter=10,
    break_cost_redu=1e-10,
    sCu=20.0,
    sCp=20.0,
    sCv=2.0,
    sCen=0.0,
    fCp=200.0,
    fCv=10.0,
    fCen=0.0,
    dynamics="runge_kutta",
)
controller.set_goal(goal)
controller.init(x0=x0)

# start control loop for ak80_6
data_dict = motor_control_loop.ak80_6(
    controller,
    kp=0.0,
    kd=0.0,
    torque_limit=torque_limit,
    dt=dt,
    tf=t_final,
    motor_id=motor_id,
    motor_type="AK80_6_V2",
    can_port=can_port,
)

# save measurements
process_data.save_trajectory(save_path_csv, data_dict)
# write_to_leaderboard(leaderboard_file, swingup_time)

# plot data
# plot.swingup(False, output_folder, data_dict)
fig, ax = plt.subplots(3, 1, figsize=(18, 6), sharex="all")
ax[0].plot(data_dict["meas_time"], data_dict["meas_pos"], label="theta")
ax[0].set_ylabel("angle [rad]")
ax[0].legend(loc="best")
ax[1].plot(data_dict["meas_time"], data_dict["meas_vel"], label="theta dot")
ax[1].set_ylabel("angular velocity [rad/s]")
ax[1].legend(loc="best")
ax[2].plot(data_dict["meas_time"], data_dict["meas_tau"], label="u")
ax[2].set_xlabel("time [s]")
ax[2].set_ylabel("input torque [Nm]")
ax[2].legend(loc="best")
plt.show()
