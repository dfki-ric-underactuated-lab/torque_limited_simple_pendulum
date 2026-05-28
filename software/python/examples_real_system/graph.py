import os
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt

from simple_pendulum.utilities import plot, process_data
from simple_pendulum.controllers import motor_control_loop
from simple_pendulum.controllers.discrete_graph.graph_controller import GraphController

# set motor parameters
motor_id = 0x01
can_port = "can0"

# pendulum parameters
mass = 0.57288  # 0.57288  # 0.6755
length = 0.4
damping = 0.1  # 15
gravity = 9.81
coulomb_fric = 0.05
inertia = mass * length * length
torque_limit = 1.5

dt = 0.01
t_final = 10.0

# set your workspace
TIMESTAMP = datetime.now().strftime("%Y%m%d-%I%M%S-%p")
save_path = f"data/graph/{TIMESTAMP}"
save_path_csv = os.path.join(save_path, "experiment_trajectory.csv")

# controller parameters
graph_dt = 0.1
graph_pos_res = np.deg2rad(1.0)
graph_vel_res = graph_pos_res / graph_dt
graph_vel_max = 50 * graph_vel_res

policy_path = "../../../data/models/graph/L0.4/dijkstra_35868.csv"
# policy_path = "../../../data/models/graph/L0.5/dijkstra_36360.csv"
controller = GraphController(
    graph_csv_path=policy_path,
    pos_res=graph_pos_res,
    vel_res=graph_vel_res,
    vel_max=graph_vel_max,
    damping=0.0,
    coulomb_fric=coulomb_fric,
    torque_limit=torque_limit,
)

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
