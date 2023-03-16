from datetime import datetime
import matplotlib.pyplot as plt

from simple_pendulum.utilities import plot, process_data
from simple_pendulum.controllers import motor_control_loop
from simple_pendulum.controllers.gamepad.gamepad_controller import GamepadController
# from simple_pendulum.analysis.leaderboard import get_swingup_time

from simple_leaderboard import write_to_leaderboard


# set motor parameters
motor_id = 0x01
can_port = "can0"

# pendulum parameters
mass = 0.57288
length = 0.4
damping = 0.30  # 0.15
gravity = 9.81
coulomb_fric = 0.19
inertia = mass * length * length

# set your workspace
TIMESTAMP = datetime.now().strftime("%Y%m%d-%I%M%S-%p")
save_csv_path = f"data/gamepad/{TIMESTAMP}/experiment_trajectory.csv"
leaderboard_file = "data/simple_leaderboard.csv"

# controller parameters
dt = 0.005
t_final = 60.0
torque_limit = 0.5

controller = GamepadController(
    torque_limit=torque_limit,
    gamepad_name="Logitech Logitech RumblePad 2 USB",
    # gamepad_name="Logitech WingMan Cordless Gamepad",
    mass=mass,
    length=length,
    damping=damping,
    gravity=gravity,
    lqr_torque_limit=2.0,
    max_vel=10.0,
    rumble=True,
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

swingup_time = controller.get_swingup_time()

print("\n Your swing-up time was: ", swingup_time, "s\n")

# save measurements
process_data.save_trajectory(save_csv_path, data_dict)
write_to_leaderboard(leaderboard_file, swingup_time)

# # plot data
# #plot.swingup(False, output_folder, data_dict)
# fig, ax = plt.subplots(3, 1, figsize=(18, 6), sharex="all")
# ax[0].plot(data_dict["meas_time"], data_dict["meas_pos"], label="theta")
# ax[0].set_ylabel("angle [rad]")
# ax[0].legend(loc="best")
# ax[1].plot(data_dict["meas_time"], data_dict["meas_vel"], label="theta dot")
# ax[1].set_ylabel("angular velocity [rad/s]")
# ax[1].legend(loc="best")
# ax[2].plot(data_dict["meas_time"], data_dict["meas_tau"], label="u")
# ax[2].set_xlabel("time [s]")
# ax[2].set_ylabel("input torque [Nm]")
# ax[2].legend(loc="best")
# plt.show()
