import sys
import pandas as pd

# check if motor id is provided
""""
motor_id = 0x03
if len(sys.argv) != 2:
    print('Provide Motor ID (e.g. 1, 2, 3)')
    sys.exit(0)
motor_id = int(sys.argv[1])
"""

# if we also want to use mjbots
""""
    if args.qdd100:
        (start, end, meas_dt, meas_pos, meas_vel, meas_tau, meas_time) = \
            asyncio.run(motor_control_loop.qdd100(CSV_FILE, n, dt,
                                                  des_pos_out, des_vel_out,
                                                  des_tau_in, meas_pos,
                                                  meas_vel, meas_tau,
                                                  meas_time, gr,
                                                  rad2outputrev))
 """

# read input data with pandas
"""
data = pd.read_csv("trajectory_optimisation/traj_opt_traj.csv")
pos_traj = data["pos"]
vel_traj = data["vel"]
tau_traj = data["torque"]
dt = data["time"][1] - data["time"][0]
"""

# additional options for the parser
"""
    parser.add_argument("-sim", action='store_true',
                        help="simulate the simple pendulum plant, instead "
                             "of controlling the real system")
    actuator = parser.add_mutually_exclusive_group(required=False)
    actuator.add_argument("-qdd100", action='store_true',
                          help="to control the qdd100 actuators from mjbots")
    actuator.add_argument("-ak80_6", action='store_true',
                          help="to control the AK80-6 actuators from t-motors")
"""
