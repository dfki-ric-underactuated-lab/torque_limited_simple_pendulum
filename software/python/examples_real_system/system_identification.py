import numpy as np
import os

from simple_pendulum.utilities import plot
from simple_pendulum.model.system_identification import SystemIdentification


# setting path variables
result_folder = "20210611-040141-PM_pd"
output_path = os.path.join("../../../results", result_folder)

csv_file = "data_measured.csv"
csv_path = os.path.join(output_path, csv_file)

save_dir = "log_data/system_identification"
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# load results from csv file
trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
meas_time = trajectory.T[0].T       # desired time in s
meas_pos = trajectory.T[1].T        # desired position in radian
meas_vel = trajectory.T[2].T        # desired velocity in radian/s
meas_tau = trajectory.T[3].T        # desired torque in Nm
acc = None
plot.sys_id_unified(save_dir, meas_time, meas_pos, meas_vel, meas_tau, acc)

# function calls
SysId = SystemIdentification(meas_time, meas_pos, meas_vel, meas_tau)

t, vel_dict, acc_dict, tau_dict = SysId.filter_data()

plot.sys_id_comparison(save_dir, meas_time, vel_dict, tau_dict, acc_dict)

param_names, term_names, p1, eq, ref_trq, est_trq = \
    SysId.analyse_plant(save_dir, vel_dict, acc_dict, tau_dict)

plot.sys_id_result(save_dir, t, ref_trq, est_trq)
