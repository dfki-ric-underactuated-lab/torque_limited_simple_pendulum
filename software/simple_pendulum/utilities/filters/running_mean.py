"""
Moving Average
==============
"""


import numpy as np
import matplotlib.pyplot as plt


def data_filter(x, N):
    cumsum = np.cumsum(np.insert(x, 0, 0))
    return (cumsum[N:] - cumsum[:-N]) / float(N)

"""""
# Example:
# Create arrays from 1 to 100 with step size 0.1, can be changed to random 
# values
time_vec = np.arrange(1, 100.1, 0.1)
torque = np.arrange(1, 100.1, 0.1)
desired_torque = np.arrange(1, 100.1, 0.1)

# Filter the arrays with the running_mean_filter
filtered_torque = running_mean_filter(np.array(torque), 10)
time_vec_filtered = running_mean_filter(np.array(time_vec), 10)
filtered_desired_torque = running_mean_filter(np.array(desired_torque), 10)

# plot the filtered data
plt.plot(time_vec_filtered, filtered_torque)
plt.plot(time_vec_filtered, filtered_desired_torque)
plt.xlabel("Time (s)")
plt.ylabel("Torque (Nm)")
plt.title("Filtered Torque (Nm) vs Time (s) with moving average filter (window = 100)")
plt.legend(['Measured', 'Desired'])
plt.show()
"""


# data_measured_list = []
def data_filter_realtime_1(data_measured_list, data_measured, window=10):
    data_measured_list.append(data_measured)
    if len(data_measured_list) > window:
        del data_measured_list[0]
    data_filtered = np.mean(data_measured_list)
    return data_filtered


def data_filter_realtime_2(i, data_measured_list, window=10):
    data_filtered = np.mean(data_measured_list[max(0, i - window):i])
    return data_filtered

