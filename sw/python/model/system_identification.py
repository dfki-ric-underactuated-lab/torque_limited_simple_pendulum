# global imports
import numpy as np
import yaml
import os
from pathlib import Path

# local imports
# Set path for local imports
#import site
#site.addsitedir('../..')

from filters import fast_fourier_transform as fft
from filters import butterworth as butter

from utilities import plots

def errfunc_with_friction(p, trq, phi):
    return trq - phi.dot(p)


def analyse_plant(meas_time, meas_pos, meas_vel, meas_tau):
    t = meas_time

    # filter measured velocities
    vel_fft = fft.scipy_fft(meas_vel, 200)
    vel_butter = butter.data_filter(meas_vel, order=3, cutoff=0.2)
    vel_grad = np.gradient(meas_pos, t)
    vel_grad_butter = np.gradient(meas_pos, t)

    # filter measured torques
    tau_fft = fft.scipy_fft(meas_tau, 50)
    tau_butter = butter.data_filter(meas_tau, order=3, cutoff=0.1)

    # get acceleration from filtered velocities
    acc = np.gradient(meas_vel, t)
    acc_grad_butter = butter.data_filter(acc, order=3, cutoff=0.5)          # 1
    acc_butter = np.gradient(vel_butter, t)                                 # 2
    acc_vel_grad_butter = np.gradient(vel_butter, t)
    acc_grad_2butter = butter.data_filter(acc_vel_grad_butter, order=3,     # 3
                                           cutoff=0.1)


    # Plots ...

# ident matrix R num_samples x num_params
num_samples = len(t)
num_params = 5
g = 9.81

phi = np.empty((num_samples, num_params))
phi[:,0] = ref_acc
phi[:,1] = np.sin(ref_pos)*g
phi[:,2] = ref_vel
phi[:,3] = np.arctan(100.0 * ref_vel)
phi[:,4] = 1.0

param_names = ["J", "m*cx", "Fv", "Fc", "offset"]
term_names = ["qdotdot", "g*sin(q)", "qdot", "atan(100*qdot)", "1"]

r = np.linalg.matrix_rank( phi, tol=0.1 )
print("rank:", r)
#%%
p0 = [0.0, 1.0, 0.0, 0.0, 0.0] # Initial guess for the parameters
bounds = ([0.0,-np.Inf,0.0,0.0,-1.0000000011],[np.Inf,np.Inf,np.Inf,np.Inf,1.000000001])
optres = optimize.least_squares(fun=errfunc_with_friction, x0=p0[:], args=(ref_trq, phi), max_nfev=100000, bounds=bounds)
p1 = optres.x
success = optres.success
print("success:", success)

for i in range(len(param_names)):
    print("{:10s} = {:+.3e}".format(param_names[i], p1[i]))


eq = "tau = "
for i in range(len(param_names)):
    if i != 0:
        eq += " + "
    eq += "{} * {}".format(param_names[i], term_names[i])

print(eq)

#%%

est_trq = phi.dot(p1)
plt.figure("est compare", clear=True)

plt.plot(t, ref_trq)
plt.plot(t, est_trq)

plt.legend(["trq meas", "trqest"] )
plt.show()

if __name__ == '__main__':
    # default parameters, can be changed
    csv_file = "data_measured"
    csv_path = os.path.join(Path(__file__).parents[4],
                            'results/20210611-040141-PM_pd_control/' + csv_file)

    # load results from csv file
    trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
    meas_time = trajectory.T[0].T       # desired time in s
    meas_pos = trajectory.T[1].T        # desired position in radian
    meas_vel = trajectory.T[2].T        # desired velocity in radian/s
    meas_tau = trajectory.T[3].T        # desired torque in Nm


