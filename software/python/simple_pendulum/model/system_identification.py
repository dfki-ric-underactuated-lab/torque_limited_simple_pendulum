"""
System Identification
=====================
"""


# Global imports
import numpy as np
import os
from scipy import optimize

# Local imports
from simple_pendulum.model.parameters import earth
from simple_pendulum.utilities.filters import fast_fourier_transform as fft
from simple_pendulum.utilities.filters import butterworth as butter


class SystemIdentification:
    def __init__(self, meas_time, meas_pos, meas_vel, meas_tau):

        self.meas_time = meas_time
        self.meas_pos = meas_pos
        self.meas_vel = meas_vel
        self.meas_tau = meas_tau

    def filter_data(self):
        t = self.meas_time

        # filter measured velocities
        vel_fft = fft.scipy_fft(self.meas_vel, 200)
        vel_butter = butter.data_filter(self.meas_vel, order=3, cutoff=0.2)
        vel_grad = np.gradient(self.meas_pos, t)
        vel_grad_butter = np.gradient(self.meas_pos, t)

        # get acceleration from filtered velocities
        acc = np.gradient(self.meas_vel, t)
        acc_butter = butter.data_filter(acc, order=3, cutoff=0.5)
        acc_grad_butter = np.gradient(vel_butter, t)
        acc_vel_grad_butter = np.gradient(vel_butter, t)
        acc_grad_2butter = butter.data_filter(acc_vel_grad_butter, order=3,
                                              cutoff=0.1)

        # filter measured torques
        tau_fft = fft.scipy_fft(self.meas_tau, 50)
        tau_butter = butter.data_filter(self.meas_tau, order=3, cutoff=0.1)

        # store data in dictonaries
        vel_dict = {"vel_raw": self.meas_vel,
                    "vel_fft": vel_fft,
                    "vel_butter": vel_butter,
                    "vel_grad":  vel_grad,
                    "vel_grad_butter": vel_grad_butter}
        acc_dict = {"acc_raw": acc,
                    "acc_butter": acc_butter,
                    "acc_grad_butter":  acc_grad_butter,
                    "acc_grad_2butter": acc_grad_2butter}
        tau_dict = {"tau_raw": self.meas_tau,
                    "tau_fft": tau_fft,
                    "tau_butter": tau_butter}

        return t, vel_dict, acc_dict, tau_dict
        # Plots ...

    def errfunc_with_friction(p, ref_trq, phi):
        """
        phi.dot(p) = est_trq
        p = parameters at each timestep
        """
        return ref_trq - phi.dot(p)

    def analyse_plant(self, output_path, vel_dict, acc_dict, tau_dict):
        plant = "Simple Pendulum"
        g = earth.gravity
        ref_pos = self.meas_pos
        ref_vel = vel_dict["vel_grad_butter"]
        ref_acc = acc_dict["acc_grad_2butter"]
        ref_trq = tau_dict["tau_butter"]

        # identification matrix R (num_samples x num_params)
        num_samples = len(self.meas_time)
        num_params = 5                      # all 5 terms contribute to est_trq
        phi = np.empty((num_samples, num_params))
        phi[:, 0] = ref_acc
        phi[:, 1] = np.sin(ref_pos)*g
        phi[:, 2] = ref_vel
        phi[:, 3] = np.arctan(100.0 * ref_vel)
        phi[:, 4] = 1.0
        term_names = ["qdotdot", "g*sin(q)", "qdot", "atan(100*qdot)", "1"]
        r = np.linalg.matrix_rank(phi, tol=0.1)
        print("rank:", r)

        # Solve least_squares optimization problem
        param_names = ["J", "m*cx", "Fv", "Fc", "offset"]
        # (Inertia (J), mass*center of mass (m*cx), Viscous Friction (Fv), Coulomb Friction (Fc) and offset terms

        p0 = [0.0, 1.0, 0.0, 0.0, 0.0]                # initial parameter guess
        bounds = ([0.0, -np.Inf, 0.0, 0.0, -1.0000000011], [np.Inf, np.Inf,
                                                            np.Inf, np.Inf,
                                                            1.000000001])
        optres = optimize.least_squares(
                            fun=SystemIdentification.errfunc_with_friction,
                            x0=p0[:], args=(ref_trq, phi),
                            max_nfev=100000, bounds=bounds)
        p1 = optres.x
        success = optres.success
        print("success:", success)
        est_trq = phi.dot(p1)

        # print outs
        for i in range(len(param_names)):
            print("{:10s} = {:+.3e}".format(param_names[i], p1[i]))
        eq = "tau = "
        for i in range(len(param_names)):
            if i != 0:
                eq += " + "
            eq += "{} * {}".format(param_names[i], term_names[i])
        # tau = J*(qdotdot) + m*cx*(g*sin(q)) + Fv*(qdot) + Fc*(atan(100*qdot))
        #       + offset*(1)
        print(eq)

        # save print outs in .txt file
        file_name = "system_identification.txt"
        if not os.path.exists(output_path):
            os.makedirs(output_path)
        print_path = os.path.join(output_path, file_name)
        """
        print_file = open(print_path, "w")
        print_file.write("System Identification: ")
        print_file.close()
        """

        with open(print_path, "w") as f:
            print("System Identification: ", plant, file=f)
            for i in range(len(param_names)):
                print("{:10s} = {:+.3e}".format(param_names[i], p1[i]), file=f)
            print(eq, file=f)

        return param_names, term_names, p1, eq, ref_trq, est_trq
