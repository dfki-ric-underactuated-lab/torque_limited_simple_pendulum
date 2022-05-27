import os
import numpy as np
import matplotlib.pyplot as plt

from simple_pendulum.trajectory_optimization.direct_collocation.direct_collocation import DirectCollocationCalculator
from simple_pendulum.model.pendulum_plant import PendulumPlant
from simple_pendulum.simulation.simulation import Simulator
from simple_pendulum.controllers.tvlqr.tvlqr import TVLQRController
from simple_pendulum.controllers.lqr.lqr_controller import LQRController
from simple_pendulum.utilities.process_data import prepare_trajectory
from simple_pendulum.controllers.tvlqr.roa.probabilistic import TVprobRhoVerification, TVprobRhoComputation
from simple_pendulum.controllers.tvlqr.roa.plot import plotFirstLastEllipses, plotFunnel, plotRhoEvolution
from simple_pendulum.controllers.lqr.roa.sos import SOSequalityConstrained

log_dir = "log_data/direct_collocation"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)

# pendulum parameters
mass = 0.57288
length = 0.5
damping = 0.15
gravity = 9.81
coulomb_fric = 0.0
torque_limit = 1.5

# swingup parameters
x0 = [0.0, 0.0]
goal = [np.pi, 0.0]

# direct collocation parameters, N is chosen also to be the number of knot points
N = 30 
max_dt = 0.1

#######################################################
# Compute the nominal trajectory via direct collocation
#######################################################

dircal = DirectCollocationCalculator()
dircal.init_pendulum(mass=mass,
                     length=length,
                     damping=damping,
                     gravity=gravity,
                     torque_limit=torque_limit)
x_trajectory, dircol, result = dircal.compute_trajectory(N=N,
                                                         max_dt=max_dt,
                                                         start_state=x0,
                                                         goal_state=goal)
T, X, XD, U = dircal.extract_trajectory(x_trajectory, dircol, result, N=N)

# save results
csv_data = np.vstack((T, X, XD, U)).T
csv_path = os.path.join(log_dir, "trajectory.csv")
np.savetxt(csv_path, csv_data, delimiter=',',
           header="time,pos,vel,torque", comments="")

# load results
csv_path = "log_data/direct_collocation/trajectory.csv"
data_dict = prepare_trajectory(csv_path)
trajectory = np.loadtxt(csv_path, skiprows=1, delimiter=",")
time = trajectory.T[0].T
dt = time[1]-time[0]
x0_traj = [trajectory.T[1].T, trajectory.T[2].T]

##################################
# RoA computation and Funnels plot
##################################

pendulum = PendulumPlant(mass=mass,
                         length=length,
                         damping=damping,
                         gravity=gravity,
                         coulomb_fric=coulomb_fric,
                         inertia=None,
                         torque_limit=torque_limit)

sim = Simulator(plant=pendulum)

controller = TVLQRController(data_dict=data_dict, mass=mass, length=length,
                             damping=damping, gravity=gravity,
                             torque_limit=torque_limit)

# Taking the finals values of S and rho from the invariant case, SOS method has been chosen
(rhof, Sf) = SOSequalityConstrained(pendulum,LQRController(mass=mass,
                                                            length=length,
                                                            damping=damping,
                                                            gravity=gravity,
                                                            torque_limit=torque_limit))
controller.set_Qf(Sf)
controller.set_goal(goal)
S_t = controller.tvlqr.S

# Application of the algorithm for time-variand RoA estimation
nSimulations = 50
(rho, ctg) = TVprobRhoComputation(pendulum, controller, x0_traj, time, N, nSimulations, rhof)
print("The final rho is: "+str(rho))

# Plotting the evolution of rho
plotRhoEvolution(rho, x0_traj, time, N)

# 2d Funnel plot
plotFunnel(rho, S_t, x0_traj, time)

# First and Last ellipses plot
plotFirstLastEllipses(rho, x0, goal, x0_traj, S_t, time)

##################
# RoA verification
##################

print("---")
nVerifications = 50
tests = [0,12,24] # knot points to be verified
for knotVerified in tests:
    print(f"Verifying knot number {knotVerified} ...")
    TVprobRhoVerification(pendulum, controller, rho, x0_traj, time, nVerifications, knotVerified)
    
plt.show()